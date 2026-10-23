#include "../../p62_r14b_newer_college/conditional_base_pilot/tools/common.hpp"
#include <Eigen/Eigenvalues>
#include <sys/stat.h>
#include <set>

namespace r16screen {
using namespace pilot;
using coverage::Row;
struct Frame {
  int index=0;
  int64_t stamp=0;
  std::string path;
  size_t bytes=0;
};
struct Format { size_t points=0,header=0,step=0,tail=0; };
struct Metric {
  Frame frame;
  bool valid=false;
  std::string stage,status;
  size_t raw_points=0,source_points=0,training_points=0;
  Eigen::Vector3d eigen=Eigen::Vector3d::Constant(NAN);
  double ratio=NAN,linearity=NAN,load_ms=NAN,preprocess_ms=NAN;
  std::string source_fnv;
};

inline std::vector<std::string> words(const std::string& line){
  std::istringstream in(line);std::vector<std::string> out;std::string value;
  while(in>>value)out.push_back(value);return out;
}
inline Format inspect(const Frame& f){
  struct stat st{};require(::lstat(f.path.c_str(),&st)==0&&S_ISREG(st.st_mode),"PCD is absent, symlink, or non-regular");
  require(size_t(st.st_size)==f.bytes,"inventory size mismatch");
  std::ifstream input(f.path,std::ios::binary);require(bool(input),"PCD open failed");
  std::map<std::string,std::vector<std::string>> fields;std::string line;size_t header=0;bool found=false;
  for(int count=0;count<64;++count){
    require(bool(std::getline(input,line)),"PCD header truncated");
    if(!line.empty()&&line.back()=='\r')line.pop_back();
    if(line.empty()||line[0]=='#')continue;
    auto items=words(line);if(items.empty())continue;
    fields[items[0]]=std::vector<std::string>(items.begin()+1,items.end());
    if(items[0]=="DATA"){header=size_t(input.tellg());found=true;break;}
  }
  require(found,"PCD DATA header missing");
  const std::array<std::vector<std::string>,7> expected{{
    {"x","y","z","intensity","t","reflectivity","ring","noise","range"},
    {"4","4","4","4","4","2","1","2","4"},
    {"F","F","F","F","U","U","U","U","U"},
    {"1","1","1","1","1","1","1","1","1"},
    {"1024"},{"64"},{"binary"}}};
  const std::array<std::string,7> keys{{"FIELDS","SIZE","TYPE","COUNT","WIDTH","HEIGHT","DATA"}};
  for(size_t i=0;i<keys.size();++i)require(fields[keys[i]]==expected[i],"PCD schema differs from frozen Ouster layout");
  require(fields["POINTS"].size()==1,"PCD POINTS missing");
  Format out;out.points=std::stoull(fields["POINTS"][0]);out.header=header;out.step=29;
  require(out.points==std::stoull(fields["WIDTH"][0])*std::stoull(fields["HEIGHT"][0]),"PCD dimensions inconsistent");
  const size_t declared=out.points*out.step;
  require(f.bytes>=out.header+declared,"PCD payload truncated");out.tail=f.bytes-out.header-declared;
  input.clear();input.seekg(std::streamoff(out.header+declared));std::array<char,4096> buffer{};size_t remaining=out.tail;
  while(remaining){const size_t n=std::min(remaining,buffer.size());input.read(buffer.data(),std::streamsize(n));require(size_t(input.gcount())==n,"PCD tail truncated");
    for(size_t i=0;i<n;++i)require(buffer[i]==0,"nonzero PCD suffix outside declared records");remaining-=n;}
  return out;
}
inline Metric measure(const Frame& f,const std::string& stage){
  Metric m;m.frame=f;m.stage=stage;const Format fmt=inspect(f);m.raw_points=fmt.points;
  Row row;row["path"]=f.path;row["points"]=std::to_string(fmt.points);row["point_step"]=std::to_string(fmt.step);row["header_bytes"]=std::to_string(fmt.header);
  const Source s=pilot::loadSource(row);m.source_points=s.points->size();m.training_points=s.split.training->size();
  require(m.source_points>=2,"preprocessed input has too few finite points");
  Eigen::Vector3d mean=Eigen::Vector3d::Zero();
  for(const auto& p:*s.points)mean+=Eigen::Vector3d(p.x,p.y,p.z);mean/=double(s.points->size());
  Eigen::Matrix3d covariance=Eigen::Matrix3d::Zero();
  for(const auto& p:*s.points){const Eigen::Vector3d d=Eigen::Vector3d(p.x,p.y,p.z)-mean;covariance.noalias()+=d*d.transpose();}
  covariance/=double(s.points->size()-1);Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(covariance);
  require(eigen.info()==Eigen::Success&&eigen.eigenvalues().allFinite(),"raw geometry eigensolver failed");
  m.eigen=eigen.eigenvalues();require(m.eigen[2]>0,"raw geometry covariance has zero extent");
  m.ratio=m.eigen[0]/m.eigen[2];m.linearity=m.eigen[1]/m.eigen[2];m.source_fnv=cloudHash(s.points);
  m.load_ms=s.load_ms;m.preprocess_ms=s.preprocess_ms;m.valid=std::isfinite(m.ratio)&&m.ratio>=0;
  m.status=m.valid?"OK":"INVALID_SCORE";return m;
}
inline void emit(std::ostream& out,const Metric& m){
  out<<m.frame.index<<','<<m.frame.stamp<<','<<m.frame.path<<','<<m.frame.bytes<<','<<m.stage<<','<<m.raw_points<<','
     <<m.source_points<<','<<m.training_points<<','<<std::setprecision(17)<<m.eigen[0]<<','<<m.eigen[1]<<','<<m.eigen[2]<<','
     <<m.ratio<<','<<m.linearity<<','<<m.source_fnv<<','<<m.load_ms<<','<<m.preprocess_ms<<','<<m.status<<'\n';
}
}

int main(int argc,char** argv){using namespace r16screen;try{
  require(argc==4,"prescreen INVENTORY.csv OUTPUT_DIR SOURCE_ROOT");
  require(::mkdir(argv[2],0775)==0,"prescreen output must be new; no overwrite");
  const auto rows=coverage::readCsv(argv[1]);require(rows.size()==15301,"canonical inventory must contain 15,301 scans");
  std::vector<Frame> frames;frames.reserve(rows.size());
  for(size_t i=0;i<rows.size();++i){Frame f;f.index=int(i);f.stamp=std::stoll(rows[i].at("filename_stamp_ns"));f.path=rows[i].at("path");f.bytes=std::stoull(rows[i].at("size_bytes"));
    require(i==0||f.stamp>frames.back().stamp,"inventory timestamps not strictly increasing");require(f.path.find(argv[3])==0,"scan path outside frozen source root");frames.push_back(f);}
  const int64_t first=frames.front().stamp,last=frames.back().stamp,span=last-first;
  int eligible=0;while(eligible<int(frames.size())&&20*(frames[eligible].stamp-first)<span)++eligible;
  require(eligible+31<int(frames.size()),"not enough post-5-percent frames");
  constexpr int stride=20,window=31,cap=800;std::map<int,Metric> cache;std::set<int> read_ids;
  auto get=[&](int i,const std::string& stage)->Metric&{
    auto it=cache.find(i);if(it!=cache.end()){if(stage=="WINDOW")it->second.stage="SAMPLED_AND_WINDOW";return it->second;}
    require(read_ids.insert(i).second,"duplicate read accounting");Metric m=measure(frames[i],stage);return cache.emplace(i,std::move(m)).first->second;};
  std::vector<int> samples;for(int i=eligible;i+15<int(frames.size());i+=stride)samples.push_back(i);
  require(!samples.empty()&&samples.size()<=cap,"invalid fixed-stride sample count");
  int winner=-1;double best=std::numeric_limits<double>::infinity();
  for(int i:samples){Metric& m=get(i,"SAMPLED");if(m.valid&&(m.ratio<best||(m.ratio==best&&(winner<0||i<winner)))){winner=i;best=m.ratio;}}
  require(winner>=0,"no valid sampled input geometry; stop without alternate selection");
  const int start=std::max(eligible,std::min(int(frames.size())-window,winner-window/2));
  const int end=start+window;for(int i=start;i<end;++i)get(i,"WINDOW");
  require(read_ids.size()<=cap,"unique PCD reads exceeded frozen 800 cap");
  const std::string outdir=argv[2];std::ofstream csv(outdir+"/PRESCREEN_GEOMETRY.csv");require(bool(csv),"cannot create prescreen CSV");
  csv<<"raw_ordinal_one_based,stamp_ns,path,file_bytes,stage,raw_points,source_points,training_points,lambda_min,lambda_mid,lambda_max,lambda_min_over_max,lambda_mid_over_max,source_fnv,load_ms,preprocess_ms,status\n";
  for(const auto& kv:cache)emit(csv,kv.second);csv.close();require(bool(csv),"prescreen CSV write failed");
  std::ofstream summary(outdir+"/PRESCREEN_SUMMARY.json");require(bool(summary),"cannot create summary");
  const int64_t threshold=first+(span+19)/20;const size_t valid=std::count_if(cache.begin(),cache.end(),[](const auto& kv){return kv.second.valid;});
  summary<<std::setprecision(17)<<"{\n  \"inventory_frames\": "<<frames.size()<<",\n  \"prescreen_stride\": "<<stride<<",\n  \"sampled_scans\": "<<samples.size()<<",\n  \"unique_pcd_reads_including_window\": "<<read_ids.size()<<",\n  \"read_cap\": "<<cap<<",\n  \"first_5pct_eligible_index_zero_based\": "<<eligible<<",\n  \"first_5pct_threshold_ns\": "<<threshold<<",\n  \"geometry_valid_rows\": "<<valid<<",\n  \"selection_metric\": \"scanner-frame preprocessed XYZ covariance lambda_min/lambda_max; raw-input prescreen proxy only\",\n  \"winner_sample_index_zero_based\": "<<winner<<",\n  \"winner_raw_ordinal_one_based\": "<<winner+1<<",\n  \"winner_score\": "<<best<<",\n  \"window_start_index_zero_based\": "<<start<<",\n  \"window_end_index_exclusive\": "<<end<<",\n  \"window_raw_ordinals_one_based\": [";
  for(int i=start;i<end;++i){if(i>start)summary<<',';summary<<i+1;}summary<<"],\n  \"window_stamps_ns\": [";
  for(int i=start;i<end;++i){if(i>start)summary<<',';summary<<frames[i].stamp;}summary<<"],\n  \"ndt_align_calls\": 0,\n  \"gt_pose_values_read\": 0,\n  \"map_accessed\": false,\n  \"gt_error_accessed\": false\n}\n";
  summary.close();require(bool(summary),"summary write failed");
  std::cout<<"PRESCREEN_COMPLETE sampled="<<samples.size()<<" unique_reads="<<read_ids.size()<<" winner_raw="<<winner+1<<" window="<<start+1<<"-"<<end<<" proxy="<<best<<" NDT=0 GT_values=0"<<std::endl;
}catch(const std::exception& e){std::cerr<<"PRESCREEN_STOP "<<e.what()<<std::endl;return 1;}}
