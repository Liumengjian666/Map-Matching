#pragma once
#include "coverage.hpp"
#include "../../p62_r8_structural_admission/tools/preprocess.hpp"
#include <fstream>
#include <map>
#include <sys/stat.h>

namespace coverage {
using Row=std::map<std::string,std::string>;
struct Csv {
  std::ofstream stream;
  Csv(const std::string& path,const std::string& header):stream(path){require(bool(stream),"CSV open");stream<<std::setprecision(17)<<header<<'\n';stream.flush();}
  void end(){stream<<'\n';stream.flush();}
};
inline std::vector<Row> readCsv(const std::string& path){std::ifstream f(path);std::string line;require(bool(std::getline(f,line)),"frozen CSV missing");auto keys=csvFields(line);std::vector<Row> rows;
  while(std::getline(f,line)){auto vals=csvFields(line);require(vals.size()==keys.size(),"CSV width");Row r;for(size_t i=0;i<keys.size();++i)r[keys[i]]=vals[i];rows.push_back(r);}return rows;}
inline Pose prediction(const Row& r){Pose p=Pose::Identity();Eigen::Quaterniond q(std::stod(r.at("qw")),std::stod(r.at("qx")),std::stod(r.at("qy")),std::stod(r.at("qz")));
  p.block<3,3>(0,0)=q.normalized().toRotationMatrix();p.block<3,1>(0,3)=Eigen::Vector3d(std::stod(r.at("tx")),std::stod(r.at("ty")),std::stod(r.at("tz")));require(rigid(p),"invalid common prediction");return p;}
inline Cloud::Ptr loadSource(const Row& r){std::ifstream f(r.at("raw_xyz_offset_file"),std::ios::binary);require(bool(f),"raw missing");auto c=preprocessRaw(f,std::stoul(r.at("raw_points")));require(f.peek()==std::char_traits<char>::eof(),"raw suffix");return c;}
inline void number(std::ostream& o,double v){if(std::isfinite(v))o<<v;else o<<"NA";}
inline void poseHeader(std::string& s,const std::string& prefix){for(int i=0;i<4;++i)for(int j=0;j<4;++j)s+=","+prefix+std::to_string(i)+std::to_string(j);}
inline void emitPose(std::ostream& o,const Pose& p){for(int i=0;i<4;++i)for(int j=0;j<4;++j)o<<','<<p(i,j);}
inline void coverageRow(Csv& out,const Row& row,const Cloud::Ptr& c,const Split& s,const Mask& m){
  std::string mask;for(int b=0;b<8;++b)mask+=s.blocks[b]->size()>=10?'1':'0';
  out.stream<<row.at("scan_index")<<','<<row.at("raw_index")<<','<<row.at("seq")<<','<<row.at("header_ns")<<','<<c->size()<<','<<cloudHash(c)<<','<<s.training->size()<<','<<mask<<','<<m.active.size()<<','<<m.points<<','<<8-m.active.size()<<','<<m.active.size()*22.5<<',';
  number(out.stream,m.all_validation?double(m.all_validation-m.points)/m.all_validation:NAN);out.stream<<','<<double(m.all_validation-m.points)/c->size()<<','<<s.evaluable()<<','<<m.evaluable();
  for(const auto& b:s.blocks)out.stream<<','<<b->size();out.end();
}
inline const char* coverageHeader(){return "scan_index,raw_index,seq,header_ns,source_points,source_fnv,training_points,active_mask,B,active_validation_points,inactive_blocks,validation_sector_angle_deg,excluded_validation_fraction,excluded_source_fraction,V1_evaluable,V2_evaluable,val0,val1,val2,val3,val4,val5,val6,val7";}
} // namespace coverage
