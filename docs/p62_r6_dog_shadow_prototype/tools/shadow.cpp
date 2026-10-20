#include "native_kernel.hpp"
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <iostream>
#include <map>
#include <sys/stat.h>
#include <sys/resource.h>
using namespace shadow;
using Row=std::map<std::string,std::string>;
struct Csv {
  std::ofstream stream;
  Csv(const std::string& path,const std::string& header):stream(path){require(bool(stream),"CSV creation failed");stream<<std::setprecision(17)<<header<<'\n';stream.flush();}
  void end(){stream<<'\n';stream.flush();}
};
std::vector<Row> readCsv(const std::string& path){
  std::ifstream f(path);std::string line;require(bool(std::getline(f,line)),"missing frozen scan CSV");const auto keys=csvFields(line);std::vector<Row> rows;
  while(std::getline(f,line)){auto vals=csvFields(line);require(vals.size()==keys.size(),"scan CSV width");Row r;for(size_t i=0;i<keys.size();++i)r[keys[i]]=vals[i];rows.push_back(r);}return rows;
}
Pose prediction(const Row& r){Pose p=Pose::Identity();Eigen::Quaterniond q(std::stod(r.at("qw")),std::stod(r.at("qx")),std::stod(r.at("qy")),std::stod(r.at("qz")));
  p.block<3,3>(0,0)=q.normalized().toRotationMatrix();p.block<3,1>(0,3)=Eigen::Vector3d(std::stod(r.at("tx")),std::stod(r.at("ty")),std::stod(r.at("tz")));require(rigid(p),"invalid common prediction");return p;}
Cloud::Ptr loadSource(const Row& r){
  std::ifstream f(r.at("raw_xyz_offset_file"),std::ios::binary);require(bool(f),"raw source missing");Cloud::Ptr c(new Cloud);std::array<float,3> xyz;uint32_t offset;
  const int n=std::stoi(r.at("raw_points"));
  for(int i=0;i<n;++i){f.read(reinterpret_cast<char*>(xyz.data()),12);f.read(reinterpret_cast<char*>(&offset),4);require(bool(f),"truncated raw source");
    if(!std::isfinite(xyz[0])||!std::isfinite(xyz[1])||!std::isfinite(xyz[2]))continue;
    const double range=std::sqrt(xyz[0]*xyz[0]+xyz[1]*xyz[1]+xyz[2]*xyz[2]);
    if(range>=.5&&range<=80)c->push_back(pcl::PointXYZ(xyz[0],xyz[1],xyz[2]));}
  require(f.peek()==std::char_traits<char>::eof(),"unexpected source suffix");finish(c);return voxel(c,.25f,1400);
}
void poseHeader(std::string& s,const std::string& prefix){for(int i=0;i<4;++i)for(int j=0;j<4;++j)s+=","+prefix+std::to_string(i)+std::to_string(j);}
void emitPose(std::ostream& o,const Pose& p){for(int i=0;i<4;++i)for(int j=0;j<4;++j)o<<','<<p(i,j);}
int main(int argc,char** argv){try{
  require(argc==4,"usage: shadow FROZEN_SCANS_CSV FROZEN_MAP NEW_OUTPUT_DIR");require(::mkdir(argv[3],0775)==0,"refuse existing output directory");const std::string root=argv[3];
  const auto rows=readCsv(argv[1]);require(rows.size()==100,"not frozen100 scans");
  Csv inputs(root+"/input_preflight.csv","scan_index,source_points,source_fnv,training_points,val0,val1,val2,val3,val4,val5,val6,val7,evaluable");
  std::vector<Cloud::Ptr> sources;std::vector<Split> splits;std::vector<double> preprocess_times;
  for(const auto& r:rows){const auto ps=Clock::now();auto c=loadSource(r);auto s=splitSource(c);prediction(r);preprocess_times.push_back(milliseconds(ps));
    require(c->size()==std::stoul(r.at("expected_processed_points"))&&cloudHash(c)==r.at("expected_processed_fnv"),"production Source parity failed BEFORE align");
    inputs.stream<<r.at("scan_index")<<','<<c->size()<<','<<cloudHash(c)<<','<<s.training->size();for(auto& b:s.blocks)inputs.stream<<','<<b->size();inputs.stream<<','<<s.evaluable();inputs.end();
    sources.push_back(c);splits.push_back(s);
  }
  std::cout<<"INPUT_PREFLIGHT_PASS scans=100 native_aligns=0"<<std::endl;
  const auto build_start=Clock::now();Cloud::Ptr raw(new Cloud),finite_map(new Cloud);
  require(pcl::io::loadPCDFile(argv[2],*raw)==0,"map load failed");for(const auto& p:*raw)if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))finite_map->push_back(p);finish(finite_map);
  auto target=voxel(voxel(finite_map,.15f),.15f);NativeNdt ndt;ndt.configure(target);pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(target);
  Csv map(root+"/map_runtime.csv","original_points,target_points,target_fnv,actual_grid_m,map_load_grid_kdtree_ms");map.stream<<raw->size()<<','<<target->size()<<','<<cloudHash(target)<<",0.8,"<<milliseconds(build_start);map.end();
  std::string candidate_header="scan_index,candidate_id,kind,duplicate_of,finite,converged,iterations,native_mean_score,fitness,translation_from_prediction_m,rotation_from_prediction_deg,heldout_overlap_0p5m,mean_block_loss,eligible,align_ms,validation_ms";poseHeader(candidate_header,"seed_");poseHeader(candidate_header,"final_");
  Csv candidates(root+"/CANDIDATE_POOL.csv",candidate_header),blocks(root+"/HELDOUT_BLOCK_LOSSES.csv","scan_index,candidate_id,block_id,points,loss,NN_queries");
  Csv selection(root+"/SELECTOR_COMPARISON.csv","scan_index,selector,pool_ids,selected_id,outcome,min_pairwise_advantage");
  Csv frame(root+"/SHADOW_FRAME_RESULTS.csv","scan_index,header_ns,reference_ns,source_fnv,full_points,training_points,validation_evaluable,full_calls,split_calls,extra_calls,trigger,direction_status,budget_used,unique_pool_size,proposed_selected,proposed_status,lambda_min,lambda_max,G_support_points");
  Csv runtime(root+"/RUNTIME_AND_MEMORY.csv","scan_index,preprocess_ms,control_full_align_ms,control_split_align_ms,extra_align_ms,G_ms,validation_ms,selectors_ms,frame_compute_ms,process_ru_maxrss_KiB");
  Csv calls(root+"/NATIVE_ALIGN_CALLS.csv","scan_index,candidate_id,kind,phase,cumulative_attempts,budget_used");
  Budget budget;int total=0;bool smoke_inputs_valid=true;
  for(int index=0;index<100;++index){const auto& r=rows[index];const auto& source=sources[index];const auto& split=splits[index];const Pose initial=prediction(r);const auto start=Clock::now();
    const int scan=std::stoi(r.at("scan_index"));require(scan==index+1,"source order changed");
    auto run=[&](const Cloud::Ptr& cloud,const Pose& guess,int id,const char* kind){
      ++total;calls.stream<<scan<<','<<id<<','<<kind<<",START,"<<total<<','<<budget.used;calls.end();
      try{auto c=align(ndt,cloud,guess,id);calls.stream<<scan<<','<<id<<','<<kind<<",RETURNED,"<<total<<','<<budget.used;calls.end();return c;}
      catch(...){calls.stream<<scan<<','<<id<<','<<kind<<",THREW,"<<total<<','<<budget.used;calls.end();throw;}
    };
    auto full=run(source,initial,-1,"CONTROL_FULL");validate(full,split,tree,initial);
    std::vector<Candidate> pool_c;int split_calls=0,extra=0;bool requested=false,invalid_model=false;std::string direction="NOT_REQUESTED",status="NOT_EVALUABLE";double g_ms=0,extra_ms=0,lambda_min=NAN,lambda_max=NAN;size_t supported=0;
    if(full.finite&&split.training->size()>=300){++split_calls;pool_c.push_back(run(split.training,initial,0,"CONTROL_SPLIT"));validate(pool_c[0],split,tree,initial);}
    if(split.evaluable()&&!pool_c.empty()&&pool_c[0].finite){
      auto& nominal=pool_c[0];requested=trigger(nominal.converged,nominal.iterations,nominal.loss);status="ORDINARY_NO_RECOVERY";
      if(requested){status="NO_SUPPORTED_RECOVERY";
        if(budget.used>=20){direction="BUDGET_EXHAUSTED";status="BUDGET_EXHAUSTED";}
        else if(!nominal.finite){direction="INVALID_NOMINAL";}
        else{const auto gs=Clock::now();const Matrix6 g=ndt.geometry(split.training,nominal.pose,supported);Eigen::SelfAdjointEigenSolver<Matrix6> eig;if(g.allFinite())eig.compute(g);
          if(!g.allFinite()||eig.info()!=Eigen::Success){direction="INVALID_G";invalid_model=true;}
          else{lambda_min=eig.eigenvalues()[0];lambda_max=eig.eigenvalues()[5];
            if(lambda_max<=1e-12){direction="NO_INFORMATION";}
            else if(lambda_min< -1e-8*lambda_max){direction="INVALID_G_SPECTRUM";invalid_model=true;}
            else if(lambda_min/lambda_max>.01){direction="NO_WEAK_DIRECTION";}
            else{direction="WEAK_DIRECTION";const Vector6 v=eig.eigenvectors().col(0);const Pose center=nominal.pose;g_ms=milliseconds(gs);
              std::vector<Pose> seeds;for(double sign:{1.,-1.}){const Pose guess=seed(center,v,sign);bool same=false;for(const auto& old:seeds)if(translation(guess,old)<1e-10&&rotation(guess,old)<1e-8)same=true;
                if(same)continue;seeds.push_back(guess);if(!budget.consume()){status="BUDGET_EXHAUSTED";break;}
                ++extra;auto c=run(split.training,guess,int(pool_c.size()),"RECOVERY");extra_ms+=c.align_ms;validate(c,split,tree,initial);pool_c.push_back(c);if(!c.finite){invalid_model=true;direction="INVALID_RECOVERY_CANDIDATE";break;}}
            }}if(direction!="WEAK_DIRECTION")g_ms=milliseconds(gs);
        }}
    }
    deduplicate(pool_c);const auto select_start=Clock::now();const auto chosen=select(pool_c);const double selectors_ms=milliseconds(select_start);
    const auto ids=pool(pool_c);std::string pool_ids;for(int j:ids){if(!pool_ids.empty())pool_ids+='|';pool_ids+=std::to_string(j);}
    if(split.evaluable()&&chosen.robust>=0)status="SUPPORTED_RECOVERY";
    if(invalid_model)status="NUMERICAL_FAILURE_STOP";
    for(const auto& choice:std::vector<std::pair<std::string,int>>{{"NATIVE_SCORE",chosen.native},{"HELDOUT_MEAN",chosen.heldout_mean},{"ROBUST_PAIRED",chosen.robust}}){
      double margin=NAN;if(choice.second>=0){margin=1;for(int k:ids)if(k!=choice.second)margin=std::min(margin,paired(pool_c[choice.second].loss,pool_c[k].loss));}
      selection.stream<<scan<<','<<choice.first<<','<<pool_ids<<','<<choice.second<<','<<(choice.second<0?"ABSTAIN":(choice.second==0?"KEEP_NOMINAL":"SELECT_ALTERNATIVE"))<<','<<margin;selection.end();
    }
    auto record=[&](const Candidate& c,const char* kind){candidates.stream<<scan<<','<<c.id<<','<<kind<<','<<c.duplicate_of<<','<<c.finite<<','<<c.converged<<','<<c.iterations<<','<<c.score<<','<<c.fitness<<','<<translation(c.pose,initial)<<','<<rotation(c.pose,initial)<<','<<c.overlap<<','<<(c.loss_valid?mean(c.loss):NAN)<<','<<c.eligible<<','<<c.align_ms<<','<<c.validation_ms;emitPose(candidates.stream,c.initial);emitPose(candidates.stream,c.pose);candidates.end();
      for(int b=0;b<8;++b){blocks.stream<<scan<<','<<c.id<<','<<b<<','<<split.blocks[b]->size()<<','<<c.loss[b]<<','<<(c.loss_valid?split.blocks[b]->size():0);blocks.end();}};
    record(full,"CONTROL_FULL");for(const auto& c:pool_c)record(c,c.id==0?"CONTROL_SPLIT_NOMINAL":"RECOVERY");
    frame.stream<<scan<<','<<r.at("header_ns")<<','<<r.at("reference_ns")<<','<<cloudHash(source)<<','<<source->size()<<','<<split.training->size()<<','<<split.evaluable()<<",1,"<<split_calls<<','<<extra<<','<<requested<<','<<direction<<','<<budget.used<<','<<ids.size()<<','<<chosen.robust<<','<<status<<','<<lambda_min<<','<<lambda_max<<','<<supported;frame.end();
    double validation_ms=full.validation_ms;for(const auto& c:pool_c)validation_ms+=c.validation_ms;struct rusage usage;getrusage(RUSAGE_SELF,&usage);
    runtime.stream<<scan<<','<<preprocess_times[index]<<','<<full.align_ms<<','<<(pool_c.empty()?NAN:pool_c[0].align_ms)<<','<<extra_ms<<','<<g_ms<<','<<validation_ms<<','<<selectors_ms<<','<<milliseconds(start)<<','<<usage.ru_maxrss;runtime.end();
    require(extra<=2&&budget.used<=20,"real align budget violated");require(!invalid_model,"invalid G or recovery candidate (record preserved)");require(full.finite&&!pool_c.empty()&&pool_c[0].finite,"nonfinite native state");
    if(index<20)smoke_inputs_valid=smoke_inputs_valid&&split.evaluable();
    if(index==19){require(smoke_inputs_valid,"20frame smoke validation input not evaluable");std::cout<<"SMOKE20_PASS no_feedback=1 budget_used="<<budget.used<<std::endl;}
    std::cout<<"FRAME "<<scan<<" status="<<status<<" extra="<<extra<<" total_budget="<<budget.used<<std::endl;
  }
  require(total==200+budget.used,"native align accounting");std::cout<<"COMPLETED frames=100 native_aligns="<<total<<" extra="<<budget.used<<" feedback=0"<<std::endl;
}catch(const std::exception& error){std::cerr<<"STOP "<<error.what()<<std::endl;return 1;}}
