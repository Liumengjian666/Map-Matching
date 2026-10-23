#include "../../p62_r14b_newer_college/conditional_base_pilot/tools/common.hpp"
#include "../../p62_r17_consensus_admission/tools/policy.hpp"
#include <iostream>

using namespace pilot;

static Pose fromRow(const Row& row,const char* prefix){
  Pose p;
  for(int i=0;i<4;++i)for(int j=0;j<4;++j)p(i,j)=std::stod(row.at(std::string(prefix)+std::to_string(i)+std::to_string(j)));
  require(rigid(p),"stored raw terminal pose invalid");return p;
}

int main(int argc,char** argv){try{
  require(argc==7,"probe_stability PLAN.csv SCANS.csv MAP.pcd MAP_BUILD.csv NDT_RESULTS.csv NEW_OUTPUT_DIR");
  require(::mkdir(argv[6],0775)==0,"exclusive stability probe directory");
  const auto plan=readCsv(argv[1]),scans=readCsv(argv[2]),prior=readCsv(argv[5]),map_receipt=readCsv(argv[4]);
  require(plan.size()==6&&scans.size()==61&&prior.size()==240&&map_receipt.size()==1,"stability probe frozen row counts");
  Cloud::Ptr map(new Cloud);require(pcl::io::loadPCDFile(argv[3],*map)==0,"probe map unreadable");finish(map);
  require(map->size()==std::stoul(map_receipt[0].at("target_points"))&&cloudHash(map)==map_receipt[0].at("target_fnv"),"probe target identity changed");
  NativeNdt ndt;ndt.configure(map);pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(map);
  Csv out(std::string(argv[6])+"/STABILITY_PROBE.csv","role,arm,frame,raw_ordinal,stamp_ns,seed_from_frozen_terminal,original_native_score,reoptimized_native_score,reoptimized_finite,reoptimized_converged,reoptimized_iterations,reoptimized_overlap,reoptimized_quality,translation_from_original_m,rotation_from_original_deg,align_ms,validation_ms,peak_rss_KiB");
  int aligns=0;
  for(const Row& request:plan){
    const int frame=std::stoi(request.at("frame"));require(frame>=1&&frame<=60,"probe frame outside frozen window");
    const auto& scan=scans[frame];require(scan.at("raw_ordinal")==request.at("raw_ordinal")&&scan.at("stamp_ns")==request.at("stamp_ns"),"probe scan identity differs");
    const Row* frozen=nullptr;
    for(const auto& r:prior)if(r.at("arm")==request.at("arm")&&r.at("frame")==request.at("frame")){
      require(!frozen,"duplicate frozen terminal row");frozen=&r;}
    require(frozen&&frozen->at("role")==request.at("role")&&frozen->at("raw_ordinal")==request.at("raw_ordinal"),"frozen terminal identity differs");
    require(frozen->at("quality_feedback")=="1"&&frozen->at("converged")=="1","probe requires originally valid terminal");
    const Pose seed=fromRow(*frozen,"raw");const Source source=pilot::loadSource(scan);
    require(source.points->size()==std::stoul(scan.at("expected_source_points"))&&cloudHash(source.points)==scan.at("expected_source_fnv"),"probe source changed");
    ++aligns;Candidate c=capturedAlign(ndt,source.points,seed,0,[](const Candidate&){});
    if(c.finite)coverage::validate(c,source.split,source.mask,tree,seed);
    out.stream<<request.at("role")<<','<<request.at("arm")<<','<<frame<<','<<request.at("raw_ordinal")<<','<<request.at("stamp_ns")<<",1,"<<frozen->at("native_score")<<',';
    number(out.stream,c.score);out.stream<<','<<c.finite<<','<<c.converged<<','<<c.iterations<<',';
    number(out.stream,c.overlap);out.stream<<','<<r17::quality(c,source.mask)<<',';
    number(out.stream,rigid(c.pose)?translation(c.pose,seed):NAN);out.stream<<',';
    number(out.stream,rigid(c.pose)?rotation(c.pose,seed):NAN);
    out.stream<<','<<c.align_ms<<','<<c.validation_ms<<','<<peakKiB();out.end();
  }
  require(aligns==6,"stability align count mismatch");
  std::cout<<"STABILITY_PROBE_COMPLETE aligns="<<aligns<<" peak_rss_KiB="<<peakKiB()<<std::endl;
}catch(const std::exception& e){std::cerr<<"STABILITY_PROBE_STOP "<<e.what()<<std::endl;return 1;}}
