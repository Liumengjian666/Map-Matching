// OFFLINE ONLY. Reuse exact production extraction without changing its PImpl API.
// This TU is never linked into the P7 runner. Do not link current_frame_ndt twice.
#include "../../src/current_frame_ndt.cpp"
#include "dog_prior_map_fastlio2_frontend_exp/p7_replay_io.hpp"
#include <dcreg.hpp>
#include <pcl/common/transforms.h>
#include <Eigen/QR>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <sys/resource.h>

namespace p7 = dog_prior_map_fastlio2_frontend_exp;
namespace rel = p7::reliability;
using M6 = Eigen::Matrix<double,6,6>;
using V6 = Eigen::Matrix<double,6,1>;
using V3 = Eigen::Vector3d;
using M3 = Eigen::Matrix3d;
using Obs = std::vector<rel::GeometricObservation>;
using Clock = std::chrono::steady_clock;
double ms(Clock::time_point start) {
  return std::chrono::duration<double,std::milli>(Clock::now()-start).count();
}
void check(bool value, const std::string& why) {
  if (!value) throw std::runtime_error(why);
}
template<class T> void matrix(std::ostream& out, const T& m) {
  for(int i=0;i<m.rows();++i) for(int j=0;j<m.cols();++j) out << ',' << m(i,j);
}
void columns(std::ostream& out,const std::string& prefix,int n) {
  for(int i=0;i<n;++i) out << ',' << prefix << i;
}
void header(std::ostream& out) {
  out << "case,variant,parameter,source_hash,observations,uobs_valid,uobs_dim,uobs_ms,dc_valid,dc_dim,dc_ms";
  columns(out,"H",36); columns(out,"W",36); columns(out,"DC",36);
  columns(out,"reference",36); columns(out,"transport",36);
  out << ",reference_dim,length_scale\n";
}
void record(std::ostream& out,const std::string& name,const std::string& variant,
            double parameter,const Obs& obs,const M6& reference,int reference_dim,
            const M6& transport=M6::Identity(),double scale=1.,uint64_t hash=0,
            double extraction_ms=0.) {
  auto start=Clock::now();
  auto local=rel::analyzeGeometricObservability(obs,true,.8*scale,1e-6*scale*scale);
  auto sub=rel::classifyFixedPhysicalJointSubspace(local);
  double uobs_ms=ms(start)+extraction_ms;
  start=Clock::now();
  dcreg::SolverParameters params;
  dcreg::DegeneracyCharacterization dc;
  if(local.physical_geometric_information.allFinite()) {
    auto det=dcreg::DetectDegeneracy(local.physical_geometric_information,params);
    dc=dcreg::CharacterizeDegeneracy(det,params);
  }
  M6 dw=M6::Zero(); int dd=0;
  if(dc.factorization_ok) for(int i=0;i<3;++i) {
    if(dc.degenerate_mask[i]) dw.block<3,1>(0,dd++)=dc.aligned_rot_basis.col(i);
    if(dc.degenerate_mask[i+3]) dw.block<3,1>(3,dd++)=dc.aligned_trans_basis.col(i)/(.8*scale);
  }
  double dc_ms=ms(start);
  out << name << ',' << variant << ',' << parameter << ',' << hash << ',' << obs.size()
      << ',' << sub.valid << ',' << sub.weak_dimension << ',' << uobs_ms
      << ',' << dc.factorization_ok << ',' << dd << ',' << dc_ms;
  matrix(out,local.physical_geometric_information); matrix(out,sub.weak_basis);
  matrix(out,dw); matrix(out,reference); matrix(out,transport);
  out << ',' << reference_dim << ',' << .8*scale << '\n';
}
M3 skew(const V3& x) {
  M3 a; a << 0,-x.z(),x.y(), x.z(),0,-x.x(), -x.y(),x.x(),0; return a;
}
void point(Obs& obs,const V3& p,const V3& n,double weight=1) {
  rel::GeometricObservation o; o.rotated_source_map=p;
  o.voxel_covariance_map=.05*M3::Identity()+(.0005-.05)*n*n.transpose();
  o.nonnegative_weight=weight; obs.push_back(o);
}
void plane(Obs& obs,int normal,double position,double weight=1) {
  for(int i=-10;i<=10;++i) for(int j=-10;j<=10;++j) {
    V3 p=V3::Zero(),n=V3::Zero(); n(normal)=1; p(normal)=position;
    p((normal+1)%3)=i*.5; p((normal+2)%3)=j*.5; point(obs,p,n,weight);
  }
}
void controlled(std::ostream& out) {
  for(const auto& name : {"plane","parallel","corner","tunnel","corridor"}) {
    Obs obs; M6 ref=M6::Zero(); int dim=0;
    if(std::string(name)=="plane") {
      plane(obs,2,2); for(int a:{2,3,4}) ref(a,dim++)=1;
    } else if(std::string(name)=="parallel") {
      plane(obs,1,2); plane(obs,1,-2); for(int a:{1,3,5}) ref(a,dim++)=1;
    } else if(std::string(name)=="corner") {
      plane(obs,0,2); plane(obs,1,2); plane(obs,2,2);
    } else if(std::string(name)=="tunnel") {
      for(int i=-10;i<=10;++i) for(int j=0;j<64;++j) {
        double a=2*std::acos(-1.)*j/64.; V3 n(0,std::cos(a),std::sin(a));
        point(obs,V3(i*.5,2*n.y(),2*n.z()),n);
      }
      for(int a:{0,3}) ref(a,dim++)=1;
    } else {
      plane(obs,1,2); plane(obs,1,-2); plane(obs,2,2); plane(obs,2,-2); ref(3,dim++)=1;
    }
    record(out,name,"base",0,obs,ref,dim);
    Obs cm=obs;
    for(auto& o:cm) { o.rotated_source_map*=100; o.voxel_covariance_map*=10000; }
    record(out,name,"cm",0,cm,ref,dim,M6::Identity(),100);
    M3 q=Eigen::AngleAxisd(.63,V3(1,2,3).normalized()).toRotationMatrix();
    Obs rotated=obs; M6 qt=M6::Zero(); qt.topLeftCorner<3,3>()=q; qt.bottomRightCorner<3,3>()=q;
    for(auto& o:rotated) { o.rotated_source_map=q*o.rotated_source_map;
      o.voxel_covariance_map=q*o.voxel_covariance_map*q.transpose(); }
    record(out,name,"rotated",0,rotated,qt*ref,dim,qt.transpose());
    for(double amount:{.1,1.}) {
      V3 c=amount*V3(1,2,3); Obs shifted=obs;
      for(auto& o:shifted) o.rotated_source_map-=c;
      M6 t=M6::Identity(); t.bottomLeftCorner<3,3>()=skew(c)/.8;
      M6 ti=M6::Identity(); ti.bottomLeftCorner<3,3>()=-skew(c)/.8;
      record(out,name,"reference_point",amount,shifted,ti*ref,dim,t);
    }
    if(std::string(name)=="corridor") for(double weight:{1.,.1,.01,.001,0.}) {
      Obs transition=obs; plane(transition,0,5,weight);
      record(out,"transition","endwall",weight,transition,weight==0 ? ref : M6::Zero(),weight==0?1:0);
    }
  }
}

class OfflineNdt : public p7::ObservableNdt {
 public:
  Eigen::Vector3f leafSize() const { return target_cells_.getLeafSize(); }
  double cost(const p7::Cloud& source,const p7::Pose3d& pose) {
    p7::Cloud transformed;
    const Eigen::Isometry3d transform=p7::asIsometry(pose);
    pcl::transformPointCloud(source,transformed,transform.matrix().cast<float>());
    V6 p; p.head<3>()=pose.position;
    p.tail<3>()=transform.rotation().eulerAngles(0,1,2);
    V6 gradient; M6 hessian;
    // Existing P6 scoreHessian diagnostic pattern. Only scalar score is used;
    // neither Euler Hessian nor this result enters a production detector.
    return -computeDerivatives(gradient,hessian,transformed,p,false);
  }
};
std::vector<std::string> split(const std::string& line) {
  std::stringstream stream(line); std::vector<std::string> result; std::string field;
  while(std::getline(stream,field,',')) result.push_back(field);
  return result;
}
M6 curvature(OfflineNdt& ndt,const p7::Cloud& cloud,const p7::Pose3d& pose,double h) {
  auto f=[&](const V6& dx) {
    p7::Pose3d moved=pose; double angle=dx.head<3>().norm();
    if(angle>0) moved.orientation=Eigen::Quaterniond(Eigen::AngleAxisd(angle,dx.head<3>()/angle))*pose.orientation;
    moved.position+=.8*dx.tail<3>(); return ndt.cost(cloud,moved);
  };
  M6 result; double center=f(V6::Zero());
  for(int i=0;i<6;++i) {
    V6 a=V6::Zero(); a(i)=h;
    result(i,i)=(f(a)-2*center+f(-a))/(h*h);
    for(int j=0;j<i;++j) {
      V6 b=V6::Zero(); b(j)=h;
      result(i,j)=result(j,i)=(f(a+b)-f(a-b)-f(-a+b)+f(-a-b))/(4*h*h);
    }
  }
  return result;
}
void real(std::ostream& out,std::ostream& curves,char** argv) {
  const auto scans=p7::readP7Scans(argv[3],argv[4]);
  p7::Cloud::Ptr raw(new p7::Cloud),finite(new p7::Cloud);
  check(pcl::io::loadPCDFile(argv[2],*raw)>=0,"map read failed");
  for(const auto& p:*raw) if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)) finite->push_back(p);
  p7::finalize(finite);
  auto target=p7::voxelDown(p7::voxelDown(finite,.15),.15);
  OfflineNdt ndt; ndt.setResolution(.8f); ndt.setInputTarget(target);
  ndt.setStepSize(.08); ndt.setTransformationEpsilon(1e-5); ndt.setMaximumIterations(80);
  check((ndt.leafSize().array()==.8f).all(),"offline grid mismatch");
  std::ifstream reg(argv[6]); check(bool(reg),"registration read failed");
  std::string line; std::getline(reg,line); const auto fields=split(line);
  auto col=[&](const std::string& key) { auto it=std::find(fields.begin(),fields.end(),key);
    check(it!=fields.end(),"missing registration field "+key); return std::distance(fields.begin(),it); };
  bool initialized=false; int rows=0;
  curves << "transaction_id,ndt_effective,source_hash_match,geometry_ms,curvature_ms";
  columns(curves,"coarse",36); columns(curves,"fine",36); curves << '\n';
  while(std::getline(reg,line)) {
    const auto row=split(line); check(row.size()==fields.size(),"bad registration row");
    auto get=[&](const std::string& key)->const std::string& { return row.at(col(key)); };
    auto tx=std::stoull(get("transaction_id")); check(tx==static_cast<uint64_t>(++rows),"nonsequential tx");
    const auto& scan=scans.at(tx-1); check(scan.stamp_ns==std::stoull(get("stamp_ns")),"stamp mismatch");
    if(get("effective")!="1") continue;
    auto prepared=p7::preprocessRegistrationCloud(p7::readP7PackedCloud(argv[5],scan),p7::CurrentFrameNdtParameters{});
    uint64_t hash=p7::registrationCloudHash(prepared);
    check(hash==std::stoull(get("source_hash_actual")),"source hash mismatch");
    p7::Cloud::Ptr cloud(new p7::Cloud);
    for(const auto& p:prepared) cloud->push_back(p7::Point(p.x,p.y,p.z));
    p7::finalize(cloud);
    ndt.setInputSource(cloud);
    p7::Pose3d pose; pose.position=V3(std::stod(get("raw_x")),std::stod(get("raw_y")),std::stod(get("raw_z")));
    pose.orientation=Eigen::Quaterniond(std::stod(get("raw_qw")),std::stod(get("raw_qx")),std::stod(get("raw_qy")),std::stod(get("raw_qz")));
    if(!initialized) { p7::Cloud ignored; ndt.align(ignored,p7::asIsometry(pose).matrix().cast<float>()); initialized=true; }
    auto start=Clock::now(); auto observations=ndt.geometricObservations(*cloud,pose,.8); double geo_ms=ms(start);
    record(out,std::to_string(tx),"real",0,observations,M6::Zero(),-1,M6::Identity(),1,hash,geo_ms);
    start=Clock::now(); M6 coarse=curvature(ndt,*cloud,pose,.01),fine=curvature(ndt,*cloud,pose,.005);
    curves << tx << ",1,1," << geo_ms << ',' << ms(start); matrix(curves,coarse); matrix(curves,fine); curves << '\n';
  }
  check(rows==100,"expected fixed 100-frame preregistered prefix");
}
int main(int argc,char** argv) {
  try {
    check(argc==2||argc==7,"usage: audit OUTPUT_PREFIX [MAP FILTER_SCANS SCANS PACKED REGISTRATION]");
    std::ofstream out(std::string(argv[1])+"_geometry.csv"),curves(std::string(argv[1])+"_curvature.csv");
    check(bool(out)&&bool(curves),"output open failed"); out<<std::setprecision(17); curves<<std::setprecision(17);
    header(out); controlled(out); if(argc==7) real(out,curves,argv);
    check(bool(out)&&bool(curves),"output write failed");
    rusage usage{}; getrusage(RUSAGE_SELF,&usage);
    std::cout << "audit_peak_RSS_MiB=" << usage.ru_maxrss/1024. << '\n';
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
