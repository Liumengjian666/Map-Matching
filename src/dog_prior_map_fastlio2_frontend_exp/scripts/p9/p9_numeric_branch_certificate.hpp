#pragma once
#include "p9_stationarity_numerics.hpp"

namespace {
const std::vector<double> branchSteps={.004,.002,.001,.0005,.00025,.000125};
std::vector<int> stableBranchRegion(const std::vector<NumericJet>& js) {
  std::vector<int> best;
  for(int a=0;a<int(js.size());++a)for(int b=a+2;b<int(js.size());++b) {
    bool pass=true;
    for(int i=a;i<b;++i) {
      const auto& p=js[i];const auto& q=js[i+1];
      if(!p.spd||!q.spd||!p.d.allFinite()||!q.d.allFinite()||
         !std::isfinite(p.d.norm())||!std::isfinite(q.d.norm())||
         !std::isfinite(p.decrement)||!std::isfinite(q.decrement)){pass=false;break;}
      const double dd=(p.d-q.d).norm()/std::max(1e-300,std::max(p.d.norm(),q.d.norm()));
      const double de=((p.eig-q.eig).cwiseAbs().array()/p.eig.cwiseAbs().cwiseMax(q.eig.cwiseAbs()).array()).maxCoeff();
      if(!(std::isfinite(dd)&&std::isfinite(de)&&dd<=.20&&de<=.10)){pass=false;break;}
    }
    if(pass&&b-a+1>=int(best.size())){best.clear();for(int i=a;i<=b;++i)best.push_back(i);}
  }
  return best;
}

struct BranchCertificate {
  bool resolved=false,derivative_valid=false,spd=false,guards_equal=false;
  std::string status="NUMERICALLY_UNRESOLVED";
  double eps_g=0,eps_dv=0,eps_e=0,dn=0,decrement=0,reference_move=0,reference_drop=0;
  double float_g=0,gradient_fd_error=0,curvature_fd_error=0;
  std::vector<int> region;
  std::vector<NumericJet> floats,doubles;
  std::vector<std::array<double,7>> fd_audits;
  ReferenceMinimum reference;
  NumericJet joint;
};

bool completeBranchCertificate(const BranchCertificate& c,bool support_equal) {
  return support_equal&&c.resolved&&c.derivative_valid&&c.spd&&c.guards_equal;
}

template<class FloatValue,class DoubleValue,class JointFloat,class JointDouble,class Guard>
BranchCertificate numericBranchCertificate(const Eigen::VectorXd& u,const Eigen::VectorXd& v,
    std::size_t terms,FloatValue ff,DoubleValue fd,JointFloat jf,JointDouble jd,Guard guards) {
  BranchCertificate c;
  for(double h:branchSteps) {
    auto a=numericalJet(v,h,ff),b=numericalJet(v,h,fd);completeJet(a);completeJet(b);
    c.floats.push_back(a);c.doubles.push_back(b);
  }
  c.dn=c.doubles.back().d.norm();c.decrement=c.doubles.back().decrement;
  c.float_g=c.floats.back().g.norm();
  for(const auto* js:{&c.floats,&c.doubles})for(const auto& j:*js)
    if(j.spd&&(!j.d.allFinite()||!std::isfinite(j.d.norm())||!std::isfinite(j.decrement)))
      throw std::runtime_error("nonfinite branch Newton sensitivity");
  c.spd=c.floats[2].spd;
  for(const auto& j:c.doubles)c.spd=c.spd&&j.spd;
  c.region=stableBranchRegion(c.doubles);
  if(!c.spd){c.status="H_VV_NOT_SPD";return c;}
  if(c.region.empty())return c;
  Eigen::VectorXd previous_g,previous_d;
  double spread_g=0,spread_d=0;
  for(int i:c.region) {
    if(!c.floats[i].spd){c.status="FLOAT_ENVELOPE_NOT_SPD";return c;}
    const Eigen::VectorXd bg=c.floats[i].g-c.doubles[i].g,bd=c.floats[i].d-c.doubles[i].d;
    c.eps_g=std::max(c.eps_g,bg.norm());c.eps_dv=std::max(c.eps_dv,bd.norm());
    if(previous_g.size()){spread_g=std::max(spread_g,(bg-previous_g).norm());spread_d=std::max(spread_d,(bd-previous_d).norm());}
    previous_g=bg;previous_d=bd;
  }
  c.eps_g+=spread_g;c.eps_dv+=spread_d;
  c.guards_equal=true;
  for(int i=-1;i<8;++i) {
    Eigen::VectorXd x=v;if(i>=0)x(i/2)+=(i%2?1:-1)*.001;
    c.eps_e=std::max(c.eps_e,std::abs(ff(x)-fd(x)));
    const bool parity=guards(x);c.guards_equal=c.guards_equal&&parity;
  }
  c.reference=referenceMinimum(v,terms,fd);
  const auto& star=c.reference;
  c.eps_e=2*c.eps_e+star.rho;
  c.dn=c.doubles.back().d.norm();c.decrement=c.doubles.back().decrement;
  c.reference_move=(star.v-v).norm();c.reference_drop=fd(v)-fd(star.v);
  c.float_g=c.floats.back().g.norm();
  if(!std::isfinite(c.eps_g)||!std::isfinite(c.eps_dv)||!std::isfinite(c.eps_e)||
     !std::isfinite(c.dn)||!std::isfinite(c.decrement)||!std::isfinite(c.reference_move)||
     !std::isfinite(c.reference_drop))throw std::runtime_error("nonfinite branch numerical envelope");
  c.resolved=star.status=="DOUBLE_RESOLUTION_STATIONARY"&&c.dn<=c.eps_dv&&c.reference_move<=c.eps_dv&&
    c.decrement<=c.eps_e&&c.reference_drop<=c.eps_e&&c.float_g<=c.eps_g;
  Eigen::VectorXd x(6);x<<u,v;
  c.joint=numericalJet(x,.001,jf);completeJet(c.joint);
  c.derivative_valid=true;
  for(int precision=0;precision<2;++precision) {
    const double h=precision?.000125:.001;
    auto value=[&](const Eigen::VectorXd& z){return precision?jd(z):jf(z);};
    const auto jet=numericalJet(x,h,value);
    for(int a=0;a<3;++a) {
      Eigen::VectorXd q(6);for(int j=0;j<6;++j)q(j)=std::sin((a+1)*(j+2));q.normalize();
      const double plus=value(x+h*q),minus=value(x-h*q);
      const double g=(plus-minus)/(2*h),curv=(plus-2*jet.energy+minus)/(h*h);
      const double gp=jet.g.dot(q),hp=q.dot(jet.H*q);
      const double ge=std::abs(g-gp),ce=std::abs(curv-hp);
      const double gr=ge/std::max(1e-12,std::max(std::abs(g),std::abs(gp)));
      const double cr=ce/std::max(1e-12,std::max(std::abs(curv),std::abs(hp)));
      c.gradient_fd_error=std::max(c.gradient_fd_error,ge);c.curvature_fd_error=std::max(c.curvature_fd_error,ce);
      const bool pass=(ge<=1e-4||gr<=.02)&&(ce<=.02||cr<=.05);
      c.fd_audits.push_back({{double(precision),double(a),ge,gr,ce,cr,pass?1.:0.}});
      c.derivative_valid=c.derivative_valid&&pass;
    }
  }
  if(!c.guards_equal)c.status="GUARD_PARITY_FAIL";
  else if(!c.derivative_valid)c.status="DERIVATIVE_INVALID";
  else if(c.resolved)c.status="NUMERICALLY_RESOLVED_STATIONARITY";
  else c.status=star.status=="DOUBLE_RESOLUTION_STATIONARY"?"NONSTATIONARY_CANDIDATE":"REFERENCE_UNCLOSED";
  return c;
}

bool numericBranchSelfTest() {
  if(!stationaritySelfTest())return false;
  std::vector<NumericJet> jets(6);
  for(auto& j:jets){j.spd=true;j.d=Eigen::VectorXd::Ones(4);j.eig=Eigen::VectorXd::LinSpaced(4,1,4);}
  if(stableBranchRegion(jets).size()!=6)return false;
  for(int i=0;i<6;++i)jets[i].d*=std::pow(2.,i);
  if(!stableBranchRegion(jets).empty())return false;
  for(auto& j:jets)j.d=Eigen::VectorXd::Ones(4);
  jets[2].d(0)=std::numeric_limits<double>::quiet_NaN();
  const auto region=stableBranchRegion(jets);
  if(std::find(region.begin(),region.end(),2)!=region.end())return false;
  BranchCertificate c;c.resolved=c.spd=c.derivative_valid=c.guards_equal=true;
  if(!completeBranchCertificate(c,true)||completeBranchCertificate(c,false))return false;
  c.derivative_valid=false;
  return !completeBranchCertificate(c,true);
}
}
