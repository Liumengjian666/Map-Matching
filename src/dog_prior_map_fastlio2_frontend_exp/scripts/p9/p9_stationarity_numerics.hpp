#pragma once

namespace {

// R1C2 only: h is explicit; production R1C finite differences are untouched.
struct NumericJet {
  double energy=0, decrement=0;
  Eigen::VectorXd g,d,eig;
  Eigen::MatrixXd H;
  bool spd=false;
};

template<class Value>
NumericJet numericalJet(const Eigen::VectorXd& x,double h,Value f) {
  NumericJet j;const int n=x.size();j.energy=f(x);
  j.g.resize(n);j.H.resize(n,n);
  for(int a=0;a<n;++a) {
    Eigen::VectorXd da=Eigen::VectorXd::Zero(n);da(a)=h;
    const double fp=f(x+da),fm=f(x-da);
    j.g(a)=(fp-fm)/(2*h);j.H(a,a)=(fp-2*j.energy+fm)/(h*h);
    for(int b=0;b<a;++b) {
      Eigen::VectorXd db=Eigen::VectorXd::Zero(n);db(b)=h;
      j.H(a,b)=j.H(b,a)=(f(x+da+db)-f(x+da-db)-f(x-da+db)+f(x-da-db))/(4*h*h);
    }
  }
  return j;
}

void completeJet(NumericJet& j) {
  if(!std::isfinite(j.energy)||!j.g.allFinite()||!j.H.allFinite())
    throw std::runtime_error("nonfinite numerical jet");
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(j.H);
  if(es.info()!=Eigen::Success)throw std::runtime_error("numeric Hessian eigensolver");
  j.eig=es.eigenvalues();j.spd=j.eig.minCoeff()>0;
  j.d=Eigen::VectorXd::Constant(j.g.size(),std::numeric_limits<double>::quiet_NaN());
  j.decrement=std::numeric_limits<double>::quiet_NaN();
  if(j.spd) {j.d=-j.H.ldlt().solve(j.g);j.decrement=-.5*j.g.dot(j.d);}
}

template<class Value>
NumericJet richardsonJet(const Eigen::VectorXd& x,double h,Value f) {
  const auto coarse=numericalJet(x,h,f),fine=numericalJet(x,h/2,f);
  NumericJet r;r.energy=fine.energy;r.g=(4*fine.g-coarse.g)/3;r.H=(4*fine.H-coarse.H)/3;
  completeJet(r);return r;
}

double doubleEnergyResolution(double energy,std::size_t terms) {
  return 64*std::numeric_limits<double>::epsilon()*std::max(1.0,std::abs(energy))*(terms+32);
}

struct ReferenceMinimum {
  Eigen::VectorXd v;
  NumericJet jet;
  int iterations=0,accepted=0;
  double rho=0,d_resolution=0,fd_displacement_difference=0,fd_eigen_relative=0;
  double coarse_d=0,coarse_decrement=0,coarse_min_eig=0;
  std::string status="REFERENCE_MAX_ITERATIONS";
};

template<class Value>
ReferenceMinimum referenceMinimum(const Eigen::VectorXd& initial,std::size_t terms,Value f) {
  ReferenceMinimum r;r.v=initial;double trust=.10;
  for(int i=0;i<100;++i) {
    r.iterations=i+1;
    const auto coarse=richardsonJet(r.v,.000125,f);
    r.jet=richardsonJet(r.v,.0000625,f);
    r.rho=doubleEnergyResolution(r.jet.energy,terms);
    if(!coarse.spd||!r.jet.spd){r.status="REFERENCE_NOT_SPD";return r;}
    r.coarse_d=coarse.d.norm();r.coarse_decrement=coarse.decrement;r.coarse_min_eig=coarse.eig.minCoeff();
    r.fd_displacement_difference=(r.jet.d-coarse.d).norm();
    r.fd_eigen_relative=((r.jet.eig-coarse.eig).cwiseAbs().array()/r.jet.eig.cwiseAbs().array()).maxCoeff();
    r.d_resolution=std::sqrt(2*r.rho/r.jet.eig.minCoeff());
    const bool resolution_candidate=r.fd_eigen_relative<=.10&&r.jet.d.norm()<=r.d_resolution&&r.jet.decrement<=r.rho&&
       coarse.d.norm()<=std::sqrt(2*r.rho/coarse.eig.minCoeff())&&coarse.decrement<=r.rho&&
       r.fd_displacement_difference<=r.d_resolution;
    bool any_resolved_descent=false;
    if(resolution_candidate)for(int line=0;line<16;++line)
      if(f(r.v+std::ldexp(1.0,-line)*r.jet.d)<r.jet.energy)any_resolved_descent=true;
    if(resolution_candidate&&!any_resolved_descent) {
      r.status="DOUBLE_RESOLUTION_STATIONARY";return r;
    }
    Eigen::VectorXd step=r.jet.d;
    if(step.norm()>trust)step*=trust/step.norm();
    bool accepted=false;
    for(int line=0;line<16;++line) {
      const Eigen::VectorXd move=std::ldexp(1.0,-line)*step;
      const double candidate=f(r.v+move);
      if(std::isfinite(candidate)&&candidate<r.jet.energy) {
        r.v+=move;++r.accepted;accepted=true;
        if(line==0&&move.norm()>.9*trust)trust=std::min(.50,2*trust);
        break;
      }
    }
    if(!accepted)trust*=.5;
    if(trust<1e-12){r.status="REFERENCE_RESOLUTION_UNCLOSED";return r;}
  }
  const auto coarse=richardsonJet(r.v,.000125,f);
  r.jet=richardsonJet(r.v,.0000625,f);
  r.rho=doubleEnergyResolution(r.jet.energy,terms);
  if(coarse.spd&&r.jet.spd) {
    r.coarse_d=coarse.d.norm();r.coarse_decrement=coarse.decrement;r.coarse_min_eig=coarse.eig.minCoeff();
    r.d_resolution=std::sqrt(2*r.rho/r.jet.eig.minCoeff());
    r.fd_displacement_difference=(r.jet.d-coarse.d).norm();
    r.fd_eigen_relative=((r.jet.eig-coarse.eig).cwiseAbs().array()/r.jet.eig.cwiseAbs().array()).maxCoeff();
  }
  return r;
}

Eigen::Matrix4d continuousPose(const Eigen::Matrix4f& base,const Vector6d& eta) {
  Eigen::Matrix4d p=base.cast<double>();
  p.block<3,1>(0,3)+=kResolution*eta.head<3>();
  p.block<3,3>(0,0)=expRotation(eta.tail<3>())*base.block<3,3>(0,0).cast<double>();
  return p;
}

class DoubleFrozenNdt : public ExactPclNdt {
 public:
  std::size_t guardDisagreement(const Cloud::ConstPtr& source,const Eigen::Matrix4f& pf,
      const Eigen::Matrix4d& pd,const FrozenSupport& support)const {
    Cloud transformed;pcl::transformPointCloud(*source,transformed,pf);
    std::size_t differences=0;
    for(std::size_t i=0;i<source->size();++i) {
      const auto& p=source->points[i];const auto& q=transformed.points[i];
      const Eigen::Vector3d xf(q.x,q.y,q.z),xd=pd.block<3,3>(0,0)*Eigen::Vector3d(p.x,p.y,p.z)+pd.block<3,1>(0,3);
      auto passes=[&](const Eigen::Vector3d& x,const Leaf& leaf) {
        const Eigen::Vector3d r=x-leaf->getMean();
        const double g=gauss_d2_*std::exp(-gauss_d2_*r.dot(leaf->getInverseCov()*r)/2.0);
        return std::isfinite(g)&&g<=1&&g>=0;
      };
      for(const auto& leaf:support[i])if(passes(xf,leaf)!=passes(xd,leaf))++differences;
    }
    return differences;
  }
  double frozenScoreDouble(const Cloud::ConstPtr& source,const Eigen::Matrix4d& pose,
                           const FrozenSupport& support)const {
    if(!source||support.size()!=source->size()||!pose.allFinite())
      throw std::runtime_error("invalid DOUBLE source/support/pose");
    double score=0;
    for(std::size_t i=0;i<source->size();++i) {
      const auto& point=source->points[i];
      const Eigen::Vector3d x=pose.block<3,3>(0,0)*Eigen::Vector3d(point.x,point.y,point.z)+pose.block<3,1>(0,3);
      for(const auto& leaf:support[i]) {
        const Eigen::Vector3d residual=x-leaf->getMean();
        const double exponential=std::exp(-gauss_d2_*residual.dot(leaf->getInverseCov()*residual)/2.0);
        const double guard=gauss_d2_*exponential;
        if(!std::isfinite(guard)||guard>1.0||guard<0.0)continue;
        score+=-gauss_d1_*exponential;
      }
    }
    return score;
  }
};

bool stationaritySelfTest() {
  Eigen::MatrixXd H=Eigen::MatrixXd::Identity(4,4);H(0,0)=5;H(2,2)=70;H(1,3)=H(3,1)=.1;
  const Eigen::VectorXd center=Eigen::VectorXd::LinSpaced(4,-.002,.003);
  auto f=[&](const Eigen::VectorXd& x){return .5*(x-center).dot(H*(x-center))-3;};
  for(const double h:{.004,.002,.001,.0005,.00025,.000125}) {
    auto j=numericalJet(Eigen::VectorXd::Zero(4),h,f);completeJet(j);
    if(!j.spd||(j.d-center).norm()>1e-8||(j.H-H).norm()>1e-5)return false;
  }
  const auto minimum=referenceMinimum(Eigen::VectorXd::Zero(4),100,f);
  if(minimum.status!="DOUBLE_RESOLUTION_STATIONARY"||(minimum.v-center).norm()>1e-7)return false;
  Cloud::Ptr cloud(new Cloud);
  for(int a=-3;a<=3;++a)for(int b=-3;b<=3;++b)for(int c=-3;c<=3;++c)
    cloud->push_back(Point(.4f+.025f*a,.4f+.025f*b,.4f+.025f*c));
  finalize(cloud);DoubleFrozenNdt ndt;configureNdt(ndt,cloud);ndt.setInputSource(cloud);
  Support signature;ExactPclNdt::FrozenSupport support;
  ndt.dynamicValueOnly(cloud,Eigen::Matrix4f::Identity(),&signature,&support);
  if(signature.empty()||support.front().empty())return false;
  const double a=ndt.frozenScore(cloud,Eigen::Matrix4f::Identity(),support);
  const double b=ndt.frozenScoreDouble(cloud,Eigen::Matrix4d::Identity(),support);
  return a==b&&std::isfinite(a)&&a>0;
}
}
