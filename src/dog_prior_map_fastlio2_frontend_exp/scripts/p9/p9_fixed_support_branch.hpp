#pragma once

// R1C derivatives are value-only central FD, never PCL dynamic scoreJet jets.
struct FrozenFd {
  double energy=0, asymmetry=0;
  Eigen::VectorXd gradient, eigenvalues;
  Eigen::MatrixXd hessian;
  bool valid=false, spd=false;
  double condition=std::numeric_limits<double>::infinity();
};

template<class Value>
FrozenFd frozenFiniteDifference(const Eigen::VectorXd& x, Value value) {
  constexpr double h=1e-3;
  FrozenFd r;
  const int n=x.size();
  r.energy=value(x);r.gradient.resize(n);r.hessian.resize(n,n);
  for (int i=0;i<n;++i) {
    Eigen::VectorXd d=Eigen::VectorXd::Zero(n);d(i)=h;
    const double plus=value(x+d),minus=value(x-d);
    r.gradient(i)=(plus-minus)/(2*h);
    r.hessian(i,i)=(plus-2*r.energy+minus)/(h*h);
    for (int j=0;j<i;++j) {
      Eigen::VectorXd e=Eigen::VectorXd::Zero(n);e(j)=h;
      const double mixed=(value(x+d+e)-value(x+d-e)-value(x-d+e)+value(x-d-e))/(4*h*h);
      r.hessian(i,j)=mixed;r.hessian(j,i)=mixed;
    }
  }
  r.asymmetry=(r.hessian-r.hessian.transpose()).norm()/std::max(1e-12,r.hessian.norm());
  r.valid=std::isfinite(r.energy)&&r.gradient.allFinite()&&r.hessian.allFinite()&&r.asymmetry<=1e-3;
  if (r.valid) {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(r.hessian);
    r.valid=eig.info()==Eigen::Success;
    if (r.valid) {
      r.eigenvalues=eig.eigenvalues();r.spd=r.eigenvalues.minCoeff()>0;
      if(r.spd) r.condition=r.eigenvalues.maxCoeff()/r.eigenvalues.minCoeff();
    }
  }
  return r;
}

struct FrozenMinimum {
  Eigen::VectorXd v;
  FrozenFd fd;
  int iterations=0, accepted=0;
  std::string status="MAX_INNER_ITERATIONS";
};

template<class Value>
FrozenMinimum minimizeFrozenStrong(const Eigen::VectorXd& initial, Value value) {
  FrozenMinimum r;r.v=initial;
  double trust=.10;
  for(int iteration=0;iteration<20;++iteration) {
    r.iterations=iteration+1;
    r.fd=frozenFiniteDifference(r.v,value);
    if(!r.fd.valid) {r.status="DERIVATIVE_INVALID";return r;}
    if(r.fd.gradient.norm()<=1e-5) {
      r.status=r.fd.spd?"STATIONARY_SPD":"STATIONARY_NOT_SPD";return r;
    }
    const double floor=1e-4*std::max(1.0,r.fd.hessian.diagonal().cwiseAbs().maxCoeff());
    const double damping=std::max(0.0,floor-r.fd.eigenvalues.minCoeff());
    Eigen::VectorXd step=-(r.fd.hessian+damping*Eigen::MatrixXd::Identity(initial.size(),initial.size())).ldlt().solve(r.fd.gradient);
    if(!step.allFinite()){r.status="STEP_INVALID";return r;}
    if(step.norm()>trust)step*=trust/step.norm();
    bool accepted=false;
    for(int line=0;line<10;++line) {
      const Eigen::VectorXd move=std::ldexp(1.0,-line)*step;
      const double candidate=value(r.v+move);
      if(std::isfinite(candidate)&&candidate<r.fd.energy-1e-12) {
        const double predicted=-r.fd.gradient.dot(move)-.5*move.dot(r.fd.hessian*move);
        const double ratio=predicted>0?(r.fd.energy-candidate)/predicted:0;
        r.v+=move;++r.accepted;accepted=true;
        if(ratio>.75&&move.norm()>.9*trust)trust=std::min(.50,2*trust);
        else if(ratio<.25)trust*=.5;
        break;
      }
    }
    if(!accepted)trust*=.5;
    if(trust<1e-6){r.status="FROZEN_TRUST_EXHAUSTED";break;}
  }
  // The certificate always uses derivatives at the returned endpoint.
  r.fd=frozenFiniteDifference(r.v,value);
  if(r.fd.valid&&r.fd.gradient.norm()<=1e-5&&r.fd.spd)r.status="STATIONARY_SPD";
  return r;
}

bool runFrozenBranchSelfTest() {
  Eigen::MatrixXd h=Eigen::MatrixXd::Identity(6,6);
  h(0,2)=h(2,0)=.2;h(1,4)=h(4,1)=-.3;
  Eigen::VectorXd center=Eigen::VectorXd::LinSpaced(6,-.15,.10);
  auto f=[&](const Eigen::VectorXd& x){return .5*(x-center).dot(h*(x-center));};
  const auto fd=frozenFiniteDifference(Eigen::VectorXd::Zero(6),f);
  if(!fd.valid||!fd.spd||(fd.gradient+h*center).norm()>1e-9||(fd.hessian-h).norm()>1e-7)return false;
  auto strong=[&](const Eigen::VectorXd& v){Eigen::VectorXd x=Eigen::VectorXd::Zero(6);x.tail(4)=v;return f(x);};
  const auto minimum=minimizeFrozenStrong(Eigen::VectorXd::Zero(4),strong);
  if(minimum.status!="STATIONARY_SPD"||minimum.fd.gradient.norm()>1e-5)return false;
  const auto jet=frozenFiniteDifference(Eigen::VectorXd::Zero(6),f);
  const Eigen::VectorXd ub=(Eigen::Vector2d()<<.3,-.2).finished();
  const Eigen::VectorXd predictor=-jet.hessian.bottomRightCorner(4,4).ldlt().solve(jet.hessian.bottomLeftCorner(4,2)*ub);
  return (predictor+h.bottomLeftCorner(4,2)*ub).norm()<1e-7;
}
