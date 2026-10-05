// Offline fixed-W/S inner solvers. Included inside the closure runner namespace.
// All acceptance tests use the exact dynamic PCL score, never a quadratic model.
struct StrongTrace {
  int iteration = 0;
  int evaluations = 0;
  Eigen::VectorXd v;
  double energy = 0.0;
  double support_from_T0 = 0.0;
  double support_from_previous = 0.0;
  double projected_gradient_before_step = std::numeric_limits<double>::quiet_NaN();
  double scale = 0.0;
};

struct StrongSolution {
  Eigen::VectorXd v;
  double initial_energy = 0.0;
  double energy = 0.0;
  Support support;
  int evaluations = 0;
  int iterations = 0;
  int accepted_steps = 0;
  double runtime_ms = 0.0;
  std::string status = "NOT_RUN";
  std::vector<StrongTrace> trace;
};

double supportFraction(const Support& a, const Support& b) {
  return static_cast<double>(compareSupport(a,b).changed_points)/a.size();
}

void recordStrongTrace(StrongSolution& result, const FrameContext& context,
    const Support& previous, int iteration, double scale,
    double gradient = std::numeric_limits<double>::quiet_NaN()) {
  StrongTrace item;
  item.iteration=iteration; item.evaluations=result.evaluations;
  item.v=result.v; item.energy=result.energy;
  item.support_from_T0=supportFraction(context.nominal_support,result.support);
  item.support_from_previous=supportFraction(previous,result.support);
  item.projected_gradient_before_step=gradient; item.scale=scale;
  result.trace.push_back(item);
}

// Unlike the old one-step solver, this pullback is local to the current Euler
// branch and has no .45-radian base-Euler domain cap. The global pose chart
// remains Exp(Wu+Sv)R0. Its strong-coordinate second derivatives are retained.
bool strongNativePullback(const FrameContext& context, const Vector6d& eta,
    Vector6d& p, Eigen::MatrixXd& jacobian,
    std::array<Eigen::MatrixXd,6>& second) {
  const Eigen::Matrix4f& base=context.obs.pose;
  const Eigen::Vector3d base_euler=base.block<3,3>(0,0).eulerAngles(0,1,2).cast<double>();
  if (!parametersAtEta(base,base_euler,eta,&p,4.0)) return false;
  const Eigen::Vector3d current_euler=p.tail<3>();
  auto native=[&](const Vector6d& trial, Vector6d& out) {
    return parametersAtEta(base,current_euler,trial,&out,4.0);
  };
  const int d=context.strong.cols();
  jacobian.resize(6,d);
  for (auto& matrix:second) matrix=Eigen::MatrixXd::Zero(d,d);
  constexpr double jh=1e-5, hh=1e-3;
  for (int i=0;i<d;++i) {
    Vector6d plus,minus;
    if (!native(eta+jh*context.strong.col(i),plus) ||
        !native(eta-jh*context.strong.col(i),minus)) return false;
    jacobian.col(i)=(plus-minus)/(2*jh);
    if (!native(eta+hh*context.strong.col(i),plus) ||
        !native(eta-hh*context.strong.col(i),minus)) return false;
    const Vector6d diag=(plus-2*p+minus)/(hh*hh);
    for (int a=0;a<6;++a) second[a](i,i)=diag(a);
    for (int j=i+1;j<d;++j) {
      const Vector6d di=hh*context.strong.col(i),dj=hh*context.strong.col(j);
      Vector6d pp,pm,mp,mm;
      if (!native(eta+di+dj,pp) || !native(eta+di-dj,pm) ||
          !native(eta-di+dj,mp) || !native(eta-di-dj,mm)) return false;
      const Vector6d mix=(pp-pm-mp+mm)/(4*hh*hh);
      for (int a=0;a<6;++a) second[a](i,j)=second[a](j,i)=mix(a);
    }
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
  const auto singular=svd.singularValues();
  return singular.allFinite() && singular.minCoeff()>1e-9 &&
         singular.maxCoeff()/singular.minCoeff()<1e4;
}

StrongSolution iterativeNewton(ExactPclNdt& ndt, const FrameContext& context,
    const Eigen::VectorXd& u, const Eigen::VectorXd& initial) {
  const auto started=std::chrono::steady_clock::now();
  StrongSolution result;
  result.v=initial;
  auto pose=[&](const Eigen::VectorXd& v) {
    return poseAtEta(context.obs.pose,context.weak*u+context.strong*v);
  };
  result.energy=-ndt.dynamicValueOnly(context.source,pose(initial),&result.support)/context.source->size();
  result.initial_energy=result.energy; ++result.evaluations;
  double trust=.10;
  recordStrongTrace(result,context,result.support,0,trust);
  result.status="MAX_ITERATIONS";
  for (int iteration=0;iteration<20;++iteration) {
    result.iterations=iteration+1;
    const Vector6d eta=context.weak*u+context.strong*result.v;
    Vector6d p;
    Eigen::MatrixXd jacobian;
    std::array<Eigen::MatrixXd,6> second;
    if (!strongNativePullback(context,eta,p,jacobian,second)) {
      result.status="CURRENT_CHART_INVALID"; break;
    }
    const NativeEnergyJet jet=evaluateNativeEnergy(ndt,context.source,pose(result.v),p,true,true);
    ++result.evaluations;
    if (std::abs(jet.energy-result.energy)>1e-10)
      throw std::runtime_error("Newton/value-only objective disagreement");
    const Eigen::VectorXd gradient=jacobian.transpose()*jet.gradient;
    Eigen::MatrixXd hessian=jacobian.transpose()*jet.hessian*jacobian;
    for (int a=0;a<6;++a) hessian+=jet.gradient(a)*second[a];
    hessian=.5*(hessian+hessian.transpose()).eval();
    if (!gradient.allFinite() || !hessian.allFinite()) {result.status="DERIVATIVE_NONFINITE";break;}
    if (gradient.norm()<1e-5) {result.status="BRANCH_GRADIENT_SMALL";break;}
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(hessian);
    if (eig.info()!=Eigen::Success) {result.status="HESSIAN_INVALID";break;}
    const double floor=1e-4*std::max(1.0,hessian.diagonal().cwiseAbs().maxCoeff());
    const double damping=std::max(0.0,floor-eig.eigenvalues().minCoeff());
    const Eigen::MatrixXd model=hessian+damping*Eigen::MatrixXd::Identity(hessian.rows(),hessian.cols());
    Eigen::VectorXd step=-model.ldlt().solve(gradient);
    if (!step.allFinite()) {result.status="STEP_NONFINITE";break;}
    if (step.norm()>trust) step*=trust/step.norm();
    if (step.norm()<1e-6) {result.status="STEP_SMALL";break;}
    bool accepted=false;
    const double old_energy=result.energy;
    const Support old_support=result.support;
    for (int line=0;line<8;++line) {
      const double alpha=std::ldexp(1.0,-line);
      const Eigen::VectorXd move=alpha*step,trial=result.v+move;
      Support support;
      const double energy=-ndt.dynamicValueOnly(context.source,pose(trial),&support)/context.source->size();
      ++result.evaluations;
      if (std::isfinite(energy) && energy<old_energy-1e-10) {
        const double predicted=-gradient.dot(move)-.5*move.dot(hessian*move);
        const double ratio=predicted>0 ? (old_energy-energy)/predicted : 0.0;
        if (ratio<.25) trust=std::max(1e-5,.5*trust);
        else if (ratio>.75 && move.norm()>.9*trust) trust=std::min(.50,2*trust);
        result.v=trial; result.energy=energy; result.support=std::move(support);
        ++result.accepted_steps;
        recordStrongTrace(result,context,old_support,iteration+1,trust,gradient.norm());
        accepted=true; break;
      }
    }
    if (!accepted) {
      trust=std::max(1e-5,.5*trust);
      if (trust<=1e-5) {result.status="TRUST_RADIUS_EXHAUSTED";break;}
      continue;
    }
    if (old_energy-result.energy<1e-9*std::max(1.0,std::abs(old_energy))) {
      result.status="ENERGY_CHANGE_SMALL";break;
    }
  }
  result.runtime_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
  return result;
}

// Generic deterministic pattern search also permits isolated objective/budget
// tests. A value callback may capture dynamic support, without additional calls.
template<class ValueFunction>
StrongSolution patternSearch(const Eigen::VectorXd& initial, ValueFunction value,
                             const FrameContext& context) {
  const auto started=std::chrono::steady_clock::now();
  StrongSolution result;
  result.v=initial; result.energy=value(initial,result.support);
  result.initial_energy=result.energy; result.evaluations=1;
  double step=.10;
  recordStrongTrace(result,context,result.support,0,step);
  result.status="EVALUATION_BUDGET";
  while (result.evaluations<100) {
    ++result.iterations;
    const Eigen::VectorXd sweep_start=result.v;
    const double sweep_energy=result.energy;
    const Support sweep_support=result.support;
    int polls=0;
    for (int axis=0;axis<initial.size() && result.evaluations<100;++axis) {
      const Eigen::VectorXd center=result.v;
      for (double sign : {1.0,-1.0}) {
        if (result.evaluations>=100) break;
        Eigen::VectorXd trial=center;
        trial(axis)+=sign*step;
        Support support;
        const double energy=value(trial,support); ++result.evaluations;
        ++polls;
        if (std::isfinite(energy) && energy<result.energy-1e-10) {
          result.v=trial; result.energy=energy; result.support=std::move(support);
          ++result.accepted_steps;
        }
      }
    }
    if (polls<2*initial.size()) {
      if (result.energy<sweep_energy-1e-10)
        recordStrongTrace(result,context,sweep_support,result.iterations,step);
      break;  // An incomplete poll cannot certify POLL_STEP_SMALL.
    }
    if (result.energy<sweep_energy-1e-10) {
      if (result.evaluations<100) {
        const Eigen::VectorXd trial=result.v+(result.v-sweep_start);
        Support support;
        const double energy=value(trial,support); ++result.evaluations;
        if (std::isfinite(energy) && energy<result.energy-1e-10) {
          result.v=trial;result.energy=energy;result.support=std::move(support);++result.accepted_steps;
        }
      }
      recordStrongTrace(result,context,sweep_support,result.iterations,step);
      step=std::min(.25,1.2*step);
    } else {
      step*=.5;
      if (step<.001) {result.status="POLL_STEP_SMALL";break;}
    }
  }
  result.runtime_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
  return result;
}

StrongSolution derivativeFree(ExactPclNdt& ndt, const FrameContext& context,
    const Eigen::VectorXd& u, const Eigen::VectorXd& initial) {
  return patternSearch(initial,[&](const Eigen::VectorXd& v,Support& support) {
    return -ndt.dynamicValueOnly(context.source,
      poseAtEta(context.obs.pose,context.weak*u+context.strong*v),&support)/context.source->size();
  },context);
}
