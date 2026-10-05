// A separate offline executable reuses the exact R1 evaluator, product chart,
// and ONE_STEP_NEWTON without changing their original executable semantics.
#define P9_NDT_ENERGY_CONTRACT_LIBRARY
#include "p9_ndt_energy_contract.cpp"

namespace {

std::string vectorText(const Eigen::VectorXd& values) {
  std::ostringstream out;
  out << std::setprecision(17);
  for (Eigen::Index i = 0; i < values.size(); ++i) {
    if (i) out << ';';
    out << values(i);
  }
  return out.str();
}

std::string poseText(const Eigen::Matrix4f& pose) {
  std::ostringstream out;
  out << std::setprecision(17);
  for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) {
    if (r || c) out << ';';
    out << pose(r,c);
  }
  return out.str();
}

std::string ndtStatus(ExactPclNdt& ndt) {
  if (!ndt.getFinalTransformation().allFinite()) return "NONFINITE";
  if (ndt.getFinalNumIteration() >= 80) return "ITERATION_LIMIT";
  if (ndt.getFinalNumIteration() == 0) return "ZERO_ITERATION";
  return ndt.hasConverged() ? "SUCCESS" : "NOT_CONVERGED";
}

struct FrameContext {
  FrameSource frame;
  ArchivedUObs obs;
  Cloud::Ptr source;
  Eigen::MatrixXd weak, strong;
  Eigen::VectorXd bounds;
  Support nominal_support;
  int k = 0;
};

FrameContext frameContext(const FrameSource& frame, const ArchivedUObs& obs,
                          ExactPclNdt& ndt, std::size_t target_count) {
  FrameContext context;
  context.frame = frame;
  context.obs = obs;
  const auto leaf = ndt.actualGridLeaf();
  if (!obs.valid || obs.source_hash != frame.expected_hash ||
      obs.source_points != frame.expected_points || obs.target_points != target_count ||
      std::abs(obs.length_scale_m - kResolution) > 1e-6 ||
      std::abs(obs.configured_resolution_m - kResolution) > 1e-6 ||
      (obs.actual_grid_leaf_m - Eigen::Vector3d::Constant(leaf[0])).cwiseAbs().maxCoeff() > 1e-6 ||
      std::abs(obs.step_size - .08) > 1e-8 || std::abs(obs.epsilon - 1e-5) > 1e-10 ||
      obs.maximum_iterations != 80 || obs.eigenvalues.minCoeff() <= 0 ||
      !obs.eigenvectors.allFinite() ||
      (obs.eigenvectors.transpose()*obs.eigenvectors-Matrix6d::Identity()).norm() > 1e-3 ||
      translationDistance(obs.pose,frame.baseline_terminal) > .001 ||
      rotationDistanceDeg(obs.pose,frame.baseline_terminal) > .01)
    throw std::runtime_error("frozen frame/U_obs contract mismatch");
  context.source = preprocessSource(loadPackedSource(frame.path));
  if (context.source->size() != frame.expected_points ||
      sourceHash(*context.source) != frame.expected_hash)
    throw std::runtime_error("prepared source provenance mismatch");
  context.k = obs.eigenvalues(1)/obs.eigenvalues(0) >= 2.0 ? 1 : 2;
  context.weak = obs.eigenvectors.leftCols(context.k);
  context.strong = obs.eigenvectors.rightCols(6-context.k);
  context.bounds.resize(context.k);
  for (int axis = 0; axis < context.k; ++axis) {
    const double tn = context.weak.col(axis).head<3>().norm();
    const double rn = context.weak.col(axis).tail<3>().norm();
    context.bounds(axis) = std::min(tn > 1e-12 ? 2.0/(kResolution*tn) : 1e12,
                                  rn > 1e-12 ? 15.0*kPi/(180.0*rn) : 1e12);
  }
  ndt.setInputSource(context.source);
  ndt.dynamicValueOnly(context.source, obs.pose, &context.nominal_support);
  return context;
}

bool insideOriginalBounds(const FrameContext& context, const Eigen::VectorXd& u) {
  const Vector6d eta = context.weak*u;
  return (u.cwiseAbs().array() <= context.bounds.array()+1e-9).all() &&
         kResolution*eta.head<3>().norm() <= 2.0+1e-9 &&
         eta.tail<3>().norm() <= 15.0*kPi/180.0+1e-9;
}

using CandidateKey = std::pair<std::uint64_t, std::string>;
std::map<CandidateKey, PoseCandidate> readRealCandidates(const std::string& path) {
  const CsvTable table = readCsv(path);
  std::map<CandidateKey, PoseCandidate> result;
  for (const auto& row : table.rows) {
    if (table.get(row,"converged") != "1") continue;
    PoseCandidate item;
    item.transaction = parseU64(table.get(row,"transaction_id"));
    item.seed_index = table.get(row,"seed_index");
    item.pose = parseMatrix16(table.get(row,"final_pose_matrix16"));
    item.saved_score = std::stod(table.get(row,"raw_ndt_score_sum"));
    if (!result.emplace(CandidateKey(item.transaction,item.seed_index),item).second)
      throw std::runtime_error("duplicate real candidate seed");
  }
  return result;
}

void configureNdt(ExactPclNdt& ndt, const Cloud::Ptr& target) {
  ndt.setResolution(static_cast<float>(kResolution));
  ndt.setOulierRatio(kOutlierRatio);
  ndt.configureScoreConstants();
  ndt.setInputTarget(target);
  ndt.setStepSize(.08);
  ndt.setTransformationEpsilon(1e-5);
  ndt.setMaximumIterations(80);
}

void canonicalize(const std::string& map_path, const std::string& cohort_path,
    const std::string& candidate_path, const std::string& uobs_path,
    const std::string& request_path, const std::string& out_dir) {
  const auto frames = readFrameSources(cohort_path);
  const auto observations = readArchivedUObs(uobs_path);
  const auto candidates = readRealCandidates(candidate_path);
  const CsvTable requests = readCsv(request_path);
  if (requests.rows.size() != 22) throw std::runtime_error("canonical target count is not 22");
  Cloud::Ptr target = loadTarget(map_path);
  if (target->size() != 549606) throw std::runtime_error("target provenance count mismatch");
  ExactPclNdt ndt;
  configureNdt(ndt,target);
  std::ofstream output(out_dir+"/canonical_oracle.csv");
  if (!output) throw std::runtime_error("cannot open canonical output");
  writeHeader(output,{"transaction_id","frame_id","cluster_id","seed_count","canonical_seed_index",
      "member_seed_indices","representative_pose_matrix16","canonical_pose_matrix16","closed_pose_matrix16",
      "representative_canonical_translation_m","representative_canonical_rotation_deg",
      "representative_mean_energy","canonical_mean_energy","closed_mean_energy","energy_change",
      "gradient_norm_before","gradient_norm_after","hessian_eigenvalues_before","hessian_eigenvalues_after",
      "refine_translation_m","refine_rotation_deg","refine_iterations","refine_converged","refine_status",
      "refine_runtime_ms","cluster_changed_after_refine","complete_link_admission_original",
      "max_member_translation_m","max_member_rotation_deg","support_changed_from_canonical",
      "support_changed_from_T0","value_only_score_abs_error_before","value_only_score_abs_error_after",
      "delta_b","u_b","v_b","delta_norm","u_norm","v_norm","weak_projection",
      "reconstruction_error","reconstruction_translation_m","reconstruction_rotation_deg",
      "inside_fallback","group_a","canonical_status"});
  std::uint64_t current_tx = 0;
  FrameContext context;
  int changed = 0, nonstationary = 0, group_a = 0;
  std::set<std::pair<std::uint64_t,std::string>> identities;
  for (const auto& row : requests.rows) {
    const std::uint64_t tx = parseU64(requests.get(row,"transaction_id"));
    if (tx != current_tx) {
      context = frameContext(frames.at(tx), observations.at(tx),ndt,target->size());
      current_tx = tx;
    }
    const std::string cluster = requests.get(row,"cluster_id");
    if (!identities.emplace(tx,cluster).second) throw std::runtime_error("duplicate oracle target");
    const std::string seed = requests.get(row,"canonical_seed_index");
    const PoseCandidate& canonical = candidates.at(CandidateKey(tx,seed));
    const Eigen::Matrix4f rep = parseMatrix16(requests.get(row,"representative_pose_matrix16"));
    if ((canonical.pose-parseMatrix16(requests.get(row,"canonical_pose_matrix16"))).norm() > 1e-7 ||
        std::abs(canonical.saved_score-std::stod(requests.get(row,"canonical_score"))) > 1e-8)
      throw std::runtime_error("canonical request does not match real candidate");
    std::vector<const PoseCandidate*> members;
    double best_score = -std::numeric_limits<double>::infinity();
    for (const std::string& member_seed : split(requests.get(row,"member_seed_indices"),';')) {
      const auto& member = candidates.at(CandidateKey(tx,member_seed));
      members.push_back(&member);
      best_score = std::max(best_score,member.saved_score);
    }
    if (std::abs(best_score-canonical.saved_score) > 1e-8 ||
        members.size() != parseU64(requests.get(row,"seed_count")) ||
        std::find(members.begin(),members.end(),&canonical)==members.end())
      throw std::runtime_error("canonical is not highest-score real member");
    const EnergyJet before = evaluate(ndt,context.source,canonical.pose);
    const double value_before = ndt.dynamicValueOnly(context.source,canonical.pose);
    const double rep_energy = -ndt.dynamicValueOnly(context.source,rep)/context.source->size();
    if (std::abs(before.score-canonical.saved_score) > 1e-8 ||
        std::abs(value_before-before.score) > 1e-8)
      throw std::runtime_error("canonical same-objective/value-only score mismatch");
    Cloud aligned;
    const auto start = std::chrono::steady_clock::now();
    ndt.align(aligned,canonical.pose);
    const double runtime = std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-start).count();
    const Eigen::Matrix4f closed = ndt.getFinalTransformation();
    const std::string status = ndtStatus(ndt);
    if (!closed.allFinite()) throw std::runtime_error("canonical refine nonfinite");
    const EnergyJet after = evaluate(ndt,context.source,closed);
    const double value_after = ndt.dynamicValueOnly(context.source,closed);
    if (std::abs(value_after-after.score) > 1e-8) throw std::runtime_error("value-only score mismatch after refine");
    const bool cluster_changed = !sameBasin(canonical.pose,closed);
    double max_dt = 0, max_dr = 0;
    for (const PoseCandidate* member : members) {
      max_dt = std::max(max_dt,translationDistance(member->pose,closed));
      max_dr = std::max(max_dr,rotationDistanceDeg(member->pose,closed));
    }
    const bool complete_link = max_dt <= .2 && max_dr <= 2.0;
    const Vector6d delta = mapChartDisplacement(context.obs.pose,closed);
    const Eigen::VectorXd u = context.weak.transpose()*delta;
    const Eigen::VectorXd v = context.strong.transpose()*delta;
    const Vector6d reconstructed = context.weak*u+context.strong*v;
    const Eigen::Matrix4f reconstructed_pose = poseAtEta(context.obs.pose,reconstructed);
    const double projection = (context.weak*u).norm()/delta.norm();
    const bool inside = insideOriginalBounds(context,u);
    const bool eligible = status=="SUCCESS" && !cluster_changed && projection >= .8 && inside;
    nonstationary += before.gradient_eta_norm > .1;
    changed += cluster_changed;
    group_a += eligible;
    writeRow(output,{std::to_string(tx),context.frame.frame_id,cluster,std::to_string(members.size()),seed,
        requests.get(row,"member_seed_indices"),poseText(rep),poseText(canonical.pose),poseText(closed),
        number(translationDistance(rep,canonical.pose)),number(rotationDistanceDeg(rep,canonical.pose)),
        number(rep_energy),number(before.mean_energy),number(after.mean_energy),number(after.mean_energy-before.mean_energy),
        number(before.gradient_eta_norm),number(after.gradient_eta_norm),vectorText(before.eigenvalues),vectorText(after.eigenvalues),
        number(translationDistance(canonical.pose,closed)),number(rotationDistanceDeg(canonical.pose,closed)),
        std::to_string(ndt.getFinalNumIteration()),ndt.hasConverged()?"1":"0",status,number(runtime),
        cluster_changed?"1":"0",complete_link?"1":"0",number(max_dt),number(max_dr),
        number(double(compareSupport(before.support,after.support).changed_points)/context.source->size()),
        number(double(compareSupport(context.nominal_support,after.support).changed_points)/context.source->size()),
        number(std::abs(value_before-before.score)),number(std::abs(value_after-after.score)),
        vectorText(delta),vectorText(u),vectorText(v),number(delta.norm()),number(u.norm()),number(v.norm()),
        number(projection),number((reconstructed-delta).norm()),number(translationDistance(reconstructed_pose,closed)),
        number(rotationDistanceDeg(reconstructed_pose,closed)),inside?"1":"0",eligible?"1":"0",
        status!="SUCCESS"?"CANONICAL_REFINE_FAILED":
          (cluster_changed?"ORACLE_CLUSTER_NOT_STATIONARY":"CLOSED_CANONICAL_TERMINAL")});
    output.flush();
    std::cout<<"CANONICAL tx="<<tx<<" cluster="<<cluster<<" gradient="<<before.gradient_eta_norm
             <<"->"<<after.gradient_eta_norm<<" moved="<<translationDistance(canonical.pose,closed)
             <<"m/"<<rotationDistanceDeg(canonical.pose,closed)<<"deg changed="<<cluster_changed
             <<" GROUP_A="<<eligible<<std::endl;
  }
  std::cout<<"CANONICAL_TARGETS=22 NONSTATIONARY_BEFORE="<<nonstationary
           <<" CLUSTER_CHANGED="<<changed<<" GROUP_A="<<group_a<<std::endl;
}

#include "p9_strong_profile_solvers.hpp"

Eigen::VectorXd parseVectorText(const std::string& value) {
  const auto parts=split(value,';');
  Eigen::VectorXd result(parts.size());
  for (std::size_t i=0;i<parts.size();++i) result(i)=std::stod(parts[i]);
  if (!result.allFinite()) throw std::runtime_error("nonfinite coordinate vector");
  return result;
}

void recordStrongFd(ExactPclNdt& ndt,const FrameContext& context,const Eigen::VectorXd& u,
    const Eigen::VectorXd& v,const std::string& cluster,const std::string& method,
    const std::string& init,std::ofstream& output) {
  const Vector6d eta=context.weak*u+context.strong*v;
  const Eigen::Matrix4f center=poseAtEta(context.obs.pose,eta);
  Vector6d p,score_gradient;
  Matrix6d score_hessian;
  Eigen::MatrixXd jacobian;
  std::array<Eigen::MatrixXd,6> second;
  if (!strongNativePullback(context,eta,p,jacobian,second))
    throw std::runtime_error("strong FD pullback invalid");
  Support support;ExactPclNdt::FrozenSupport frozen;
  const double score=ndt.scoreJet(context.source,center,&score_gradient,&score_hessian,&support,&frozen,true,&p);
  const double n=context.source->size();
  const Vector6d native_gradient=-score_gradient/n;
  const Eigen::VectorXd gradient=jacobian.transpose()*native_gradient;
  Eigen::MatrixXd hessian=jacobian.transpose()*(-score_hessian/n)*jacobian;
  for (int a=0;a<6;++a) hessian+=native_gradient(a)*second[a];
  const double e0=-score/n;
  for (int direction=0;direction<context.strong.cols();++direction)
    for (double h : {.001,.0005}) {
      const auto plus=poseAtEta(context.obs.pose,eta+h*context.strong.col(direction));
      const auto minus=poseAtEta(context.obs.pose,eta-h*context.strong.col(direction));
      const double fp=-ndt.frozenScore(context.source,plus,frozen)/n;
      const double fm=-ndt.frozenScore(context.source,minus,frozen)/n;
      Support sp,sm;
      const double dp=-ndt.dynamicValueOnly(context.source,plus,&sp)/n;
      const double dm=-ndt.dynamicValueOnly(context.source,minus,&sm)/n;
      const double fg=(fp-fm)/(2*h),dg=(dp-dm)/(2*h);
      const double fc=(fp-2*e0+fm)/(h*h),dc=(dp-2*e0+dm)/(h*h);
      writeRow(output,{std::to_string(context.frame.transaction),cluster,method,init,
        std::to_string(direction),number(h),number(e0),number(gradient(direction)),number(hessian(direction,direction)),
        number(fg),number(dg),number(fc),number(dc),number(std::abs(fg-gradient(direction))),
        number(std::abs(dg-gradient(direction))),number(std::abs(fc-hessian(direction,direction))),
        number(std::abs(dc-hessian(direction,direction))),number(supportFraction(support,sp)),number(supportFraction(support,sm))});
    }
}

void compareTrueUbSolvers(const std::string& map_path,const std::string& cohort_path,
    const std::string& candidate_path,const std::string& uobs_path,
    const std::string& canonical_path,const std::string& out_dir) {
  const auto frames=readFrameSources(cohort_path);
  const auto observations=readArchivedUObs(uobs_path);
  const auto candidates=readRealCandidates(candidate_path);
  const CsvTable canonicals=readCsv(canonical_path);
  if (canonicals.rows.size()!=22) throw std::runtime_error("solver requires fixed22 oracle rows");
  const Cloud::Ptr target=loadTarget(map_path);
  if (target->size()!=549606) throw std::runtime_error("solver target count mismatch");
  ExactPclNdt ndt; configureNdt(ndt,target);
  std::ofstream results(out_dir+"/true_ub_solver_comparison.csv");
  std::ofstream traces(out_dir+"/strong_solver_trace.csv");
  std::ofstream fd(out_dir+"/strong_directional_fd.csv");
  if (!results || !traces || !fd) throw std::runtime_error("cannot open solver outputs");
  writeHeader(fd,{"transaction_id","cluster_id","method","initialization","strong_direction","h",
    "mean_energy","analytic_gradient","analytic_curvature","frozen_fd_gradient","dynamic_fd_gradient",
    "frozen_fd_curvature","dynamic_fd_curvature","frozen_gradient_absolute_error","dynamic_gradient_absolute_error",
    "frozen_curvature_absolute_error","dynamic_curvature_absolute_error","support_changed_plus","support_changed_minus"});
  writeHeader(results,{"transaction_id","cluster_id","method","initialization","k","u_b","v_b","initial_v",
    "endpoint_v","endpoint_eta","initial_energy","endpoint_energy","energy_evaluations","strong_iterations",
    "accepted_steps","solver_status","solver_valid","solver_runtime_ms","endpoint_pose_matrix16",
    "endpoint_distance_translation_m","endpoint_distance_rotation_deg","endpoint_gradient_norm",
    "endpoint_projected_gradient_norm","endpoint_support_from_T0","initial_support_from_T0",
    "max_accepted_support_change","full_refine_count","full_refine_iterations","full_refine_converged",
    "full_refine_status","full_refine_runtime_ms","refined_pose_matrix16","refined_mean_energy",
    "refined_gradient_norm","refined_distance_translation_m","refined_distance_rotation_deg",
    "refined_matches_canonical","recovered_at_true_ub","endpoint_matches_canonical","diagnostic_evaluations",
    "strong_fd_energy_evaluations"});
  writeHeader(traces,{"transaction_id","cluster_id","method","initialization","iteration","evaluations",
    "v","energy","support_from_T0","support_from_previous_accepted","branch_projected_gradient_before_step","step_or_trust_scale"});
  std::uint64_t current_tx=0;
  FrameContext context;
  int targets=0;
  for (const auto& row:canonicals.rows) {
    if (canonicals.get(row,"group_a")!="1") continue;
    const auto tx=parseU64(canonicals.get(row,"transaction_id"));
    const std::string cluster=canonicals.get(row,"cluster_id");
    if (tx!=current_tx) {context=frameContext(frames.at(tx),observations.at(tx),ndt,target->size());current_tx=tx;}
    const auto& real=candidates.at(CandidateKey(tx,canonicals.get(row,"canonical_seed_index")));
    if ((real.pose-parseMatrix16(canonicals.get(row,"canonical_pose_matrix16"))).norm()>1e-7)
      throw std::runtime_error("solver canonical seed not real archived candidate");
    const Eigen::Matrix4f closed=parseMatrix16(canonicals.get(row,"closed_pose_matrix16"));
    const Eigen::VectorXd u=parseVectorText(canonicals.get(row,"u_b"));
    const Eigen::VectorXd v_b=parseVectorText(canonicals.get(row,"v_b"));
    if (u.size()!=context.k || v_b.size()!=6-context.k ||
        (mapChartDisplacement(context.obs.pose,closed)-(context.weak*u+context.strong*v_b)).norm()>1e-6)
      throw std::runtime_error("true-u product-chart reconstruction mismatch");
    ++targets;
    for (int variant=0;variant<5;++variant) {
      const bool oracle=variant>=3;
      const std::string method=variant==0?"ONE_STEP_NEWTON":
        ((variant==1 || variant==3)?"ITERATIVE_PROJECTED_NEWTON":"DERIVATIVE_FREE_PATTERN");
      const std::string init=oracle?"ORACLE_V_INIT_DIAGNOSTIC":"ZERO_V";
      const Eigen::VectorXd initial=oracle?v_b:Eigen::VectorXd::Zero(6-context.k);
      StrongSolution solution;
      bool valid=true;
      if (variant==0) {
        const auto start=std::chrono::steady_clock::now();
        const Eigen::Vector3d euler=context.obs.pose.block<3,3>(0,0).eulerAngles(0,1,2).cast<double>();
        const ProfileNode legacy=profileAt(ndt,context.source,context.obs.pose,euler,
            context.weak,context.strong,u,context.nominal_support);
        solution.runtime_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        solution.v=context.strong.transpose()*(legacy.eta-context.weak*u);
        solution.energy=legacy.energy;solution.evaluations=legacy.energy_evaluations;
        solution.iterations=1;solution.accepted_steps=legacy.strong_iterations;solution.status=legacy.status;
        valid=legacy.valid;
        solution.initial_energy=-ndt.dynamicValueOnly(context.source,
          poseAtEta(context.obs.pose,context.weak*u))/context.source->size();
      } else if (variant==1 || variant==3) solution=iterativeNewton(ndt,context,u,initial);
      else solution=derivativeFree(ndt,context,u,initial);
      const Vector6d endpoint_eta=context.weak*u+context.strong*solution.v;
      const Eigen::Matrix4f endpoint=poseAtEta(context.obs.pose,endpoint_eta);
      const EnergyJet endpoint_jet=evaluate(ndt,context.source,endpoint);
      Support initial_support;
      ndt.dynamicValueOnly(context.source,poseAtEta(context.obs.pose,context.weak*u+context.strong*initial),&initial_support);
      if (valid && std::abs(endpoint_jet.mean_energy-solution.energy)>1e-10)
        throw std::runtime_error("solver endpoint objective mismatch");
      double max_switch=0;
      for (const auto& item:solution.trace) {
        max_switch=std::max(max_switch,item.support_from_previous);
        writeRow(traces,{std::to_string(tx),cluster,method,init,std::to_string(item.iteration),
          std::to_string(item.evaluations),vectorText(item.v),number(item.energy),number(item.support_from_T0),
          number(item.support_from_previous),number(item.projected_gradient_before_step),number(item.scale)});
      }
      Eigen::Matrix4f refined=endpoint;
      std::string full_status="SKIPPED_INVALID_SOLVER";
      int align_iterations=0; bool converged=false;
      double refine_ms=0;
      if (valid) {
        Cloud aligned;
        const auto start=std::chrono::steady_clock::now();
        ndt.align(aligned,endpoint);
        refine_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        refined=ndt.getFinalTransformation(); full_status=ndtStatus(ndt);
        align_iterations=ndt.getFinalNumIteration();converged=ndt.hasConverged();
      }
      const EnergyJet final_jet=evaluate(ndt,context.source,refined);
      const bool matches=sameBasin(closed,refined);
      const bool recovered=valid && full_status=="SUCCESS" && matches;
      // evaluate() uses a local product chart centered at the endpoint. Pull
      // its gradient back through the global Exp(eta_rotation) map chart.
      const Eigen::Vector3d theta=endpoint_eta.tail<3>();
      Eigen::Matrix3d hat;
      hat<<0,-theta.z(),theta.y(),theta.z(),0,-theta.x(),-theta.y(),theta.x(),0;
      const double angle=theta.norm();
      Eigen::Matrix3d left_jacobian=Eigen::Matrix3d::Identity();
      if (angle<1e-5) left_jacobian+=.5*hat+(1.0/6.0)*hat*hat;
      else left_jacobian+=(1-std::cos(angle))/(angle*angle)*hat+
        (angle-std::sin(angle))/(angle*angle*angle)*hat*hat;
      Matrix6d global_to_local=Matrix6d::Identity();
      global_to_local.block<3,3>(3,3)=left_jacobian;
      const double projected_norm=(context.strong.transpose()*global_to_local.transpose()*endpoint_jet.gradient_eta).norm();
      recordStrongFd(ndt,context,u,solution.v,cluster,method,init,fd);
      writeRow(results,{std::to_string(tx),cluster,method,init,std::to_string(context.k),vectorText(u),vectorText(v_b),
        vectorText(initial),vectorText(solution.v),vectorText(endpoint_eta),number(solution.initial_energy),
        number(solution.energy),std::to_string(solution.evaluations),std::to_string(solution.iterations),
        std::to_string(solution.accepted_steps),solution.status,valid?"1":"0",number(solution.runtime_ms),poseText(endpoint),
        number(translationDistance(endpoint,closed)),number(rotationDistanceDeg(endpoint,closed)),
        number(endpoint_jet.gradient_eta_norm),number(projected_norm),
        number(supportFraction(context.nominal_support,endpoint_jet.support)),
        number(supportFraction(context.nominal_support,initial_support)),number(max_switch),valid?"1":"0",
        std::to_string(align_iterations),converged?"1":"0",full_status,number(refine_ms),poseText(refined),
        number(final_jet.mean_energy),number(final_jet.gradient_eta_norm),number(translationDistance(refined,closed)),
        number(rotationDistanceDeg(refined,closed)),matches?"1":"0",recovered?"1":"0",
        sameBasin(endpoint,closed)?"1":"0",variant==0?"4":"3",
        std::to_string(1+8*context.strong.cols())});
      results.flush();traces.flush();fd.flush();
      std::cout<<"TRUE_UB tx="<<tx<<" cluster="<<cluster<<" method="<<method<<" init="<<init
        <<" E="<<solution.initial_energy<<"->"<<solution.energy<<" evals="<<solution.evaluations
        <<" status="<<solution.status<<" full="<<full_status<<" distance="<<translationDistance(refined,closed)
        <<"m/"<<rotationDistanceDeg(refined,closed)<<"deg recovered="<<recovered<<std::endl;
    }
  }
  std::cout<<"TRUE_UB_GROUP_A_TARGETS="<<targets<<std::endl;
}

bool runClosureSelfTests() {
  if (!runMathSelfTests()) return false;
  Cloud::Ptr target(new Cloud), source(new Cloud);
  for (int cell = 0; cell < 4; ++cell) for (int point = 0; point < 20; ++point)
    target->push_back(Point(.8f*cell+.1f+.015f*(point%4),
                           .2f+.012f*((point/4)%4),.3f+.01f*(point%3)));
  finalize(target);
  for (int i = 0; i < 16; ++i)
    source->push_back(Point(.1f+.2f*i,.23f,.32f));
  finalize(source);
  ExactPclNdt ndt;
  configureNdt(ndt,target);
  ndt.setInputSource(source);
  for (double offset : {0.0,.01,.25}) {
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose(0,3)=offset;
    Vector6d gradient; Matrix6d hessian; Support a,b;
    const double native = ndt.scoreJet(source,pose,&gradient,&hessian,&a);
    const double value = ndt.dynamicValueOnly(source,pose,&b);
    if (std::abs(native-value)>1e-12 || a!=b) return false;
  }
  Eigen::Matrix4f base=Eigen::Matrix4f::Identity();
  base.block<3,3>(0,0)=Eigen::AngleAxisd(.9,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix().cast<float>();
  Vector6d eta; eta<<.7,-.5,.2,.15,-.2,.05;
  const Eigen::Matrix4f moved=poseAtEta(base,eta);
  const Vector6d back=mapChartDisplacement(base,moved);
  if ((back-eta).norm()>=1e-6 || rotationDistanceDeg(poseAtEta(base,back),moved)>=1e-4) return false;
  FrameContext test;
  test.nominal_support=Support(1);
  Eigen::VectorXd initial=Eigen::VectorXd::Zero(4);
  const Eigen::VectorXd optimum=Eigen::VectorXd::Constant(4,.15);
  auto pattern=patternSearch(initial,[&](const Eigen::VectorXd& v,Support& support) {
    support=Support(1);return (v-optimum).squaredNorm();
  },test);
  if (pattern.evaluations>100 || pattern.evaluations<2 || pattern.energy>=pattern.initial_energy) return false;
  for (std::size_t i=1;i<pattern.trace.size();++i)
    if (pattern.trace[i].energy>=pattern.trace[i-1].energy) return false;
  auto interrupted=patternSearch(initial,[&](const Eigen::VectorXd& v,Support& support) {
    support=Support(1);return (v-Eigen::VectorXd::Constant(4,1.0736)).squaredNorm();
  },test);
  if (interrupted.evaluations!=100 || interrupted.status!="EVALUATION_BUDGET") return false;
  test.source=source;test.obs.pose=Eigen::Matrix4f::Identity();test.k=2;
  test.weak=Matrix6d::Identity().leftCols(2);test.strong=Matrix6d::Identity().rightCols(4);
  ndt.setInputSource(source);
  ndt.dynamicValueOnly(source,test.obs.pose,&test.nominal_support);
  Eigen::VectorXd u(2);u<<.01,-.01;
  const auto newton=iterativeNewton(ndt,test,u,initial);
  if (!newton.v.allFinite() || newton.energy>newton.initial_energy+1e-10 ||
      newton.iterations>20 || newton.evaluations>181) return false;
  for (std::size_t i=1;i<newton.trace.size();++i)
    if (newton.trace[i].energy>=newton.trace[i-1].energy) return false;
  Vector6d tangent; tangent<<.02,-.01,.01,.01,-.02,.01;
  Vector6d parameters,gradient;
  Matrix6d hessian;
  Eigen::MatrixXd pullback;
  std::array<Eigen::MatrixXd,6> second;
  if (!strongNativePullback(test,tangent,parameters,pullback,second)) return false;
  ExactPclNdt::FrozenSupport frozen;
  ndt.scoreJet(source,poseAtEta(test.obs.pose,tangent),&gradient,&hessian,nullptr,&frozen,true,&parameters);
  const Eigen::VectorXd analytic=-pullback.transpose()*gradient/source->size();
  for (int axis=0;axis<4;++axis) {
    constexpr double h=.001;
    const double plus=-ndt.frozenScore(source,poseAtEta(test.obs.pose,tangent+h*test.strong.col(axis)),frozen)/source->size();
    const double minus=-ndt.frozenScore(source,poseAtEta(test.obs.pose,tangent-h*test.strong.col(axis)),frozen)/source->size();
    if (std::abs((plus-minus)/(2*h)-analytic(axis))>1e-3) return false;
  }
  return true;
}

}  // namespace

int main(int argc,char** argv) {
  try {
    if (argc==2 && std::string(argv[1])=="--self-test") {
      if (!runClosureSelfTests()) throw std::runtime_error("R1A exact-value/product-chart self-test failed");
      std::cout<<"P9_R1A_SELF_TEST=PASS\n";
      return 0;
    }
    // Six paths follow the stage flag (argc=8).
    if (argc==8 && std::string(argv[1])=="--canonical") {
      canonicalize(argv[2],argv[3],argv[4],argv[5],argv[6],argv[7]);
      return 0;
    }
    if (argc==8 && std::string(argv[1])=="--solvers") {
      compareTrueUbSolvers(argv[2],argv[3],argv[4],argv[5],argv[6],argv[7]);return 0;
    }
    std::cerr<<"usage: p9_true_profile_closure --self-test\n"
             <<"or: --canonical MAP COHORT CANDIDATES UOBS REQUESTS OUTPUT_DIRECTORY\n"
             <<"or: --solvers MAP COHORT CANDIDATES UOBS CANONICAL OUTPUT_DIRECTORY\n";
    return 2;
  } catch (const std::exception& error) {
    std::cerr<<"P9_R1A_ERROR: "<<error.what()<<'\n';
    return 1;
  }
}
