#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_window.hpp"

#include <Eigen/Cholesky>
#include <Eigen/QR>
#include <chrono>
#include <map>

namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag {
namespace {
bool unavailable(WindowMarginalCovariance* output, const std::string& detail,
                 std::string* reason) {
  output->detail = detail;
  if (reason) *reason = output->status + ":" + detail;
  return false;
}
}  // namespace

bool solveSquareRootMarginalCovariance(const Eigen::MatrixXd& a,
    const Eigen::Matrix3d& rotation, WindowMarginalCovariance* output,
    std::string* reason) {
  if (!output) { if (reason) *reason="null_window_marginal_covariance_output"; return false; }
  *output = WindowMarginalCovariance();
  output->backend="SQUARE_ROOT_QR";
  output->rows=a.rows(); output->columns=a.cols();
  const double eps=std::numeric_limits<double>::epsilon();
  if (a.cols()<15 || a.cols()%15 || !a.allFinite() || !rotation.allFinite())
    return unavailable(output,"SQUARE_ROOT_COVARIANCE_INVALID_ROWS",reason);
  if (a.rows()<a.cols())
    return unavailable(output,"SQUARE_ROOT_COVARIANCE_RANK_DEFICIENT",reason);
  // A P = Q R. Eigen's setThreshold takes a RELATIVE pivot threshold.
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(a);
  const double relative_threshold=eps*std::max(a.rows(),a.cols());
  qr.setThreshold(relative_threshold);
  output->rank=qr.rank();
  output->max_abs_r=qr.maxPivot();
  output->min_abs_r=qr.matrixR().topLeftCorner(a.cols(),a.cols()).diagonal().cwiseAbs().minCoeff();
  output->rank_threshold=relative_threshold*qr.maxPivot();
  if (output->rank!=a.cols())
    return unavailable(output,"SQUARE_ROOT_COVARIANCE_RANK_DEFICIENT",reason);
  Eigen::MatrixXd selector=Eigen::MatrixXd::Zero(a.cols(),15);
  selector.bottomRows(15).setIdentity();
  const Eigen::MatrixXd rhs=qr.colsPermutation().transpose()*selector;
  const Eigen::MatrixXd r=qr.matrixR().topLeftCorner(a.cols(),a.cols()).template triangularView<Eigen::Upper>();
  const Eigen::MatrixXd y=r.transpose().template triangularView<Eigen::Lower>().solve(rhs);
  output->normalized_backward_error=(r.transpose()*y-rhs).norm()/std::max(rhs.norm(),eps);
  // Peak estimate includes caller's A/b, QR storage, explicit R, selectors,
  // solve/residual temporaries; excludes shadow and allocator overhead.
  output->temporary_estimated_bytes=sizeof(double)*(
      2*a.size()+a.rows()+2*a.cols()*a.cols()+7*a.cols()*15+4*a.cols());
  if (!y.allFinite() || !std::isfinite(output->normalized_backward_error) ||
      output->normalized_backward_error>1e-10)
    return unavailable(output,"SQUARE_ROOT_COVARIANCE_SOLVE_RESIDUAL",reason);
  output->covariance15=(y.transpose()*y).eval();
  output->covariance15=(0.5*(output->covariance15+output->covariance15.transpose())).eval();
  Eigen::Matrix<double,6,15> map_selector=Eigen::Matrix<double,6,15>::Zero();
  map_selector.block<3,3>(0,0)=rotation;
  map_selector.block<3,3>(3,3).setIdentity();
  const Eigen::MatrixXd y_map=y*map_selector.transpose();
  output->map_pose_covariance6=(y_map.transpose()*y_map).eval();
  output->map_pose_covariance6=(0.5*(output->map_pose_covariance6+
      output->map_pose_covariance6.transpose())).eval();
  if (!output->covariance15.allFinite() ||
      Eigen::LLT<Matrix15d>(output->covariance15).info()!=Eigen::Success)
    return unavailable(output,"MARGINAL_NOT_PSD",reason);
  if (!output->map_pose_covariance6.allFinite() ||
      Eigen::LLT<Matrix6d>(output->map_pose_covariance6).info()!=Eigen::Success)
    return unavailable(output,"MAP_COVARIANCE_NOT_PSD",reason);
  output->valid=true; output->detail="AVAILABLE";
  output->status="WINDOW_MARGINAL_COVARIANCE_AVAILABLE";
  if (reason) reason->clear();
  return true;
}

bool FixedLagWindow::allFactorsSquareRootRowsWithLidarSnapshot(
    const LidarIterationSnapshot& snapshot, SquareRootRows* output,
    std::string* reason) const {
  const auto fail=[&](const char* text) { if (reason) *reason=text; return false; };
  if (!output || states_.empty()) return fail("EMPTY_WINDOW");
  if (snapshot.projections.size()!=lidar_factors_.size()) return fail("square_root_snapshot_count_mismatch");
  std::map<std::uint64_t,const FrozenLidarProjection*> projections;
  for (const auto& p : snapshot.projections)
    if (!p.valid || p.reliable_rank<1 || p.reliable_rank>6 ||
        !projections.emplace(p.observation_id,&p).second)
      return fail("square_root_snapshot_invalid_or_duplicate");
  const Eigen::Index columns=15*states_.size();
  Eigen::Index count=prior_.square_root.a.rows()+15*imu_factors_.size();
  for (const auto& f : lidar_factors_) {
    const auto found=projections.find(f.measurement.observation_id);
    if (found==projections.end() || found->second->stamp_ns!=f.measurement.stamp_ns)
      return fail("square_root_snapshot_identity_mismatch");
    count+=found->second->reliable_rank;
  }
  for (const auto& f : visual_factors_) count+=f.measurement.selected_rank;
  SquareRootRows stack;
  stack.a=Eigen::MatrixXd::Zero(count,columns); stack.b.resize(count);
  SquareRootRows prior_rows;
  if (!linearizeSquareRootPriorRows(&prior_rows,reason)) return false;
  Eigen::Index offset=prior_rows.a.rows();
  stack.a.topRows(offset)=prior_rows.a; stack.b.head(offset)=prior_rows.b;
  const auto append=[&](std::size_t i,const Eigen::MatrixXd& ji,
      std::size_t j,const Eigen::MatrixXd* jj,const Eigen::VectorXd& residual,
      const Eigen::MatrixXd& covariance) {
    // Whiten only the local factor columns, then scatter into full rows.
    Eigen::MatrixXd local(ji.rows(),jj?30:15); local.leftCols(15)=ji;
    if (jj) local.rightCols(15)=*jj;
    SquareRootRows rows;
    if (!whitenSquareRootRows(local,residual,covariance,&rows,reason)) return false;
    if (offset+rows.a.rows()>count) return fail("square_root_row_count_mismatch");
    stack.a.block(offset,i*15,rows.a.rows(),15)=rows.a.leftCols(15);
    if (jj) stack.a.block(offset,j*15,rows.a.rows(),15)+=rows.a.rightCols(15);
    stack.b.segment(offset,rows.b.size())=rows.b;
    offset+=rows.a.rows(); return true;
  };
  for (const auto& f : imu_factors_) {
    std::size_t i=0,j=0;
    if (!findStateIndex(f.from_stamp_ns,&i) || !findStateIndex(f.to_stamp_ns,&j)) return fail("square_root_imu_endpoint_missing");
    Matrix15d ji,jj; Vector15d r;
    if (!linearizeImuFactor(states_[i],states_[j],f.measurement,imu_noise_,&ji,&jj,&r,reason)) return false;
    const Eigen::MatrixXd dynamic_jj=jj;
    if (!append(i,ji,j,&dynamic_jj,r,f.measurement.covariance)) return false;
  }
  for (const auto& f : lidar_factors_) {
    std::size_t i=0;
    if (!findStateIndex(f.measurement.stamp_ns,&i)) return fail("square_root_lidar_endpoint_missing");
    Eigen::MatrixXd j,covariance; Eigen::VectorXd r;
    if (!linearizeLidarFactorWithFrozenProjection(states_[i],f.measurement,
          *projections.at(f.measurement.observation_id),&r,&j,&covariance,reason) ||
        !append(i,j,0,nullptr,r,covariance)) return false;
  }
  for (const auto& f : visual_factors_) {
    std::size_t i=0,j=0;
    if (!findStateIndex(f.measurement.reference_stamp_ns,&i) || !findStateIndex(f.measurement.current_stamp_ns,&j))
      return fail("square_root_visual_endpoint_missing");
    Eigen::MatrixXd ji,jj,covariance; Eigen::VectorXd r;
    if (!linearizeSelectedVisualFactor(states_[i],states_[j],f.measurement,&r,&ji,&jj,&covariance,reason) ||
        !append(i,ji,j,&jj,r,covariance)) return false;
  }
  if (offset!=count) return fail("square_root_row_count_mismatch");
  *output=std::move(stack); return true;
}

bool FixedLagWindow::latestMarginalCovariance(
    WindowMarginalCovariance* output,std::string* reason) const {
  if (reason) reason->clear();
  if (!output) { if (reason) *reason="null_window_marginal_covariance_output"; return false; }
  const auto start=std::chrono::steady_clock::now();
  struct Timer {
    double* ms; std::chrono::steady_clock::time_point start;
    ~Timer() { *ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count(); }
  } timer{&summary_.marginal_covariance_ms,start};
  ++summary_.marginal_covariance_requests;
  *output=WindowMarginalCovariance();
  const bool root=options_.marginal_covariance_backend==MarginalCovarianceBackend::SQUARE_ROOT_QR;
  if (root) output->backend="SQUARE_ROOT_QR";
  LidarIterationSnapshot snapshot;
  std::string detail;
  if (!buildLidarIterationSnapshot(&snapshot,&detail)) return unavailable(output,detail,reason);
  if (!root) {
    const bool ok=latestMarginalCovarianceLegacyWithSnapshot(snapshot,output,&detail);
    output->detail=ok?"AVAILABLE":detail.substr(detail.find(':')+1);
    if (reason) *reason=detail;
    return ok;
  }
  SquareRootRows rows;
  const auto qr_start=std::chrono::steady_clock::now();
  bool ok=allFactorsSquareRootRowsWithLidarSnapshot(snapshot,&rows,&detail);
  if (ok) ok=solveSquareRootMarginalCovariance(rows.a,states_.back().rotation,output,&detail);
  else unavailable(output,detail,reason);
  output->qr_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-qr_start).count();
  if (reason && !ok) *reason=detail;
  if (options_.capture_covariance_shadow) {
    WindowMarginalCovariance legacy;
    std::string shadow_reason;
    output->legacy_shadow_requested=true;
    output->legacy_shadow_valid=latestMarginalCovarianceLegacyWithSnapshot(snapshot,&legacy,&shadow_reason);
    output->legacy_shadow_detail=legacy.valid?"AVAILABLE":shadow_reason.substr(shadow_reason.find(':')+1);
    output->legacy_covariance15=legacy.covariance15;
    output->legacy_map_pose_covariance6=legacy.map_pose_covariance6;
    output->legacy_backward_error=legacy.normalized_backward_error;
  }
  return ok;
}
}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
