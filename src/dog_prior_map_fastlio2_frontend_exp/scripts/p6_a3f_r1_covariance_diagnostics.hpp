#pragma once

#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_window.hpp"
#include <Eigen/Eigenvalues>
#include <iomanip>
#include <ostream>
#include <stdexcept>

namespace p6_i6b {
inline std::pair<double,double> covarianceEigenBounds(const Eigen::MatrixXd& p) {
  const double nan=std::numeric_limits<double>::quiet_NaN();
  if (!p.allFinite()) return {nan,nan};
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(p);
  if (eigen.info()!=Eigen::Success) return {nan,nan};
  return {eigen.eigenvalues().minCoeff(),eigen.eigenvalues().maxCoeff()};
}
inline void writeCovarianceRequestHeader(std::ostream& out) {
  out<<std::setprecision(17)<<"transaction_id,stamp_ns,backend,valid,detail,rows,columns,rank,"
      "rank_threshold,min_abs_R,max_abs_R,pivot_ratio,triangular_residual,qr_ms,temporary_estimated_bytes,"
      "P15_min,P15_max,Pmap_min,Pmap_max,legacy_requested,legacy_valid,legacy_detail,legacy_backward_error\n";
}
inline void writeCovarianceRequest(std::ostream& out,std::uint64_t tx,std::uint64_t stamp,
    const dog_prior_map_fastlio2_frontend_exp::fixed_lag::WindowMarginalCovariance& p) {
  const double nan=std::numeric_limits<double>::quiet_NaN();
  const auto c=p.valid?covarianceEigenBounds(p.covariance15):std::make_pair(nan,nan);
  const auto m=p.valid?covarianceEigenBounds(p.map_pose_covariance6):std::make_pair(nan,nan);
  out<<tx<<','<<stamp<<','<<p.backend<<','<<p.valid<<','<<p.detail<<','<<p.rows<<','<<p.columns<<','
      <<p.rank<<','<<p.rank_threshold<<','<<p.min_abs_r<<','<<p.max_abs_r<<','
      <<(p.max_abs_r>0?p.min_abs_r/p.max_abs_r:nan)<<','<<p.normalized_backward_error<<','
      <<p.qr_ms<<','<<p.temporary_estimated_bytes<<','<<c.first<<','<<c.second<<','
      <<m.first<<','<<m.second<<','<<p.legacy_shadow_requested<<','<<p.legacy_shadow_valid<<','
      <<p.legacy_shadow_detail<<','<<p.legacy_backward_error<<'\n';
  out.flush();
  if (!out) throw std::runtime_error("covariance_request_diagnostics_flush_failed");
}
}  // namespace p6_i6b
