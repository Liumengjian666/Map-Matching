#pragma once

#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace r15c {

struct GradientOnlySolve {
  bool ok = false;
  std::string status = "NOT_RUN";
  Eigen::VectorXd ds_gradient_raw;
  Eigen::VectorXd ds_coupling_raw;
  Eigen::VectorXd ds_coupled_raw;
  Eigen::VectorXd ds_gradient_trusted;
  Eigen::VectorXd ds_coupled_trusted;
  double gradient_trust_scale = std::numeric_limits<double>::quiet_NaN();
  double coupled_trust_scale = std::numeric_limits<double>::quiet_NaN();
};

// Isolate the coupling-specific contribution in the same regularized block
// system used by R15B. Inputs are already projected into the frozen
// eigenbasis: Hss(ns x ns), Hsw(ns x nw), gs(ns), dw(nw).
inline GradientOnlySolve solveAttribution(const Eigen::MatrixXd& hss,
                                          const Eigen::MatrixXd& hsw,
                                          const Eigen::VectorXd& gs,
                                          const Eigen::VectorXd& dw,
                                          double lambda,
                                          double unit_trust_radius = 1.0) {
  GradientOnlySolve out;
  const Eigen::Index ns = hss.rows(), nw = dw.size();
  if (ns <= 0 || hss.cols() != ns || hsw.rows() != ns || hsw.cols() != nw ||
      gs.size() != ns || nw <= 0 || !hss.allFinite() || !hsw.allFinite() ||
      !gs.allFinite() || !dw.allFinite() || !std::isfinite(lambda) ||
      lambda < 0.0 || !std::isfinite(unit_trust_radius) || unit_trust_radius <= 0.0) {
    out.status = "INVALID_INPUT";
    return out;
  }
  if (dw.squaredNorm() > unit_trust_radius * unit_trust_radius + 1e-12) {
    out.status = "WEAK_STEP_OUTSIDE_TRUST_REGION";
    return out;
  }
  const Eigen::MatrixXd hreg = 0.5 * (hss + hss.transpose()).eval() +
      lambda * Eigen::MatrixXd::Identity(ns, ns);
  if (!hreg.allFinite()) { out.status = "NONFINITE_REGULARIZED_HSS"; return out; }
  Eigen::LDLT<Eigen::MatrixXd> ldlt(hreg);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
    out.status = "REGULARIZED_HSS_LDLT_REJECTED";
    return out;
  }

  out.ds_gradient_raw = ldlt.solve(-gs);
  out.ds_coupling_raw = ldlt.solve(-(hsw * dw));
  out.ds_coupled_raw = ldlt.solve(-(gs + hsw * dw));
  if (ldlt.info() != Eigen::Success || !out.ds_gradient_raw.allFinite() ||
      !out.ds_coupling_raw.allFinite() || !out.ds_coupled_raw.allFinite()) {
    out.status = "NONFINITE_BLOCK_SOLUTION";
    return out;
  }
  const double decomposition_error =
      (out.ds_gradient_raw + out.ds_coupling_raw - out.ds_coupled_raw).norm();
  if (decomposition_error > 1e-10 * std::max(1.0, out.ds_coupled_raw.norm())) {
    out.status = "COUPLING_DECOMPOSITION_MISMATCH";
    return out;
  }

  const double remaining = std::sqrt(std::max(0.0,
      unit_trust_radius * unit_trust_radius - dw.squaredNorm()));
  const auto scaleToRemaining = [remaining](const Eigen::VectorXd& ds) {
    return ds.norm() > remaining && ds.norm() > 0.0 ? remaining / ds.norm() : 1.0;
  };
  out.gradient_trust_scale = scaleToRemaining(out.ds_gradient_raw);
  out.coupled_trust_scale = scaleToRemaining(out.ds_coupled_raw);
  out.ds_gradient_trusted = out.gradient_trust_scale * out.ds_gradient_raw;
  out.ds_coupled_trusted = out.coupled_trust_scale * out.ds_coupled_raw;
  if (!out.ds_gradient_trusted.allFinite() || !out.ds_coupled_trusted.allFinite() ||
      out.ds_gradient_trusted.norm() * out.ds_gradient_trusted.norm() + dw.squaredNorm() > 1.0 + 1e-10 ||
      out.ds_coupled_trusted.norm() * out.ds_coupled_trusted.norm() + dw.squaredNorm() > 1.0 + 1e-10) {
    out.status = "TRUST_REGION_CHECK_FAILED";
    return out;
  }
  out.ok = true;
  out.status = "GRADIENT_AND_COUPLING_COMPONENTS_SOLVED";
  return out;
}

}  // namespace r15c
