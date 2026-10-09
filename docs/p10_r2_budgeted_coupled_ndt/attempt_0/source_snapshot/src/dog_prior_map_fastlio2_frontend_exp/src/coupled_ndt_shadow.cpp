#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_shadow.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <numeric>
#include <set>

namespace dog_prior_map_fastlio2_frontend_exp {
namespace {
using Clock = std::chrono::steady_clock;
constexpr double kPi = 3.14159265358979323846;
double elapsedMs(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

Eigen::Matrix3d expRotation(const Eigen::Vector3d& theta) {
  const double angle = theta.norm();
  return angle < 1e-14 ? Eigen::Matrix3d::Identity()
                     : Eigen::AngleAxisd(angle, theta / angle).toRotationMatrix();
}
Eigen::Matrix4f poseAtEta(const Eigen::Matrix4f& base, const CoupledVector6& eta,
                        double length_scale) {
  Eigen::Matrix4f pose = base;
  // Preserve the frozen P9 float pose carrier operations.
  pose.block<3, 1>(0, 3) += static_cast<float>(length_scale) * eta.head<3>().cast<float>();
  pose.block<3, 3>(0, 0) =
      (expRotation(eta.tail<3>()) * base.block<3, 3>(0, 0).cast<double>()).cast<float>();
  return pose;
}
bool unwrapEulerNear(const Eigen::Vector3d& raw, const Eigen::Vector3d& reference,
                     Eigen::Vector3d& output) {
  double best = std::numeric_limits<double>::infinity();
  for (int branch = 0; branch < 2; ++branch) {
    Eigen::Vector3d candidate = raw;
    if (branch) {
      candidate.x() += kPi;
      candidate.y() = kPi - candidate.y();
      candidate.z() += kPi;
    }
    for (int axis = 0; axis < 3; ++axis)
      candidate(axis) += 2 * kPi * std::round((reference(axis) - candidate(axis)) / (2 * kPi));
    const double distance = (candidate - reference).squaredNorm();
    if (distance < best) { best = distance; output = candidate; }
  }
  return std::isfinite(best) && output.allFinite();
}
// Same global map-product chart and XYZ native Euler branch as P9/R1.
bool parametersAtEta(const Eigen::Matrix4f& base, const Eigen::Vector3d& reference,
                     const CoupledVector6& eta, double length_scale,
                     CoupledVector6& parameters, double max_euler_delta = 4.0) {
  if (!eta.allFinite()) return false;
  const Eigen::Matrix3d rotation = expRotation(eta.tail<3>()) *
      base.block<3, 3>(0, 0).cast<double>();
  Eigen::Vector3d angles = Eigen::Vector3d::Zero();
  if (!unwrapEulerNear(rotation.eulerAngles(0, 1, 2), reference, angles)) return false;
  const Eigen::Matrix3d rebuilt =
      (Eigen::AngleAxisd(angles.x(), Eigen::Vector3d::UnitX()) *
       Eigen::AngleAxisd(angles.y(), Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(angles.z(), Eigen::Vector3d::UnitZ())).toRotationMatrix();
  if ((rebuilt - rotation).norm() > 1e-6 ||
      (angles - reference).norm() > max_euler_delta) return false;
  parameters.head<3>() = base.block<3, 1>(0, 3).cast<double>() + length_scale * eta.head<3>();
  parameters.tail<3>() = angles;
  return parameters.allFinite();
}
struct Pullback {
  CoupledVector6 parameters = CoupledVector6::Zero();
  CoupledMatrix6 J = CoupledMatrix6::Zero();
  std::array<CoupledMatrix6, 6> K;
  bool valid = false;
  Pullback() { for (auto& matrix : K) matrix.setZero(); }
};
bool wellConditioned(const CoupledMatrix6& J) {
  Eigen::JacobiSVD<CoupledMatrix6> svd(J);
  const auto singular = svd.singularValues();
  return singular.allFinite() && singular.minCoeff() > 1e-9 &&
      singular.maxCoeff() / singular.minCoeff() < 1e4;
}
// Mirror P9 buildProductChart at eta=0, including its float Euler anchor.
// This keeps the nominal weak/strong eigenbasis compatible with archived U_obs.
Pullback nominalPullback(const Eigen::Matrix4f& base, double length_scale) {
  Pullback pull;
  const Eigen::Vector3d reference = base.block<3, 3>(0, 0).eulerAngles(0, 1, 2).cast<double>();
  pull.parameters.head<3>() = base.block<3, 1>(0, 3).cast<double>();
  pull.parameters.tail<3>() = reference;
  pull.J.topLeftCorner<3, 3>() = length_scale * Eigen::Matrix3d::Identity();
  constexpr double jh = 1e-5, hh = 1e-3;
  auto native = [&](const CoupledVector6& eta, CoupledVector6& p) {
    return parametersAtEta(base, reference, eta, length_scale, p, .25);
  };
  const CoupledVector6 zero = CoupledVector6::Zero();
  CoupledVector6 center;
  if (!native(zero, center)) return pull;
  for (int i = 3; i < 6; ++i) {
    const CoupledVector6 axis = CoupledMatrix6::Identity().col(i);
    CoupledVector6 plus, minus;
    if (!native(jh * axis, plus) || !native(-jh * axis, minus)) return pull;
    pull.J.block<3, 1>(3, i) = (plus.tail<3>() - minus.tail<3>()) / (2 * jh);
    if (!native(hh * axis, plus) || !native(-hh * axis, minus)) return pull;
    for (int a = 3; a < 6; ++a) pull.K[a](i, i) = (plus(a) - 2 * center(a) + minus(a)) / (hh * hh);
    for (int j = i + 1; j < 6; ++j) {
      const CoupledVector6 other = CoupledMatrix6::Identity().col(j);
      CoupledVector6 pp, pm, mp, mm;
      if (!native(hh * (axis + other), pp) || !native(hh * (axis - other), pm) ||
          !native(hh * (-axis + other), mp) || !native(-hh * (axis + other), mm)) return pull;
      for (int a = 3; a < 6; ++a)
        pull.K[a](i, j) = pull.K[a](j, i) = (pp(a) - pm(a) - mp(a) + mm(a)) / (4 * hh * hh);
    }
  }
  pull.valid = wellConditioned(pull.J);
  return pull;
}
// Full six-coordinate R1 strongNativePullback, with Q=[W,S]. In particular,
// mixed K entries and the current NONZERO global eta must not be discarded.
Pullback jointPullback(const Eigen::Matrix4f& base, const CoupledVector6& eta,
                      const CoupledMatrix6& Q, double length_scale) {
  Pullback pull;
  const Eigen::Vector3d base_euler = base.block<3, 3>(0, 0).eulerAngles(0, 1, 2).cast<double>();
  if (!parametersAtEta(base, base_euler, eta, length_scale, pull.parameters)) return pull;
  const Eigen::Vector3d current_euler = pull.parameters.tail<3>();
  auto native = [&](const CoupledVector6& trial, CoupledVector6& p) {
    return parametersAtEta(base, current_euler, trial, length_scale, p);
  };
  constexpr double jh = 1e-5, hh = 1e-3;
  for (int i = 0; i < 6; ++i) {
    CoupledVector6 plus, minus;
    if (!native(eta + jh * Q.col(i), plus) || !native(eta - jh * Q.col(i), minus)) return pull;
    pull.J.col(i) = (plus - minus) / (2 * jh);
    if (!native(eta + hh * Q.col(i), plus) || !native(eta - hh * Q.col(i), minus)) return pull;
    const CoupledVector6 diagonal = (plus - 2 * pull.parameters + minus) / (hh * hh);
    for (int a = 0; a < 6; ++a) pull.K[a](i, i) = diagonal(a);
    for (int j = i + 1; j < 6; ++j) {
      const CoupledVector6 di = hh * Q.col(i), dj = hh * Q.col(j);
      CoupledVector6 pp, pm, mp, mm;
      if (!native(eta + di + dj, pp) || !native(eta + di - dj, pm) ||
          !native(eta - di + dj, mp) || !native(eta - di - dj, mm)) return pull;
      const CoupledVector6 mixed = (pp - pm - mp + mm) / (4 * hh * hh);
      for (int a = 0; a < 6; ++a) pull.K[a](i, j) = pull.K[a](j, i) = mixed(a);
    }
  }
  pull.valid = wellConditioned(pull.J);
  return pull;
}
struct JointJet {
  CoupledVector6 gradient = CoupledVector6::Zero();
  CoupledMatrix6 H = CoupledMatrix6::Zero();
  double score = std::numeric_limits<double>::quiet_NaN();
  bool valid = false;
};
JointJet pullNativeJet(const CoupledNativeJet& native, const Pullback& pull, double count) {
  JointJet joint;
  joint.score = native.score_sum;
  if (!pull.valid || !native.valid || !std::isfinite(native.score_sum) ||
      !native.score_gradient.allFinite() || !native.score_hessian.allFinite()) return joint;
  const CoupledVector6 g = -native.score_gradient / count;
  const CoupledMatrix6 H = -native.score_hessian / count;
  joint.gradient = pull.J.transpose() * g;
  joint.H = pull.J.transpose() * H * pull.J;
  for (int a = 0; a < 6; ++a) joint.H += g(a) * pull.K[a];
  joint.H = .5 * (joint.H + joint.H.transpose()).eval();
  joint.valid = joint.gradient.allFinite() && joint.H.allFinite();
  return joint;
}

struct GridNode { int id; Eigen::VectorXd u; };
struct WeakGrid {
  Eigen::MatrixXd W;
  Eigen::VectorXd bounds;
  std::vector<GridNode> admissible;
};
WeakGrid weakGrid(const CoupledMatrix6& Q, int k, const CoupledNdtConfig& config) {
  WeakGrid grid;
  grid.W = Q.leftCols(k);
  grid.bounds.resize(k);
  for (int i = 0; i < k; ++i) {
    const double tn = grid.W.col(i).head<3>().norm(), rn = grid.W.col(i).tail<3>().norm();
    grid.bounds(i) = std::min(tn > 1e-12 ? config.weak_translation_bound_m / (config.translation_scale_m * tn) : 1e12,
        rn > 1e-12 ? config.weak_rotation_bound_deg * kPi / (180 * rn) : 1e12);
  }
  const int n = k == 1 ? 17 : 9, columns = k == 1 ? 1 : n;
  for (int i = 0; i < n; ++i) for (int j = 0; j < columns; ++j) {
    Eigen::VectorXd u(k);
    u(0) = -grid.bounds(0) + 2 * grid.bounds(0) * i / (n - 1);
    if (k == 2) u(1) = -grid.bounds(1) + 2 * grid.bounds(1) * j / (n - 1);
    const CoupledVector6 eta = grid.W * u;
    if (u.norm() <= 1e-12 || config.translation_scale_m * eta.head<3>().norm() > config.weak_translation_bound_m + 1e-9 ||
        eta.tail<3>().norm() > config.weak_rotation_bound_deg * kPi / 180 + 1e-9) continue;
    grid.admissible.push_back({i * columns + j, u});
  }
  return grid;
}
double normalizedDistance(const Eigen::VectorXd& a, const Eigen::VectorXd& b,
                          const Eigen::VectorXd& bounds) {
  return ((a - b).array() / bounds.array()).matrix().squaredNorm();
}
// Farthest-point coverage uses the full ORIGINAL admissible grid, seeded by
// u=0. Zero participates in distances but is never a duplicate proposal.
std::vector<GridNode> coveringProposals(const WeakGrid& grid, int maximum) {
  std::vector<GridNode> remaining = grid.admissible, selected;
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(grid.bounds.size());
  while (!remaining.empty() && static_cast<int>(selected.size()) < maximum) {
    std::size_t best = 0;
    double largest = -1;
    for (std::size_t i = 0; i < remaining.size(); ++i) {
      double nearest = normalizedDistance(remaining[i].u, zero, grid.bounds);
      for (const auto& previous : selected)
        nearest = std::min(nearest, normalizedDistance(remaining[i].u, previous.u, grid.bounds));
      if (nearest > largest + 1e-12 ||
          (std::abs(nearest - largest) <= 1e-12 && remaining[i].id < remaining[best].id)) {
        best = i; largest = nearest;
      }
    }
    selected.push_back(remaining[best]);
    remaining.erase(remaining.begin() + best);
  }
  return selected;
}
std::vector<GridNode> stageVisitOrder(std::vector<GridNode> remaining, Eigen::VectorXd previous) {
  std::vector<GridNode> ordered;
  while (!remaining.empty()) {
    std::size_t best = 0;
    double nearest = (remaining[0].u - previous).squaredNorm();
    for (std::size_t i = 1; i < remaining.size(); ++i) {
      const double distance = (remaining[i].u - previous).squaredNorm();
      if (distance < nearest - 1e-12 ||
          (std::abs(distance - nearest) <= 1e-12 && remaining[i].id < remaining[best].id)) {
        best = i; nearest = distance;
      }
    }
    ordered.push_back(remaining[best]); previous = remaining[best].u;
    remaining.erase(remaining.begin() + best);
  }
  return ordered;
}

struct StrongStep {
  Eigen::VectorXd coupling, gradient;
  double damping = 0, condition = 0, residual = 0;
  bool valid = false, capped = false;
  std::string reason = "DERIVATIVE_NONFINITE";
};
StrongStep strongStep(const JointJet& jet, int k, const Eigen::VectorXd& du,
                      bool include_gradient, double cap) {
  StrongStep step;
  const int n = 6 - k;
  step.coupling = step.gradient = Eigen::VectorXd::Zero(n);
  if (!jet.valid || !du.allFinite()) return step;
  const Eigen::MatrixXd Hvv = jet.H.bottomRightCorner(n, n);
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(Hvv);
  step.reason = "HVV_INVALID";
  if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite()) return step;
  if (include_gradient) {
    const double floor = 1e-4 * std::max(1.0, Hvv.diagonal().cwiseAbs().maxCoeff());
    step.damping = std::max(0.0, floor - eig.eigenvalues().minCoeff());
  }
  const Eigen::MatrixXd model = Hvv + step.damping * Eigen::MatrixXd::Identity(n, n);
  const double minimum = eig.eigenvalues().minCoeff() + step.damping;
  step.reason = "HVV_NOT_SPD";
  if (minimum <= 0) return step;
  step.condition = (eig.eigenvalues().maxCoeff() + step.damping) / minimum;
  step.reason = "HVV_ILL_CONDITIONED";
  if (!std::isfinite(step.condition) || step.condition > 1e8) return step;
  Eigen::MatrixXd rhs(n, 2);
  rhs.col(0) = -jet.H.bottomLeftCorner(n, k) * du;
  rhs.col(1) = include_gradient ? (-jet.gradient.tail(n)).eval() : Eigen::VectorXd::Zero(n);
  Eigen::LDLT<Eigen::MatrixXd> ldlt(model);
  const Eigen::MatrixXd solved = ldlt.solve(rhs);
  step.reason = "STRONG_SOLVE_FAILED";
  if (ldlt.info() != Eigen::Success || !solved.allFinite()) return step;
  step.residual = (model * solved - rhs).norm() / std::max(1e-12, rhs.norm());
  if (!std::isfinite(step.residual) || step.residual > 1e-6) return step;
  step.coupling = solved.col(0); step.gradient = solved.col(1);
  const double norm = (step.coupling + step.gradient).norm();
  if (!std::isfinite(norm)) return step;
  step.capped = norm > cap;
  if (step.capped) { step.coupling *= cap / norm; step.gradient *= cap / norm; }
  step.valid = true; step.reason = "OK";
  return step;
}
double translationDistance(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
  return (a.block<3, 1>(0, 3).cast<double>() - b.block<3, 1>(0, 3).cast<double>()).norm();
}
double rotationDistanceDeg(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
  // Normalize the float carriers before measuring separation.
  Eigen::Quaterniond qa(a.block<3, 3>(0, 0).cast<double>()), qb(b.block<3, 3>(0, 0).cast<double>());
  qa.normalize(); qb.normalize();
  return 2 * std::acos(std::min(1.0, std::abs(qa.dot(qb)))) * 180 / kPi;
}
bool separated(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b, const CoupledNdtConfig& config) {
  return translationDistance(a, b) > config.separation_m || rotationDistanceDeg(a, b) > config.separation_deg;
}
double meritAt(const Eigen::Matrix4f& pose, const Eigen::Matrix4f& prediction,
               double energy, double scale, const CoupledNdtConfig& config) {
  const double dt = translationDistance(pose, prediction) / config.motion_translation_scale_m;
  const double dr = rotationDistanceDeg(pose, prediction) / config.motion_rotation_scale_deg;
  return energy / scale + config.motion_weight * (dt * dt + dr * dr);
}
std::vector<std::size_t> previewRanking(const std::vector<CoupledCandidate>& candidates) {
  std::vector<std::size_t> indices(candidates.size());
  std::iota(indices.begin(), indices.end(), 0);
  std::stable_sort(indices.begin(), indices.end(), [&](std::size_t a, std::size_t b) {
    if (candidates[a].finite != candidates[b].finite) return candidates[a].finite;
    if (candidates[a].finite && candidates[a].merit != candidates[b].merit)
      return candidates[a].merit < candidates[b].merit;
    return candidates[a].id < candidates[b].id;
  });
  return indices;
}
std::vector<std::size_t> diverseQualityPreviews(const CoupledShadowResult& result,
                                              const CoupledNdtConfig& config) {
  std::vector<std::size_t> diverse;
  const double limit = result.nominal_energy + config.near_quality_fraction * std::max(1.0, std::abs(result.nominal_energy));
  for (const auto index : previewRanking(result.candidates)) {
    const auto& candidate = result.candidates[index];
    if (!candidate.finite || candidate.energy > limit || !separated(candidate.pose, result.nominal_pose, config)) continue;
    bool distinct = true;
    for (const auto other : diverse)
      if (!separated(candidate.pose, result.candidates[other].pose, config)) { distinct = false; break; }
    if (distinct) diverse.push_back(index);
  }
  return diverse;
}
bool validConfig(const CoupledNdtConfig& c) {
  const std::array<double, 12> values{{c.translation_scale_m, c.strong_step_cap, c.weak_translation_bound_m,
      c.weak_rotation_bound_deg, c.separation_m, c.separation_deg, c.motion_translation_scale_m,
      c.motion_rotation_scale_deg, c.motion_weight, c.near_quality_fraction, c.raw_score_tolerance, 1.0}};
  for (double value : values) if (!std::isfinite(value) || value < 0) return false;
  return c.translation_scale_m > 0 && c.weak_translation_bound_m > 0 && c.weak_rotation_bound_deg > 0 &&
      c.motion_translation_scale_m > 0 && c.motion_rotation_scale_deg > 0 &&
      c.initial_candidates > 0 && c.initial_candidates <= 8 && c.maximum_candidates >= c.initial_candidates &&
      c.maximum_candidates <= 16 && c.maximum_extra_aligns >= 0 && c.maximum_extra_aligns <= 2;
}
}  // namespace

CoupledShadowResult runCoupledNdtShadow(const Eigen::Matrix4f& nominal,
    const Eigen::Matrix4f& prediction, std::size_t source_count,
    const CoupledNdtBackend& backend, const CoupledNdtConfig& config) {
  const auto started = Clock::now(); const auto cpu_started = std::clock();
  CoupledShadowResult result;
  result.nominal_pose = result.recommended_pose = nominal; result.prediction_pose = prediction;
  auto finish = [&](const std::string& status) {
    result.status = status; result.total_ms = elapsedMs(started);
    result.cpu_ms = 1000.0 * (std::clock() - cpu_started) / CLOCKS_PER_SEC;
    return result;
  };
  if (!nominal.allFinite() || !prediction.allFinite() || source_count == 0 ||
      !backend.jet || !backend.score || !backend.refine || !validConfig(config)) return finish("INVALID_INPUT");
  const double count = static_cast<double>(source_count);
  auto evaluateJet = [&](const Eigen::Matrix4f& pose, const Pullback& pull) {
    JointJet joint;
    if (!pull.valid) return joint;
    const auto begin = Clock::now(); ++result.jet_calls;
    try { joint = pullNativeJet(backend.jet(pose, pull.parameters), pull, count); } catch (...) {}
    result.jet_ms += elapsedMs(begin); return joint;
  };
  const auto model_started = Clock::now();
  const Pullback nominal_pull = nominalPullback(nominal, config.translation_scale_m);
  const JointJet nominal_jet = evaluateJet(nominal, nominal_pull);
  if (!nominal_jet.valid) { result.model_ms = elapsedMs(model_started) - result.jet_ms; return finish("NOMINAL_JET_INVALID"); }
  result.nominal_energy = -nominal_jet.score / count;
  const double energy_scale = std::max(1.0, std::abs(result.nominal_energy));
  result.nominal_merit = meritAt(nominal, prediction, result.nominal_energy, energy_scale, config);
  Eigen::SelfAdjointEigenSolver<CoupledMatrix6> eig(nominal_jet.H);
  if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite()) {
    result.model_ms = elapsedMs(model_started) - result.jet_ms; return finish("NOMINAL_EIGEN_INVALID");
  }
  result.eigenvalues = eig.eigenvalues(); result.eigenvectors = eig.eigenvectors();
  if (result.eigenvalues.minCoeff() <= 0) {
    result.model_ms = elapsedMs(model_started) - result.jet_ms; return finish("NOMINAL_NONCONVEX");
  }
  result.weak_dimension = result.eigenvalues(1) / result.eigenvalues(0) >= 2.0 ? 1 : 2;
  const int k = result.weak_dimension, n = 6 - k;
  const Eigen::MatrixXd W = result.eigenvectors.leftCols(k), S = result.eigenvectors.rightCols(n);
  const WeakGrid grid = weakGrid(result.eigenvectors, k, config);
  const auto proposals = coveringProposals(grid, config.maximum_candidates);
  result.model_ms = elapsedMs(model_started) - result.jet_ms;
  Eigen::VectorXd previous_u = Eigen::VectorXd::Zero(k), previous_v = Eigen::VectorXd::Zero(n);
  auto at = [&](const Eigen::VectorXd& u, const Eigen::VectorXd& v) {
    const CoupledVector6 eta = W * u + S * v;
    return poseAtEta(nominal, eta, config.translation_scale_m);
  };
  auto runStage = [&](int begin, int end, int stage) {
    std::vector<GridNode> nodes(proposals.begin() + begin, proposals.begin() + end);
    for (const auto& node : stageVisitOrder(nodes, previous_u)) {
      CoupledCandidate candidate;
      candidate.id = node.id; candidate.stage = stage; candidate.u = node.u;
      candidate.previous_u = previous_u; candidate.previous_v = previous_v;
      candidate.coupling_delta = candidate.gradient_delta = Eigen::VectorXd::Zero(n);
      const Eigen::VectorXd warm = config.method == CoupledMethod::WEAK_ONLY ? Eigen::VectorXd::Zero(n).eval() : previous_v;
      candidate.initial_pose = at(node.u, warm); candidate.v_predicted = candidate.v_corrected = warm;
      candidate.status = "WEAK_ONLY";
      if (config.method != CoupledMethod::WEAK_ONLY) {
        const auto begin_jet = Clock::now();
        const double jet_ms_before = result.jet_ms;
        const CoupledVector6 eta = W * previous_u + S * previous_v;
        const Pullback pull = jointPullback(nominal, eta, result.eigenvectors, config.translation_scale_m);
        // Backend jets are taken at the current previous endpoint; nominal
        // eigencurvature is never substituted at a nonzero transport point.
        const JointJet jet = evaluateJet(at(previous_u, previous_v), pull);
        result.model_ms += elapsedMs(begin_jet) - (result.jet_ms - jet_ms_before);
        const auto solve_started = Clock::now();
        candidate.strong_gradient_norm = jet.valid ? jet.gradient.tail(n).norm() : std::numeric_limits<double>::quiet_NaN();
        const Eigen::VectorXd du = node.u - previous_u;
        const StrongStep predictor = strongStep(jet, k, du, false, config.strong_step_cap);
        if (predictor.valid) candidate.v_predicted = warm + predictor.coupling;
        StrongStep chosen = predictor;
        if (config.method == CoupledMethod::R2_RESIDUAL) {
          const StrongStep residual = strongStep(jet, k, du, true, config.strong_step_cap);
          if (residual.valid) { chosen = residual; candidate.status = "R2_OK"; }
          else {
            candidate.fallback = true;
            candidate.status = predictor.valid ? "R2_FALLBACK_R1:" + residual.reason : "R2_FALLBACK_WARM:" + residual.reason;
          }
        } else {
          candidate.fallback = !predictor.valid;
          candidate.status = predictor.valid ? "R1_OK" : "R1_FALLBACK_WARM:" + predictor.reason;
        }
        if (chosen.valid) {
          candidate.coupling_delta = chosen.coupling; candidate.gradient_delta = chosen.gradient;
          candidate.v_corrected = warm + chosen.coupling + chosen.gradient;
          candidate.damping = chosen.damping; candidate.condition = chosen.condition;
          candidate.solve_residual = chosen.residual; candidate.capped = chosen.capped;
        }
        result.solve_ms += elapsedMs(solve_started);
      }
      candidate.predicted_pose = at(node.u, candidate.v_predicted);
      candidate.pose = at(node.u, candidate.v_corrected);
      const auto preview_started = Clock::now(); ++result.preview_score_calls;
      try { candidate.score_sum = backend.score(candidate.pose); } catch (...) {}
      candidate.energy = -candidate.score_sum / count;
      candidate.prediction_translation_m = translationDistance(candidate.pose, prediction);
      candidate.prediction_rotation_deg = rotationDistanceDeg(candidate.pose, prediction);
      candidate.merit = meritAt(candidate.pose, prediction, candidate.energy, energy_scale, config);
      candidate.finite = candidate.pose.allFinite() && std::isfinite(candidate.energy) && std::isfinite(candidate.merit);
      result.preview_ms += elapsedMs(preview_started);
      result.candidates.push_back(candidate);
      previous_u = node.u; previous_v = candidate.v_corrected;
    }
  };
  const int initial = std::min(config.initial_candidates, static_cast<int>(proposals.size()));
  runStage(0, initial, 1);
  if (initial < static_cast<int>(proposals.size()) &&
      (config.fixed_maximum_budget || diverseQualityPreviews(result, config).size() < 2)) {
    result.expanded = true; runStage(initial, proposals.size(), 2);
  }
  const auto ranking = previewRanking(result.candidates);
  for (std::size_t rank = 0; rank < ranking.size(); ++rank) result.candidates[ranking[rank]].rank = rank;
  const auto diverse = diverseQualityPreviews(result, config);
  double best_merit = result.nominal_merit;
  for (std::size_t j = 0; j < diverse.size() && j < static_cast<std::size_t>(config.maximum_extra_aligns); ++j) {
    auto& candidate = result.candidates[diverse[j]]; candidate.selected_for_refinement = true;
    const auto refinement_started = Clock::now(); ++result.complete_ndt_calls;
    try { candidate.refinement = backend.refine(candidate.pose); }
    catch (...) { candidate.refinement.successful = false; candidate.refinement.status = "BACKEND_EXCEPTION"; }
    if (candidate.refinement.pose.allFinite()) {
      ++result.terminal_score_calls;
      try { candidate.refinement.score_sum = backend.score(candidate.refinement.pose); }
      catch (...) { candidate.refinement.score_sum = std::numeric_limits<double>::quiet_NaN(); }
    }
    result.refinement_ms += elapsedMs(refinement_started);
    const double score = candidate.refinement.score_sum;
    const double refined_merit = meritAt(candidate.refinement.pose, prediction, -score / count, energy_scale, config);
    if (candidate.refinement.successful && candidate.refinement.converged && candidate.refinement.pose.allFinite() && std::isfinite(score) &&
        std::isfinite(refined_merit) && score >= nominal_jet.score + config.raw_score_tolerance && refined_merit < best_merit) {
      best_merit = refined_merit; result.recommended_id = candidate.id; result.recommended_pose = candidate.refinement.pose;
    }
  }
  return finish(result.recommended_id >= 0 ? "RECOMMENDED_SHADOW" : "RETAINED_NOMINAL");
}

bool coupledNdtSelfTest() {
  CoupledNdtConfig config; config.fixed_maximum_budget = true;
  const CoupledMatrix6 identity = CoupledMatrix6::Identity();
  const WeakGrid line = weakGrid(identity, 1, config), disk = weakGrid(identity, 2, config);
  const auto line_proposals = coveringProposals(line, 16), disk_proposals = coveringProposals(disk, 16);
  if (line.admissible.size() != 16 || line_proposals.size() != 16 || disk.admissible.size() != 48 || disk_proposals.size() != 16) return false;
  std::set<int> ids;
  for (const auto& node : disk_proposals) if (node.u.norm() < 1e-12 || !ids.insert(node.id).second) return false;
  // Coverage reaches both signs on each original weak axis and has bounded
  // normalized covering radius over every admissible grid point.
  for (int axis = 0; axis < 2; ++axis) {
    double low = 0, high = 0;
    for (const auto& node : disk_proposals) { low = std::min(low, node.u(axis) / disk.bounds(axis)); high = std::max(high, node.u(axis) / disk.bounds(axis)); }
    if (low > -.99 || high < .99) return false;
  }
  for (const auto& node : disk.admissible) {
    double nearest = normalizedDistance(node.u, Eigen::VectorXd::Zero(2), disk.bounds);
    for (const auto& proposal : disk_proposals) nearest = std::min(nearest, normalizedDistance(node.u, proposal.u, disk.bounds));
    if (nearest > .126) return false;
  }
  JointJet joint; joint.valid = true; joint.H = identity;
  joint.gradient(2) = .03;
  const Eigen::Vector2d du(.02, -.03);
  const auto diagonal_predictor = strongStep(joint, 2, du, false, .10);
  const auto diagonal_residual = strongStep(joint, 2, du, true, .10);
  if (!diagonal_predictor.valid || !diagonal_residual.valid || diagonal_predictor.coupling.norm() > 1e-12 ||
      diagonal_residual.gradient.norm() < .029 || diagonal_residual.coupling.norm() > 1e-12) return false;
  joint.H(0, 2) = joint.H(2, 0) = .2; joint.gradient(2) = .3;
  const auto predictor = strongStep(joint, 2, du, false, .10);
  const auto residual = strongStep(joint, 2, du, true, .10);
  if (!predictor.valid || !residual.valid || !residual.capped || predictor.gradient.norm() != 0 ||
      (predictor.coupling + joint.H.bottomLeftCorner(4, 2) * du).norm() > 1e-12 ||
      std::abs((residual.coupling + residual.gradient).norm() - .10) > 1e-12) return false;
  joint.H(2, 2) = -1;
  if (strongStep(joint, 2, du, false, .10).valid || !strongStep(joint, 2, du, true, .10).valid) return false;

  // Frozen nonidentity carrier, nonzero global eta, full mixed K and g*K.
  Eigen::Matrix4f base = Eigen::Matrix4f::Identity();
  base.block<3, 3>(0, 0) = expRotation(Eigen::Vector3d(.3, -.2, .4)).cast<float>();
  CoupledVector6 eta; eta << .1, -.2, .3, .16, -.12, .09;
  const Pullback pull = jointPullback(base, eta, identity, .8);
  if (!pull.valid) return false;
  CoupledNativeJet native; native.valid = true; native.score_sum = 10;
  native.score_gradient << -.2, .3, -.1, -.4, .2, -.3; native.score_hessian = -identity;
  const JointJet transformed = pullNativeJet(native, pull, 1);
  CoupledVector6 direction; direction << .2, -.1, .3, .4, .2, -.3;
  auto value = [&](const CoupledVector6& x) {
    CoupledVector6 p;
    if (!parametersAtEta(base, pull.parameters.tail<3>(), x, .8, p)) return std::numeric_limits<double>::quiet_NaN();
    const CoupledVector6 z = p - pull.parameters;
    return -native.score_gradient.dot(z) + .5 * z.dot(z);
  };
  constexpr double h = .001;
  if (std::abs((value(eta + h * direction) - value(eta - h * direction)) / (2 * h) - direction.dot(transformed.gradient)) > 1e-6 ||
      std::abs((value(eta + h * direction) + value(eta - h * direction) - 2 * value(eta)) / (h * h) - direction.dot(transformed.H * direction)) > 1e-5 ||
      (transformed.H - pull.J.transpose() * pull.J).norm() < 1e-3) return false;

  int jets = 0, values = 0, aligns = 0, nonzero_jets = 0;
  bool refinements_after_previews = true;
  CoupledNdtBackend backend;
  backend.jet = [&](const Eigen::Matrix4f& pose, const CoupledVector6& p) {
    ++jets; if (translationDistance(pose, base) > 1e-5 || rotationDistanceDeg(pose, base) > 1e-3) ++nonzero_jets;
    CoupledNativeJet j; j.valid = true; j.score_sum = 100;
    // Translation eigenvalues .1, .15, 2 and well-separated rotation modes.
    j.score_hessian.diagonal() << -.1 / .64, -.15 / .64, -2 / .64, -4, -6, -8;
    j.score_gradient(2) = -.03;
    // A changing off-diagonal jet exposes coupling at nonzero transport.
    const double tx = p(0) - base(0, 3);
    j.score_hessian(0, 2) = j.score_hessian(2, 0) = -.01 * tx;
    return j;
  };
  backend.score = [&](const Eigen::Matrix4f& pose) { ++values; return 100 + .01 * translationDistance(pose, base); };
  backend.refine = [&](const Eigen::Matrix4f& pose) {
    refinements_after_previews = refinements_after_previews && values == 16 + aligns;
    ++aligns; CoupledRefinement refinement; refinement.pose = pose; refinement.successful = true;
    refinement.converged = true; refinement.iterations = 1; refinement.status = "SUCCESS"; return refinement;
  };
  const auto result = runCoupledNdtShadow(base, base, 1, backend, config);
  if (result.weak_dimension != 2 || result.candidates.size() != 16 || !result.expanded ||
      result.complete_ndt_calls > 3 || result.complete_ndt_calls != 1 + aligns ||
      result.jet_calls != jets || result.jet_calls != 17 || nonzero_jets < 14 ||
      result.preview_score_calls != 16 || values != result.preview_score_calls + result.terminal_score_calls ||
      result.terminal_score_calls != aligns || !refinements_after_previews) return false;
  bool coupling_seen = false, gradient_seen = false;
  std::vector<int> proposal_ids;
  for (const auto& candidate : result.candidates) {
    proposal_ids.push_back(candidate.id);
    coupling_seen = coupling_seen || candidate.coupling_delta.norm() > 1e-8;
    gradient_seen = gradient_seen || candidate.gradient_delta.norm() > 1e-5;
    if (candidate.u.norm() < 1e-12 || (candidate.v_corrected - candidate.previous_v).norm() > .10000001) return false;
  }
  if (!coupling_seen || !gradient_seen) return false;
  const auto original_refine = backend.refine;
  backend.refine = [](const Eigen::Matrix4f& pose) {
    CoupledRefinement refinement; refinement.pose = pose; refinement.pose(0, 3) += 10;
    refinement.successful = refinement.converged = true; refinement.status = "SUCCESS"; return refinement;
  };
  const auto changed_terminals = runCoupledNdtShadow(base, base, 1, backend, config);
  if (changed_terminals.candidates.size() != result.candidates.size()) return false;
  for (std::size_t i = 0; i < result.candidates.size(); ++i)
    if ((changed_terminals.candidates[i].v_corrected - result.candidates[i].v_corrected).norm() > 1e-12) return false;
  backend.refine = original_refine;
  for (const auto method : {CoupledMethod::WEAK_ONLY, CoupledMethod::R1_PREDICTOR}) {
    config.method = method;
    const auto other = runCoupledNdtShadow(base, base, 1, backend, config);
    if (other.candidates.size() != proposal_ids.size()) return false;
    for (std::size_t i = 0; i < proposal_ids.size(); ++i) if (other.candidates[i].id != proposal_ids[i]) return false;
  }
  config.method = CoupledMethod::WEAK_ONLY; config.fixed_maximum_budget = false;
  const auto adaptive = runCoupledNdtShadow(base, base, 1, backend, config);
  if (adaptive.candidates.size() != 8 || adaptive.expanded || adaptive.preview_score_calls != 8 || adaptive.jet_calls != 1) return false;
  config.fixed_maximum_budget = true;
  // Recommendation requires a successful terminal, the raw-score margin,
  // and lower final merit. Even a high raw score cannot bypass either gate.
  Eigen::Matrix4f target = base; target(0, 3) += 1;
  backend.refine = [&](const Eigen::Matrix4f&) {
    CoupledRefinement refinement; refinement.pose = target;
    refinement.successful = refinement.converged = true; refinement.status = "SUCCESS";
    return refinement;
  };
  double terminal_gain = 1;
  backend.score = [&](const Eigen::Matrix4f& pose) {
    return (pose - target).norm() < 1e-7 ? 100 + terminal_gain : 100;
  };
  const auto recommended = runCoupledNdtShadow(base, target, 1, backend, config);
  if (recommended.recommended_id < 0 || (recommended.recommended_pose - target).norm() != 0 || recommended.complete_ndt_calls != 3) return false;
  terminal_gain = config.raw_score_tolerance / 2;
  const auto raw_gate = runCoupledNdtShadow(base, target, 1, backend, config);
  if (raw_gate.recommended_id != -1 || (raw_gate.recommended_pose - base).norm() != 0) return false;
  terminal_gain = .001;
  const auto merit_gate = runCoupledNdtShadow(base, base, 1, backend, config);
  if (merit_gate.recommended_id != -1 || (merit_gate.recommended_pose - base).norm() != 0) return false;
  backend.refine = [&](const Eigen::Matrix4f&) {
    CoupledRefinement refinement; refinement.pose = target; refinement.successful = false;
    refinement.status = "ITERATION_LIMIT"; return refinement;
  };
  terminal_gain = 10;
  const auto success_gate = runCoupledNdtShadow(base, target, 1, backend, config);
  if (success_gate.recommended_id != -1 || (success_gate.recommended_pose - base).norm() != 0) return false;
  // Invalid transport jets fall back to the exact warm endpoint; invalid
  // nominal jets and nonconvex nominal curvature preserve the input nominal.
  config.method = CoupledMethod::R2_RESIDUAL;
  const auto valid_jet = backend.jet;
  backend.jet = [&](const Eigen::Matrix4f& pose, const CoupledVector6& p) {
    auto j = valid_jet(pose, p);
    if (translationDistance(pose, base) > 1e-5) j.score_gradient(2) = 1e200;
    return j;
  };
  const auto r1_fallback = runCoupledNdtShadow(base, base, 1, backend, config);
  bool r1_fallback_seen = false;
  for (const auto& candidate : r1_fallback.candidates)
    r1_fallback_seen = r1_fallback_seen || candidate.status.find("R2_FALLBACK_R1:") == 0;
  if (!r1_fallback_seen) return false;
  int called = 0;
  backend.jet = [&](const Eigen::Matrix4f& pose, const CoupledVector6& p) {
    auto j = valid_jet(pose, p); if (++called > 1) j.valid = false; return j;
  };
  const auto fallback = runCoupledNdtShadow(base, base, 1, backend, config);
  for (const auto& candidate : fallback.candidates)
    if (!candidate.fallback || (candidate.v_corrected - candidate.previous_v).norm() > 1e-12) return false;
  backend.jet = [](const Eigen::Matrix4f&, const CoupledVector6&) { return CoupledNativeJet{}; };
  const auto invalid = runCoupledNdtShadow(base, base, 1, backend, config);
  if (invalid.status != "NOMINAL_JET_INVALID" || invalid.recommended_id != -1 || (invalid.recommended_pose - base).norm() != 0) return false;
  backend.jet = [&](const Eigen::Matrix4f&, const CoupledVector6&) {
    CoupledNativeJet j; j.valid = true; j.score_sum = 100; j.score_hessian = identity; return j;
  };
  const auto nonconvex = runCoupledNdtShadow(base, base, 1, backend, config);
  return nonconvex.status == "NOMINAL_NONCONVEX" && nonconvex.candidates.empty() && nonconvex.recommended_id == -1 &&
      (nonconvex.recommended_pose - base).norm() == 0;
}
}  // namespace dog_prior_map_fastlio2_frontend_exp
