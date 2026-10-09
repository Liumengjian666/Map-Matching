#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_shadow.hpp"

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>

namespace dog_prior_map_fastlio2_frontend_exp {
namespace {
bool rigid(const Eigen::Matrix4f& pose) {
  if (!pose.allFinite()) return false;
  const Eigen::Matrix3d R = pose.block<3, 3>(0, 0).cast<double>();
  return (pose.row(3) - Eigen::RowVector4f(0, 0, 0, 1)).norm() < 1e-6 &&
      (R.transpose() * R - Eigen::Matrix3d::Identity()).norm() < 1e-4 &&
      std::abs(R.determinant() - 1) < 1e-4;
}
double dt(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
  return (a.block<3, 1>(0, 3).cast<double>() - b.block<3, 1>(0, 3).cast<double>()).norm();
}
double dr(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
  const Eigen::Quaterniond qa(a.block<3, 3>(0, 0).cast<double>()), qb(b.block<3, 3>(0, 0).cast<double>());
  return qa.normalized().angularDistance(qb.normalized()) * 180 / std::acos(-1.0);
}
bool nonlocal(const Eigen::Matrix4f& pose, const Eigen::Matrix4f& nominal,
              const CoupledNdtConfig& config) {
  return dt(pose, nominal) > config.separation_m || dr(pose, nominal) > config.separation_deg;
}
bool physical(const Eigen::Matrix4f& pose, const Eigen::Matrix4f& nominal,
              const CoupledNdtConfig& config) {
  return dt(pose, nominal) <= config.weak_translation_bound_m &&
      dr(pose, nominal) <= config.weak_rotation_bound_deg;
}
double merit(const Eigen::Matrix4f& pose, const Eigen::Matrix4f& prediction,
             double energy, double nominal_energy, const CoupledNdtConfig& c) {
  return energy / std::max(1.0, std::abs(nominal_energy)) + c.motion_weight *
      (std::pow(dt(pose, prediction) / c.motion_translation_scale_m, 2) +
       std::pow(dr(pose, prediction) / c.motion_rotation_scale_deg, 2));
}
bool quality(const CoupledRefinement& terminal, double nominal_energy,
             double count, const CoupledNdtConfig& config) {
  return terminal.successful && terminal.converged && rigid(terminal.pose) &&
      std::isfinite(terminal.score_sum) && terminal.score_sum > 0 &&
      -terminal.score_sum / count <= nominal_energy +
          config.near_quality_fraction * std::max(1.0, std::abs(nominal_energy));
}
bool validConfig(const CoupledEventConfig& c) {
  const double positive[] = {c.search.translation_scale_m, c.search.strong_step_cap,
      c.search.weak_translation_bound_m, c.search.weak_rotation_bound_deg,
      c.search.separation_m, c.search.separation_deg, c.search.motion_translation_scale_m,
      c.search.motion_rotation_scale_deg, c.search.near_quality_fraction};
  for (double value : positive) if (!std::isfinite(value) || value <= 0) return false;
  if (!std::isfinite(c.search.motion_weight) || c.search.motion_weight < 0 ||
      !std::isfinite(c.search.raw_score_tolerance) || c.search.raw_score_tolerance < 0 ||
      c.search.initial_candidates < 1 || c.search.initial_candidates > 8 ||
      c.search.maximum_candidates < c.search.initial_candidates || c.search.maximum_candidates > 16 ||
      c.search.maximum_extra_aligns < 0) return false;
  return std::isfinite(c.trigger_translation_m) && c.trigger_translation_m > 0 &&
      std::isfinite(c.trigger_rotation_deg) && c.trigger_rotation_deg > 0 &&
      std::isfinite(c.confirmation_translation_m) && c.confirmation_translation_m > 0 &&
      std::isfinite(c.confirmation_rotation_deg) && c.confirmation_rotation_deg > 0 &&
      std::isfinite(c.maximum_prediction_gap_s) && c.maximum_prediction_gap_s > 0 &&
      std::isfinite(c.local_rotation_tolerance_deg) && c.local_rotation_tolerance_deg >= 0 &&
      c.required_confirmations == 2 && c.search.maximum_extra_aligns <= 2;
}
}  // namespace

CoupledEventResult runEventCoupledNdtShadow(const Eigen::Matrix4f& nominal,
    const Eigen::Matrix4f& prediction, uint64_t stamp_ns, std::size_t source_count,
    bool nominal_effective, const CoupledNdtBackend& backend,
    const CoupledEventConfig& config, PendingCandidate* pending) {
  const auto start = std::chrono::steady_clock::now();
  const auto cpu = std::clock();
  CoupledEventResult out;
  out.shadow.nominal_pose = nominal; out.shadow.prediction_pose = prediction;
  out.shadow.recommended_pose = nominal.allFinite() ? nominal : Eigen::Matrix4f::Identity();
  out.pending_before = pending && pending->active;
  auto finish = [&](const std::string& event) {
    out.event = event; out.shadow.status = event;
    out.pending_after = pending && pending->active;
    out.shadow.total_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    out.shadow.cpu_ms = 1000.0 * (std::clock() - cpu) / CLOCKS_PER_SEC;
    return out;
  };
  if (!pending || !validConfig(config)) {
    if (pending) *pending = PendingCandidate{};
    return finish("INVALID_EVENT_CONFIG");
  }
  if (!nominal_effective || !rigid(nominal) || !rigid(prediction) || !stamp_ns || !source_count) {
    *pending = PendingCandidate{};
    return finish(out.pending_before ? "NOMINAL_INVALID_PENDING_CLEARED" : "NOMINAL_INVALID");
  }
  out.recommendation_available = true;
  out.innovation_translation_m = dt(nominal, prediction);
  out.innovation_rotation_deg = dr(nominal, prediction);
  out.innovation_trigger = out.innovation_translation_m > config.trigger_translation_m ||
      out.innovation_rotation_deg > config.trigger_rotation_deg;
  const double count = static_cast<double>(source_count);
  if (pending->active) {
    out.mode = "PENDING";
    if (!rigid(pending->pose) || !rigid(pending->imu_prediction) || !pending->stamp_ns || pending->confirmations < 0 ||
        pending->confirmations >= config.required_confirmations || stamp_ns <= pending->stamp_ns ||
        (stamp_ns - pending->stamp_ns) / 1e9 > config.maximum_prediction_gap_s) {
      *pending = PendingCandidate{};
      return finish("PENDING_EXPIRED_OR_INVALID");
    }
    out.propagated_alternative = (pending->pose.cast<double>() *
        pending->imu_prediction.cast<double>().inverse() * prediction.cast<double>()).cast<float>();
    if (!rigid(out.propagated_alternative) || !backend.refine || !backend.score) {
      *pending = PendingCandidate{};
      return finish("PENDING_INVALID_PROPAGATION");
    }
    const auto align_start = std::chrono::steady_clock::now();
    ++out.shadow.complete_ndt_calls;
    try { out.temporal_terminal = backend.refine(out.propagated_alternative); }
    catch (...) { out.temporal_terminal.status = "BACKEND_EXCEPTION"; }
    double nominal_score = std::numeric_limits<double>::quiet_NaN();
    ++out.shadow.terminal_score_calls;
    try { nominal_score = backend.score(nominal); } catch (...) {}
    if (out.temporal_terminal.pose.allFinite()) {
      ++out.shadow.terminal_score_calls;
      out.temporal_terminal.score_sum = std::numeric_limits<double>::quiet_NaN();
      try { out.temporal_terminal.score_sum = backend.score(out.temporal_terminal.pose); } catch (...) {}
    }
    out.shadow.refinement_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - align_start).count();
    out.shadow.nominal_energy = -nominal_score / count;
    if (!std::isfinite(nominal_score) || !quality(out.temporal_terminal, out.shadow.nominal_energy, count, config.search)) {
      *pending = PendingCandidate{};
      return finish("PENDING_MATCH_QUALITY_FAILED");
    }
    out.temporal_translation_residual_m = dt(out.temporal_terminal.pose, out.propagated_alternative);
    out.temporal_rotation_residual_deg = dr(out.temporal_terminal.pose, out.propagated_alternative);
    if (!nonlocal(out.temporal_terminal.pose, nominal, config.search)) {
      *pending = PendingCandidate{};
      return finish("PENDING_COLLAPSED_TO_NOMINAL");
    }
    if (out.temporal_translation_residual_m > config.confirmation_translation_m ||
        out.temporal_rotation_residual_deg > config.confirmation_rotation_deg ||
        !physical(out.temporal_terminal.pose, nominal, config.search)) {
      *pending = PendingCandidate{};
      return finish("PENDING_CONTINUITY_FAILED");
    }
    out.confirmation_count = pending->confirmations + 1;
    if (out.confirmation_count == config.required_confirmations) {
      *pending = PendingCandidate{};
      return finish("TEMPORALLY_SUPPORTED");
    }
    pending->pose = out.temporal_terminal.pose; pending->imu_prediction = prediction;
    pending->stamp_ns = stamp_ns; pending->confirmations = out.confirmation_count;
    return finish("PENDING_FIRST_SUPPORT");
  }
  out.mode = "NORMAL";
  if (!out.innovation_trigger) return finish("UNTRIGGERED_NOMINAL");
  out.mode = "SEARCH";
  out.shadow = runCoupledNdtShadow(nominal, prediction, source_count, backend, config.search);
  // R2's recommendation may be nonlocal; R3 never recommends it directly.
  out.shadow.recommended_pose = nominal; out.shadow.recommended_id = -1;
  double best_local = out.shadow.nominal_merit;
  double best_pending = std::numeric_limits<double>::infinity();
  for (const auto& c : out.shadow.candidates) {
    if (!c.selected_for_refinement) continue;
    const auto& terminal = c.refinement;
    if (!rigid(terminal.pose)) continue;
    const bool separated = nonlocal(terminal.pose, nominal, config.search);
    if (separated) ++out.nonlocal_terminals;
    if (!quality(terminal, out.shadow.nominal_energy, count, config.search)) continue;
    const double final_merit = merit(terminal.pose, prediction, -terminal.score_sum / count,
        out.shadow.nominal_energy, config.search);
    if (!std::isfinite(final_merit)) continue;
    if (!separated) {
      if (terminal.score_sum < -out.shadow.nominal_energy * count + config.search.raw_score_tolerance ||
          dr(terminal.pose, prediction) > out.innovation_rotation_deg + config.local_rotation_tolerance_deg ||
          final_merit >= out.shadow.nominal_merit) continue;
      if (final_merit < best_local ||
          (final_merit == best_local && c.id < out.shadow.recommended_id)) {
        best_local = final_merit; out.shadow.recommended_id = c.id;
        out.shadow.recommended_pose = terminal.pose;
      }
    } else if (physical(terminal.pose, nominal, config.search)) {
      ++out.eligible_nonlocal_terminals;
      if (final_merit < best_pending || (final_merit == best_pending && c.id < out.pending_candidate_id)) {
        best_pending = final_merit; out.pending_candidate_id = c.id;
        pending->pose = terminal.pose; pending->imu_prediction = prediction;
        pending->stamp_ns = stamp_ns; pending->confirmations = 0; pending->active = true;
      }
    }
  }
  return finish(pending->active ? "PENDING_CREATED" : out.shadow.recommended_id >= 0 ?
      "LOCAL_RECOMMENDED" : "SEARCH_RETAINED_NOMINAL");
}
}  // namespace dog_prior_map_fastlio2_frontend_exp
