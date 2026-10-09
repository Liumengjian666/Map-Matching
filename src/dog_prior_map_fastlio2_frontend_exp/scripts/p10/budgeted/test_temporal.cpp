#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_shadow.hpp"
#include <Eigen/Geometry>
#include <iostream>
#include <stdexcept>

namespace p = dog_prior_map_fastlio2_frontend_exp;
namespace {
void require(bool value, const char* name) { if (!value) throw std::runtime_error(name); }
Eigen::Matrix4f pose(float x) { auto T = Eigen::Matrix4f::Identity().eval(); T(0, 3) = x; return T; }
}
int main() {
  try {
    p::CoupledEventConfig config;
    p::PendingCandidate pending;
    const auto nominal = pose(0), prediction = pose(.2f);
    int jets = 0, scores = 0, aligns = 0;
    auto terminal_pose = pose(.5f);
    double gain = -1;
    bool use_initial = false, success = true, throw_terminal_score = false;
    p::CoupledNdtBackend backend;
    backend.jet = [&](const Eigen::Matrix4f&, const p::CoupledVector6&) {
      ++jets; p::CoupledNativeJet j; j.valid = true; j.score_sum = 100;
      j.score_hessian.diagonal() << -.1, -2, -3, -4, -5, -6; return j;
    };
    backend.score = [&](const Eigen::Matrix4f& T) {
      ++scores;
      if (throw_terminal_score && T(0, 3) != 0) throw std::runtime_error("score_failure");
      return T(0, 3) == 0 ? 100.0 : 100 + gain;
    };
    backend.refine = [&](const Eigen::Matrix4f& initial) {
      ++aligns; p::CoupledRefinement r; r.pose = use_initial ? initial : terminal_pose;
      r.score_sum = 100; // A stale refine score must not survive an explicit score exception.
      r.successful = r.converged = success; r.iterations = 5; r.status = success ? "SUCCESS" : "NOT_CONVERGED"; return r;
    };
    auto run = [&](const Eigen::Matrix4f& n, const Eigen::Matrix4f& pred, uint64_t stamp, bool valid = true) {
      return p::runEventCoupledNdtShadow(n, pred, stamp, 1, valid, backend, config, &pending);
    };
    auto result = run(nominal, pose(.11f), 1000000000);
    require(result.mode == "NORMAL" && !result.innovation_trigger && jets == 0 && scores == 0 && aligns == 0 &&
        result.shadow.recommended_pose == nominal, "untriggered zero backend work");
    result = run(nominal, prediction, 1100000000);
    require(result.event == "PENDING_CREATED" && pending.active && result.shadow.recommended_id == -1 &&
        result.eligible_nonlocal_terminals > 0 && result.shadow.complete_ndt_calls <= 3,
        "worse-score nonlocal retained diagnostically");
    use_initial = true; jets = scores = aligns = 0;
    result = run(pose(.1f), pose(.3f), 1200000000);
    require(result.event == "PENDING_FIRST_SUPPORT" && result.confirmation_count == 1 &&
        result.mode == "PENDING" && jets == 0 && aligns == 1 && scores == 2 &&
        result.shadow.preview_score_calls == 0 && std::abs(pending.pose(0, 3) - .6f) < 1e-6,
        "first future frame confirms with one align no search");
    result = run(pose(.2f), pose(.4f), 1300000000);
    require(result.event == "TEMPORALLY_SUPPORTED" && result.confirmation_count == 2 && !pending.active &&
        result.shadow.recommended_id == -1 && result.shadow.recommended_pose == pose(.2f),
        "two future frames supported but never recommended");
    pending.active = true; pending.pose = pose(.5f); pending.imu_prediction = pose(.2f); pending.stamp_ns = 1300000000;
    jets = scores = aligns = 0;
    result = run(nominal, prediction, 1700000000);
    require(result.event == "PENDING_EXPIRED_OR_INVALID" && !pending.active && aligns == 0 && jets == 0,
        "expiry cannot reenter triggered search");
    pending.active = true; pending.pose = pose(.5f); pending.imu_prediction = prediction; pending.stamp_ns = 1700000000;
    auto malformed = prediction; malformed(0, 0) = 2;
    result = run(nominal, malformed, 1800000000);
    require(result.mode == "INVALID" && !pending.active && aligns == 0 && jets == 0,
        "invalid rigid prediction rejected before inverse/backend");
    pending.active = true; pending.pose = pose(.5f); pending.imu_prediction = prediction; pending.stamp_ns = 1800000000;
    gain = std::numeric_limits<double>::infinity();
    result = run(nominal, prediction, 1900000000);
    require(result.event == "PENDING_MATCH_QUALITY_FAILED" && !pending.active && jets == 0,
        "nonfinite score cannot support pending");
    pending.active = true; pending.pose = pose(.5f); pending.imu_prediction = prediction; pending.stamp_ns = 1900000000;
    throw_terminal_score = true; gain = 0;
    result = run(nominal, prediction, 1950000000);
    require(result.event == "PENDING_MATCH_QUALITY_FAILED" && !pending.active,
        "score exception cannot reuse stale refine score");
    throw_terminal_score = false;
    gain = 1; use_initial = false; terminal_pose = pose(.1f);
    result = run(nominal, prediction, 2000000000);
    require(result.event == "LOCAL_RECOMMENDED" && !pending.active && result.shadow.recommended_id >= 0,
        "local objective and prediction admission");
    terminal_pose.block<3, 3>(0, 0) = Eigen::AngleAxisf(.01f, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    result = run(nominal, prediction, 2100000000);
    require(result.shadow.recommended_id == -1, "local rotation disagreement must not increase");
    // Noncommuting frames: pred_next=pred_prev*D implies alt_next=alt_prev*D.
    pending = p::PendingCandidate{}; pending.active = true; pending.stamp_ns = 2200000000;
    pending.pose = pose(.5f); pending.imu_prediction = pose(.1f);
    pending.pose.block<3, 3>(0, 0) = Eigen::AngleAxisf(.1f, Eigen::Vector3f::UnitY()).toRotationMatrix();
    pending.imu_prediction.block<3, 3>(0, 0) = Eigen::AngleAxisf(.05f, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    auto D = pose(.02f); D.block<3, 3>(0, 0) = Eigen::AngleAxisf(.04f, Eigen::Vector3f::UnitX()).toRotationMatrix();
    const Eigen::Matrix4f expected = pending.pose * D, next_pred = pending.imu_prediction * D;
    use_initial = true; gain = 0;
    result = run(next_pred, next_pred, 2300000000);
    require((result.propagated_alternative - expected).norm() < 1e-6 && result.event == "PENDING_FIRST_SUPPORT",
        "right-increment propagation with noncommuting rotations");
    terminal_pose = next_pred; use_initial = false;
    result = run(next_pred, next_pred, 2400000000);
    require(!pending.active && result.event == "PENDING_COLLAPSED_TO_NOMINAL",
        "nominal collapse is not alternate support");
    result = run(nominal, prediction, 2500000000, false);
    require(result.mode == "INVALID" && !result.recommendation_available && result.shadow.jet_calls == 0,
        "invalid nominal is not a valid recommendation");
    pending.active = true; pending.pose = pose(.5f); pending.imu_prediction = prediction;
    pending.stamp_ns = 0; aligns = scores = jets = 0;
    result = run(nominal, prediction, 100000000);
    require(result.event == "PENDING_EXPIRED_OR_INVALID" && !pending.active && aligns == 0 && scores == 0 && jets == 0,
        "zero pending timestamp cannot call backend");
    std::cout << "temporal scheduling tests PASS\n";
    return 0;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
