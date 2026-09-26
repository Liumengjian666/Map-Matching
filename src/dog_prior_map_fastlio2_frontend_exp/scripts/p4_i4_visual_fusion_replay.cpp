// Offline-only test seam. Pre-include dependencies so the access macro affects
// only the frontend class header. No runtime source/header or ABI is changed.
#include "dog_prior_map_fastlio2_frontend_exp/frontend_types.hpp"
#include <memory>
#include <string>
#include <vector>
#define private public
#include "dog_prior_map_fastlio2_frontend_exp/fastlio2_frontend.hpp"
#undef private
#define main p4_i2_unused_entry_point
#include "p4_i2_state_contamination_replay.cpp"
#undef main

#include <chrono>
#include <map>
#include <set>

namespace p4_i4 {
using namespace p4_i2;
using Cov = Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF>;
struct Visual {
  uint64_t ref = 0, cur = 0;
  Eigen::Vector3d translation;
  int inliers = 0;
  double ratio = 0, reprojection = 0;
};
struct Event { uint64_t stamp; int kind; std::size_t index; };

std::vector<Visual> readVisual(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("visual_input_missing");
  std::string line;
  std::getline(stream, line);
  std::vector<Visual> result;
  uint64_t previous = 0;
  while (std::getline(stream, line)) {
    const auto f = splitCsv(line);
    if (f.size() != 8) throw std::runtime_error("visual_column_count");
    Visual v;
    v.ref = std::stoull(f[0]); v.cur = std::stoull(f[1]);
    v.translation = Eigen::Vector3d(std::stod(f[2]), std::stod(f[3]), std::stod(f[4]));
    v.inliers = std::stoi(f[5]); v.ratio = std::stod(f[6]); v.reprojection = std::stod(f[7]);
    if (v.ref >= v.cur || v.cur <= previous || !v.translation.allFinite())
      throw std::runtime_error("visual_invalid_order_or_payload");
    previous = v.cur; result.push_back(v);
  }
  return result;
}

void predictTo(FastLio2IkfomFrontend& frontend, const ImuVector& all, uint64_t end) {
  const auto stamp = frontend.getState().stamp_ns;
  if (stamp == end) return;
  if (stamp > end) throw std::runtime_error("backwards_event");
  auto after_state = std::upper_bound(all.begin(), all.end(), stamp,
      [](uint64_t t, const ImuSample& s) { return t < s.stamp_ns; });
  auto after_end = std::upper_bound(all.begin(), all.end(), end,
      [](uint64_t t, const ImuSample& s) { return t < s.stamp_ns; });
  if (after_state == all.begin() || after_end - all.begin() < 2)
    throw std::runtime_error("missing_causal_imu_history");
  auto first = after_state - 1;
  if (after_end - first < 2) first = after_end - 2;
  ImuVector samples(first, after_end);
  std::string reason;
  if (samples.back().stamp_ns <= stamp) {
    if (!frontend.predictHeldInputTo(end, samples[samples.size() - 2], samples.back(), &reason))
      throw std::runtime_error("short_causal_interval:" + reason);
    return;
  }
  std::vector<dog_prior_map_fastlio2_frontend_exp::ImuPoseSample,
              Eigen::aligned_allocator<dog_prior_map_fastlio2_frontend_exp::ImuPoseSample>> poses;
  if (!frontend.predictImuSequence(samples, end, &poses, &reason))
    throw std::runtime_error("event_prediction:" + reason);
  if (frontend.getState().stamp_ns != end) throw std::runtime_error("inexact_event_stamp");
}

// Linear position observation using the SAME IKFoM state and covariance. No
// Kalman rows are frozen, and no visual rotation residual is constructed.
template<int M>
void positionUpdate(FastLio2IkfomFrontend& frontend, const Eigen::Vector3d& target,
                    const Eigen::Matrix<double, M, 3>& axes, double sigma) {
  auto& impl = *frontend.impl_;
  state_ikfom prior = impl.filter.get_x();
  state_ikfom state = prior;
  const Cov covariance = impl.filter.get_P();
  Eigen::Matrix<double, M, state_ikfom::DOF> h = Eigen::Matrix<double, M, state_ikfom::DOF>::Zero();
  h.template block<M, 3>(0, MTK::getStartIdx(&state_ikfom::pos)) = axes;
  const Eigen::Matrix<double, M, M> noise = Eigen::Matrix<double, M, M>::Identity() * sigma * sigma;
  const Eigen::Matrix<double, M, M> s = h * covariance * h.transpose() + noise;
  const auto ldlt = s.ldlt();
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive())
    throw std::runtime_error("visual_innovation_covariance_not_positive");
  const Eigen::Matrix<double, state_ikfom::DOF, M> gain =
      covariance * h.transpose() * ldlt.solve(Eigen::Matrix<double, M, M>::Identity());
  const Eigen::Matrix<double, state_ikfom::DOF, 1> dx = gain * (axes * (target - prior.pos));
  state.boxplus(dx);
  const Cov residual_map = Cov::Identity() - gain * h;
  Cov posterior = residual_map * covariance * residual_map.transpose() + gain * noise * gain.transpose();
  Cov reset = Cov::Identity();
  for (const auto& item : state.SO3_state) {
    const int index = item.first;
    const MTK::vect<3, double> delta = dx.template segment<3>(index);
    reset.template block<3, 3>(index, index) = A_matrix(delta).transpose();
  }
  for (const auto& item : state.S2_state) {
    const int index = item.first;
    MTK::vect<2, double> delta; delta << dx[index], dx[index + 1];
    Eigen::Matrix<double, 2, 3> nx;
    Eigen::Matrix<double, 3, 2> mx;
    state.S2_Nx_yy(nx, index); prior.S2_Mx(mx, delta, index);
    reset.template block<2, 2>(index, index) = nx * mx;
  }
  posterior = (reset * posterior * reset.transpose()).eval();
  posterior = (0.5 * (posterior + posterior.transpose())).eval();
  dog_prior_map_fastlio2_frontend_exp::enforceFixedExtrinsicConstraint(
      state, posterior, impl.fixed_rotation, impl.fixed_translation);
  impl.filter.change_x(state); impl.filter.change_P(posterior);
  std::string reason;
  if (!frontend.postconditionsValid(&reason)) throw std::runtime_error("visual_update:" + reason);
}

int run(const std::string& mode, const Inputs& inputs, const RuntimeParameters& params,
        const Pose3d& initial, const Pose3d& extrinsic, const std::vector<Visual>& visual,
        const std::string& trajectory_path, const std::string& updates_path) {
  if (mode == "BASELINE") return runFullUpdate(inputs, params, initial, extrinsic, trajectory_path);
  const bool skip = mode == "VISUAL_SKIP";
  const bool xy = mode == "VISUAL_XY_005";
  const double sigma = mode == "VISUAL_XYZ_003" ? 0.03 : mode == "VISUAL_XYZ_010" ? 0.10 : 0.05;
  if (!skip && !xy && mode != "VISUAL_XYZ_003" && mode != "VISUAL_XYZ_005" && mode != "VISUAL_XYZ_010" && mode != "CONTRACT_ONLY")
    throw std::runtime_error("unsupported_mode");
  FastLio2IkfomFrontend frontend(params);
  if (inputs.imu.size() < static_cast<std::size_t>(params.static_init_samples))
    throw std::runtime_error("insufficient_initial_imu");
  ImuVector initialization(inputs.imu.begin(), inputs.imu.begin() + params.static_init_samples);
  std::string reason;
  if (!frontend.initializeStatic(initialization, initial, extrinsic, &reason))
    throw std::runtime_error(reason);
  // Disposable initialized clones: rotated body-Z must be unobserved in XY;
  // XYZ must match the linear isotropic Kalman solution at the seed state.
  const auto seed = frontend.getState();
  const Eigen::Matrix3d test_rotation = Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
  const Eigen::Matrix<double, 2, 3> test_axes = test_rotation.transpose().topRows<2>();
  auto test = frontend.cloneCandidate();
  positionUpdate<2>(*test, seed.map_T_imu.position + test_rotation.col(2), test_axes, 0.05);
  if ((test->getState().map_T_imu.position - seed.map_T_imu.position).norm() > 1e-10)
    throw std::runtime_error("XY_BODY_Z_NULLSPACE_TEST_FAILED");
  test = frontend.cloneCandidate();
  const Eigen::Vector3d target_delta(0.1, -0.2, 0.3);
  positionUpdate<3>(*test, seed.map_T_imu.position + target_delta, Eigen::Matrix3d::Identity(), 0.05);
  if ((test->getState().map_T_imu.position - seed.map_T_imu.position - target_delta / 1.0025).norm() > 1e-10)
    throw std::runtime_error("XYZ_LINEAR_KALMAN_TEST_FAILED");
  std::cout << "POSITION_UPDATE_CONTRACT_PASS " << mode << '\n';
  if (mode == "CONTRACT_ONLY") return 0;
  std::vector<Event> events;
  for (std::size_t i = 0; i < inputs.scans.size(); ++i) events.push_back({inputs.scans[i].stamp_ns, 0, i});
  std::set<uint64_t> references;
  for (std::size_t i = 0; i < visual.size(); ++i) {
    if (visual[i].ref < frontend.getState().stamp_ns || visual[i].cur > inputs.scans.back().stamp_ns)
      throw std::runtime_error("visual_outside_replay_interval");
    events.push_back({visual[i].cur, 1, i});
    if (references.insert(visual[i].ref).second) events.push_back({visual[i].ref, 2, i});
  }
  // Actual stamps first. Exact ties: NDT, visual, then new reference snapshot.
  std::sort(events.begin(), events.end(), [](const Event& a, const Event& b) {
    return a.stamp != b.stamp ? a.stamp < b.stamp : a.kind < b.kind;
  });
  std::map<uint64_t, Pose3d> anchors;
  std::ofstream trajectory(trajectory_path), updates(updates_path);
  if (!trajectory || !updates) throw std::runtime_error("cannot_open_outputs");
  trajectory << std::setprecision(17); updates << std::setprecision(17);
  writeHeader(trajectory);
  updates << "mode,ref_ns,stamp_ns,measurement_x,measurement_y,measurement_z,innovation_x,innovation_y,innovation_z,innovation_norm,measured_innovation_norm,position_correction,velocity_correction,rotation_correction_deg,sigma,pnp_inliers,inlier_ratio,reprojection_rmse_px,update_ms\n";
  for (const auto& event : events) {
    if (event.kind == 0) {
      const auto& scan = inputs.scans[event.index];
      predictTo(frontend, inputs.imu, event.stamp);
      const auto predicted = frontend.getState();
      PoseCorrectionDelta delta;
      if (!frontend.applyPoseMeasurement(lidarMeasurementToImu(scan.used_lidar, extrinsic), &delta, &reason))
        throw std::runtime_error("ndt_update:" + reason);
      writeRow(trajectory, event.index, scan, predicted.map_T_imu, predicted, frontend.getState(), extrinsic);
      continue;
    }
    // Read-only snapshots and skipped updates must not subdivide committed IMU
    // integration. This preserves the exact original baseline when visual is OFF.
    auto candidate = frontend.cloneCandidate();
    predictTo(*candidate, inputs.imu, event.stamp);
    if (event.kind == 2) { anchors[event.stamp] = candidate->getState().map_T_imu; continue; }
    const auto& v = visual[event.index];
    const auto anchor = anchors.at(v.ref);
    if (skip) continue;
    const Eigen::Vector3d target = anchor.position + anchor.orientation * v.translation;
    const auto before = candidate->getState();
    const Eigen::Vector3d innovation = target - before.map_T_imu.position;
    const Eigen::Matrix<double, 2, 3> axes = anchor.orientation.toRotationMatrix().transpose().topRows<2>();
    const auto tick = std::chrono::steady_clock::now();
    if (xy) positionUpdate<2>(*candidate, target, axes, sigma);
    else positionUpdate<3>(*candidate, target, Eigen::Matrix3d::Identity(), sigma);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tick).count();
    if (!frontend.commitCandidate(*candidate, &reason)) throw std::runtime_error(reason);
    const auto after = frontend.getState();
    updates << mode << ',' << v.ref << ',' << v.cur;
    writeVector(updates, target); writeVector(updates, innovation);
    updates << ',' << innovation.norm() << ',' << (xy ? (axes * innovation).norm() : innovation.norm())
            << ',' << (after.map_T_imu.position - before.map_T_imu.position).norm()
            << ',' << (after.velocity - before.velocity).norm()
            << ',' << Eigen::AngleAxisd(poseDelta(before.map_T_imu, after.map_T_imu)).angle() * 180.0 / M_PI
            << ',' << sigma << ',' << v.inliers << ',' << v.ratio << ',' << v.reprojection << ',' << ms << '\n';
  }
  std::cout << mode << "_COMPLETE scans=" << inputs.scans.size() << " visual=" << (skip ? 0 : visual.size()) << '\n';
  return 0;
}
}  // namespace p4_i4

int main(int argc, char** argv) {
  if (argc != 8) { std::cerr << "MODE imu.csv scans.csv params.txt visual.csv trajectory.csv updates.csv\n"; return 2; }
  try {
    p4_i2::Inputs inputs; std::string reason;
    if (!p4_i2::readInputs(argv[2], argv[3], &inputs, &reason)) throw std::runtime_error(reason);
    p4_i2::Pose3d initial, extrinsic;
    const auto params = p4_i2::readParameters(argv[4], &initial, &extrinsic);
    return p4_i4::run(argv[1], inputs, params, initial, extrinsic, p4_i4::readVisual(argv[5]), argv[6], argv[7]);
  } catch (const std::exception& e) { std::cerr << "P4_I4_FAILED: " << e.what() << '\n'; return 1; }
}
