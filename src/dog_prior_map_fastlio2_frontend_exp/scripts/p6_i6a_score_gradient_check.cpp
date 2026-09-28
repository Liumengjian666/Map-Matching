// One focused PCL 1.10 score/gradient and parameter-coordinate check using a
// single frozen low-jump I5C frame. This calls computeDerivatives only; it
// performs no NDT align and does not repeat the I5C endpoint audit.
#define P6_I5C_NO_MAIN
#include "../scripts/p6_i5c_stationary_audit.cpp"
#undef P6_I5C_NO_MAIN

namespace {
using namespace p6_i5c_app;

void runCheck(const std::string& selected_path, const std::string& manifest_path,
              const std::string& scans_path, const std::string& packed_path,
              const std::string& map_path, const std::string& output_path) {
  requireOmpSingleThread();
  const auto selection_hashes = verifyFrozenSelection(selected_path, manifest_path);
  verifyFrozenFiles(scans_path, packed_path, map_path);
  const std::vector<SelectedCase> selected = readSelected(selected_path);
  const auto item_it = std::find_if(selected.begin(), selected.end(),
      [](const SelectedCase& item) { return item.frame_id == "P2F003"; });
  if (item_it == selected.end()) throw std::runtime_error("focused_case_P2F003_missing");
  const SelectedCase& item = *item_it;
  const auto clouds = loadCloudContexts({item}, scans_path, packed_path);
  const Cloud::Ptr target = ::loadTarget(map_path);
  const auto cloud = clouds.find(item.transaction_id);
  if (cloud == clouds.end()) throw std::runtime_error("focused_source_cloud_missing");

  DerivativeNdt ndt;
  ::configureNdt(ndt, target);
  ndt.setInputSource(cloud->second.source);
  const Eigen::Matrix4f saved_pose = item.inside.saved_terminal.cast<float>();
  const Eigen::Matrix<double, 6, 1> p = p6_i5c::pclVector(saved_pose);
  const Eigen::Matrix4f roundtrip = p6_i5c::poseFromPclVector(p);
  const double roundtrip_t =
      (saved_pose.block<3, 1>(0, 3) - roundtrip.block<3, 1>(0, 3)).norm();
  const double roundtrip_r = p6_i5c::rotationDistanceRad(
      saved_pose.block<3, 3>(0, 0).cast<double>(),
      roundtrip.block<3, 3>(0, 0).cast<double>()) * 180.0 / M_PI;

  Eigen::Matrix<double, 6, 1> analytic_gradient;
  Eigen::Matrix<double, 6, 6> analytic_hessian;
  const double center_score = scoreAtP(ndt, cloud->second.source, p,
                                       &analytic_gradient, &analytic_hessian);
  if (!std::isfinite(center_score) || !analytic_gradient.allFinite() ||
      !analytic_hessian.allFinite())
    throw std::runtime_error("focused_score_gradient_nonfinite");

  const char* axes[] = {"tx", "ty", "tz", "rx", "ry", "rz"};
  std::ofstream output(output_path);
  if (!output) throw std::runtime_error("cannot_create_score_gradient_check_csv");
  output << std::setprecision(17)
      << "frame_id,transaction_id,endpoint,component,pcl_order,pcl_value,score_center,"
         "analytic_score_gradient,fd_gradient_h_1e-4,fd_gradient_h_5e-5,"
         "directional_sign_h,directional_sign_half_h,relative_diff_h,relative_diff_half_h,"
         "pose_roundtrip_translation_m,pose_roundtrip_rotation_deg,align_calls\n";
  int comparable = 0;
  int sign_matches = 0;
  int close_derivatives = 0;
  for (int axis = 0; axis < 6; ++axis) {
    const auto finiteDifference = [&](double step) {
      Eigen::Matrix<double, 6, 1> plus = p, minus = p;
      plus(axis) += step;
      minus(axis) -= step;
      const double score_plus = scoreAtP(ndt, cloud->second.source, plus);
      const double score_minus = scoreAtP(ndt, cloud->second.source, minus);
      return (score_plus - score_minus) / (2.0 * step);
    };
    const double fd_h = finiteDifference(1e-4);
    const double fd_half = finiteDifference(5e-5);
    const double analytic = analytic_gradient(axis);
    const double denom_h = std::max({std::abs(analytic), std::abs(fd_h), 1e-8});
    const double denom_half = std::max({std::abs(analytic), std::abs(fd_half), 1e-8});
    const double rel_h = std::abs(analytic - fd_h) / denom_h;
    const double rel_half = std::abs(analytic - fd_half) / denom_half;
    const bool meaningful = std::abs(analytic) > 1e-6 && std::abs(fd_half) > 1e-6;
    const bool sign_h = !meaningful || analytic * fd_h > 0.0;
    const bool sign_half = !meaningful || analytic * fd_half > 0.0;
    if (meaningful) {
      ++comparable;
      if (sign_h && sign_half) ++sign_matches;
      if (rel_h <= 0.10 && rel_half <= 0.10) ++close_derivatives;
    }
    output << item.frame_id << ',' << item.transaction_id << ",inside," << axes[axis]
           << ',' << axis << ',' << p(axis) << ',' << center_score << ',' << analytic
           << ',' << fd_h << ',' << fd_half << ',' << (sign_h ? "MATCH_OR_UNDEFINED" : "MISMATCH")
           << ',' << (sign_half ? "MATCH_OR_UNDEFINED" : "MISMATCH") << ',' << rel_h << ','
           << rel_half << ',' << roundtrip_t << ',' << roundtrip_r << ",0\n";
  }
  output.close();
  if (!output) throw std::runtime_error("score_gradient_check_write_failed");
  const std::string result = comparable > 0 && sign_matches == comparable &&
          close_derivatives == comparable && roundtrip_t <= 1e-5 && roundtrip_r <= 1e-4
      ? "PASS" : "INDETERMINATE";
  std::cout << "PCL_SCORE_SIGN_AND_COORDINATE_CHECK=" << result
            << ",case=" << item.frame_id << ",endpoint=inside"
            << ",meaningful_fd_axes=" << comparable
            << ",same_direction_axes=" << sign_matches
            << ",within_10pct_axes=" << close_derivatives
            << ",pcl_parameter_order=tx_ty_tz_rx_ry_rz"
            << ",pose_roundtrip_translation_m=" << roundtrip_t
            << ",pose_roundtrip_rotation_deg=" << roundtrip_r
            << ",ndt_align_calls=0"
            << ",selected_sha256=" << selection_hashes.first
            << ",manifest_sha256=" << selection_hashes.second << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 7) {
      std::cerr << "usage: p6_i6a_score_gradient_check SELECTED.csv MANIFEST.json "
                   "SCANS.csv XYZ.bin MAP.pcd OUT.csv\n";
      return 2;
    }
    runCheck(argv[1], argv[2], argv[3], argv[4], argv[5], argv[6]);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "P6_I6A_SCORE_GRADIENT_CHECK_FAILED: " << error.what() << '\n';
    return 1;
  }
}
