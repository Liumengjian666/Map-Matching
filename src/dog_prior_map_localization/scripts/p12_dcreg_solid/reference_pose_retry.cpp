#include "candidate_adapter.hpp"
#include "dcreg.hpp"  // Original external author implementation.
#include "probe_io.hpp"
#include "dog_prior_map_localization/solid_place_recognition.hpp"
#include <filesystem>
#include <sys/resource.h>

int main(int argc, char** argv) try {
  if (argc != 6) throw std::runtime_error("usage: retry map database samples expected_candidates output");
  const auto samples = p12::readSamples(argv[3]), expected = p12::readSamples(argv[4]);
  if (samples.size() != 6 || expected.size() != 18) throw std::runtime_error("frozen 6x3 cohort required");
  if (!std::filesystem::create_directory(argv[5])) throw std::runtime_error("output already exists");
  auto target = p12::voxel(p12::voxel(p12::loadFinite(argv[1], false), .15f, 0), .15f, 0);
  const auto target_hash = p12::hash(*target);
  if (target->size() != 459154 || target_hash != 0xea1e4e69cf9992c6ULL)
    throw std::runtime_error("P12 target parity failure");
  dog_prior_map_localization::SolidPlaceRecognitionAdapter solid;
  std::string reason;
  if (!solid.loadDatabase(argv[2], reason)) throw std::runtime_error(reason);
  std::ofstream results(std::string(argv[5])+"/retry.csv"), timing(std::string(argv[5])+"/timing.csv");
  if (!results || !timing) throw std::runtime_error("output open failure");
  results << std::setprecision(17)
      << "frame_index,stamp_ns,rank,reference_id,similarity,reference_header_stamp,reference_state_stamp,header_minus_state_sec,coordinate_status,source_points,source_hash,target_points,target_hash,A_converged,B_converged,valid_se3,nonfinite_pose,rotation_change_deg,translation_change_m,initial_distance_to_ndt_m,iterations,rmse_before_last_update_m,fitness_correspondence_fraction,call_wall_ms,author_iteration_ms,api_setup_proxy_ms,delta_from_initial_m,delta_from_initial_deg,degenerate_mask_rpy_xyz,qr_fallback_iterations,failure_reason";
  for (const auto& prefix : {"initial", "final"}) p12::poseHeader(results, prefix);
  results << '\n';
  timing << std::setprecision(17) << "frame_index,preprocessing_ms,descriptor_ms,search_ms,combination_ms,total_scan_ms,peak_rss_kib\n";
  dcreg::RunOptions options;
  options.algorithm = dcreg::Algorithm::kDCReg;
  options.parameterization = dcreg::Parameterization::kSO3;
  options.parallel_mode = dcreg::ParallelMode::kSerial;
  options.verbose = false;
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const auto& sample = samples[i];
    dcreg::Stopwatch scan_timer;
    auto source = p12::voxel(p12::loadFinite(sample.at("source_pcd"), true), .25f, 1400);
    const double preprocessing_ms = scan_timer.ElapsedMilliseconds();
    const auto source_hash = p12::hash(*source);
    if (source->size() != std::stoul(sample.at("expected_points")) ||
        source_hash != std::stoull(sample.at("expected_hash"), nullptr, 16))
      throw std::runtime_error("P12 source parity failure");
    const auto prior = p12::pose(sample, "initial_guess");
    if (!p12::rigid(prior)) throw std::runtime_error("invalid NDT prior");
    dcreg::Stopwatch combination_timer;
    const auto query = solid.query(*source, 3);
    if (!query.descriptor.valid || query.top.size() != 3) throw std::runtime_error("invalid live SOLiD query");
    // Validate the whole Top3 before any call for this frame; never replace IDs.
    for (int k = 0; k < 3; ++k) {
      const auto& old = expected[3*i+k];
      const auto& candidate = query.top[k];
      if (old.at("frame_index") != sample.at("frame_index") || std::stoi(old.at("rank")) != k+1 ||
          std::stoul(old.at("reference_id")) != candidate.id ||
          std::abs(std::stod(old.at("similarity"))-candidate.similarity) > 1e-12 ||
          std::abs(std::stod(old.at("reference_stamp"))-candidate.stamp_sec) > 1e-9 ||
          (p12::pose(old, "reference")-candidate.map_T_reference).norm() > 1e-12 ||
          (p12::pose(old, "initial")-p12::coarsePosition(candidate.map_T_reference, prior)).norm() > 1e-12)
        throw std::runtime_error("frozen P12 candidate parity failure");
    }
    for (int k = 0; k < 3; ++k) {
      const auto& old = expected[3*i+k];
      const auto& candidate = query.top[k];
      const auto initial = p12::approximateReferenceStatePose(candidate.map_T_reference);
      const auto change = dcreg::CalculatePoseError(p12::pose(old, "initial"), initial);
      dcreg::TestCase test_case;
      test_case.name = "P12B_APPROXIMATE_REFERENCE_STATE_POSE";
      test_case.has_ground_truth_pose = false;
      test_case.initial_pose = dcreg::MatrixToPose(initial);
      // Exactly the same unmodified author SolverParameters defaults as P12.
      dcreg::Stopwatch call_timer;
      const auto result = dcreg::RunRegistration(*source, *target, test_case, options);
      const double call_ms = call_timer.ElapsedMilliseconds();
      const bool valid = p12::rigid(result.transform);
      const double unavailable = std::numeric_limits<double>::quiet_NaN();
      const auto delta = valid ? dcreg::CalculatePoseError(initial, result.transform)
                              : dcreg::PoseError{unavailable, unavailable};
      std::array<bool, 6> mask{}; int qr = 0;
      for (const auto& iteration : result.history) {
        qr += iteration.used_qr_fallback;
        for (std::size_t a = 0; a < 6; ++a) mask[a] = mask[a] || iteration.degenerate_mask[a];
      }
      results << sample.at("frame_index") << ',' << sample.at("stamp_ns") << ',' << k+1 << ','
          << candidate.id << ',' << candidate.similarity << ',' << candidate.stamp_sec << ','
          << old.at("reference_mapping_state_stamp") << ',' << old.at("reference_header_minus_state_sec")
          << ",APPROXIMATE_REFERENCE_STATE_POSE," << source->size() << ',' << std::hex << source_hash
          << std::dec << ',' << target->size() << ',' << std::hex << target_hash << std::dec << ','
          << old.at("converged") << ',' << result.converged << ',' << valid << ',' << !result.transform.allFinite()
          << ',' << change.rotation_deg << ',' << change.translation_m << ','
          << (initial.block<3, 1>(0, 3)-prior.block<3, 1>(0, 3)).norm() << ','
          << result.iterations << ',' << result.rmse << ',' << result.fitness << ',' << call_ms << ','
          << result.time_ms << ',' << call_ms-result.time_ms << ',' << delta.translation_m << ',' << delta.rotation_deg << ',';
      for (bool bit : mask) results << (bit ? '1' : '0');
      results << ',' << qr << ',' << result.failure_reason;
      p12::writePose(results, initial); p12::writePose(results, result.transform); results << '\n'; results.flush();
      std::cout << "TX" << sample.at("frame_index") << " id=" << candidate.id << " A=" << old.at("converged")
                << " B=" << result.converged << " dR=" << change.rotation_deg << " ms=" << call_ms << std::endl;
    }
    const double combination_ms = combination_timer.ElapsedMilliseconds();
    struct rusage usage{}; getrusage(RUSAGE_SELF, &usage);
    timing << sample.at("frame_index") << ',' << preprocessing_ms << ',' << query.descriptor.descriptor_ms
           << ',' << query.search_ms << ',' << combination_ms << ',' << scan_timer.ElapsedMilliseconds()
           << ',' << usage.ru_maxrss << '\n'; timing.flush();
  }
  if (!results || !timing) throw std::runtime_error("output write failure");
  return 0;
} catch (const std::exception& error) {
  std::cerr << "P12B interface failure: " << error.what() << '\n'; return 1;
}
