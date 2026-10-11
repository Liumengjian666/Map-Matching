#include "candidate_adapter.hpp"
#include "dcreg.hpp"  // External author implementation, deliberately not vendored.
#include "probe_io.hpp"
#include "dog_prior_map_localization/solid_place_recognition.hpp"
#include <filesystem>
#include <sys/resource.h>

int main(int argc, char** argv) try {
  if (argc != 8) throw std::runtime_error("usage: probe map database samples output start count validate_only");
  const auto rows = p12::readSamples(argv[3]);
  const std::size_t start = std::stoul(argv[5]), count = std::stoul(argv[6]);
  const bool validate_only = std::stoi(argv[7]) != 0;
  if (!count || start+count > rows.size()) throw std::runtime_error("invalid sample range");
  if (!std::filesystem::create_directory(argv[4])) throw std::runtime_error("output already exists");
  auto target = p12::voxel(p12::voxel(p12::loadFinite(argv[1], false), .15f, 0), .15f, 0);
  dog_prior_map_localization::SolidPlaceRecognitionAdapter solid;
  std::string reason;
  if (!solid.loadDatabase(argv[2], reason)) throw std::runtime_error(reason);
  std::ofstream parity(std::string(argv[4])+"/source_parity.csv");
  parity << "frame_index,stamp_ns,source_points,source_hash,parity,target_points,target_hash\n";
  std::ofstream results(std::string(argv[4])+"/registration.csv");
  results << std::setprecision(17) << "frame_index,stamp_ns,rank,reference_id,similarity,reference_stamp,coordinate_status,success,converged,valid_se3,iterations,rmse_before_last_update_m,fitness_correspondence_fraction,call_wall_ms,author_iteration_ms,degenerate_mask_rpy_xyz,qr_fallback_iterations,correspondences_last,delta_initial_m,delta_initial_deg,delta_ndt_used_m,large_excursion,failure_reason";
  for (const auto& prefix : {"reference", "initial", "final"}) p12::poseHeader(results, prefix);
  results << '\n';
  std::ofstream timings(std::string(argv[4])+"/timing.csv");
  timings << std::setprecision(17) << "frame_index,preprocessing_ms,descriptor_ms,search_ms,query_wall_ms,local_dcreg_ms,top3_dcreg_ms,combination_ms,total_probe_ms,peak_rss_kib\n";
  dcreg::RunOptions options;
  options.algorithm = dcreg::Algorithm::kDCReg;
  options.parameterization = dcreg::Parameterization::kSO3;
  options.parallel_mode = dcreg::ParallelMode::kSerial;
  options.verbose = false;
  for (std::size_t index = start; index < start+count; ++index) {
    const auto& row = rows[index];
    dcreg::Stopwatch frame_timer;
    auto source = p12::voxel(p12::loadFinite(row.at("source_pcd"), true), .25f, 1400);
    const double preprocessing_ms = frame_timer.ElapsedMilliseconds();
    const auto expected_hash = std::stoull(row.at("expected_hash"), nullptr, 16);
    if (source->size() != std::stoul(row.at("expected_points")) || p12::hash(*source) != expected_hash)
      throw std::runtime_error("NDT source parity failure at "+row.at("frame_index"));
    parity << row.at("frame_index") << ',' << row.at("stamp_ns") << ',' << source->size() << ','
           << std::hex << p12::hash(*source) << std::dec << ",PASS," << target->size() << ','
           << std::hex << p12::hash(*target) << std::dec << '\n';
    if (validate_only) continue;
    const Eigen::Matrix4d ndt_initial = p12::pose(row, "initial_guess");
    const Eigen::Matrix4d ndt_used = p12::pose(row, "final_used");
    if (!p12::rigid(ndt_initial) || !p12::rigid(ndt_used)) throw std::runtime_error("invalid recorded NDT pose");
    dog_prior_map_localization::SolidQueryResult query;
    double query_ms = 0;
    double local_ms = 0, candidate_ms = 0;
    dcreg::Stopwatch combination_timer;
    for (int rank = 0; rank <= 3; ++rank) {
      if (rank == 1) {
        combination_timer.Reset();  // Begin real SOLiD -> top3 DCReg critical path, after local control.
        dcreg::Stopwatch query_timer;
        query = solid.query(*source, 3);  // Actual returned candidates become inputs below.
        query_ms = query_timer.ElapsedMilliseconds();
        if (!query.descriptor.valid || query.top.size() != 3)
          throw std::runtime_error("invalid SOLiD query: "+query.status);
      }
      const Eigen::Matrix4d reference = rank ? query.top[rank-1].map_T_reference : ndt_initial;
      const Eigen::Matrix4d initial = rank ? p12::coarsePosition(reference, ndt_initial) : ndt_initial;
      dcreg::TestCase test_case;
      test_case.name = "Loop3_map_matching";
      test_case.initial_pose = dcreg::MatrixToPose(initial);
      test_case.has_ground_truth_pose = false;
      // No parameter overrides: exact author SolverParameters defaults.
      dcreg::Stopwatch call_timer;
      const auto result = dcreg::RunRegistration(*source, *target, test_case, options);
      const double call_ms = call_timer.ElapsedMilliseconds();  // Includes author's tree setup/copy.
      if (rank == 0) local_ms = call_ms; else candidate_ms += call_ms;
      const bool valid = p12::rigid(result.transform);
      const double unavailable = std::numeric_limits<double>::quiet_NaN();
      const auto delta = valid ? dcreg::CalculatePoseError(initial, result.transform)
                               : dcreg::PoseError{unavailable, unavailable};
      std::array<bool, 6> mask{};
      int qr = 0;
      for (const auto& iteration : result.history) {
        for (std::size_t axis = 0; axis < 6; ++axis) mask[axis] = mask[axis] || iteration.degenerate_mask[axis];
        qr += iteration.used_qr_fallback;
      }
      results << row.at("frame_index") << ',' << row.at("stamp_ns") << ',' << rank << ',';
      if (rank) results << query.top[rank-1].id;
      results << ',';
      if (rank) results << query.top[rank-1].similarity;
      results << ',';
      if (rank) results << query.top[rank-1].stamp_sec;
      results << ',' << (rank ? "COARSE_POSITION_HYPOTHESIS" : "RECORDED_NDT_INITIAL_GUESS")
              << ',' << result.success << ',' << result.converged << ',' << valid
              << ',' << result.iterations << ',' << result.rmse << ',' << result.fitness
              << ',' << call_ms << ',' << result.time_ms << ',';
      for (bool bit : mask) results << (bit ? '1' : '0');
      results << ',' << qr << ',' << (result.history.empty() ? 0 : result.history.back().correspondence_count)
              << ',' << delta.translation_m << ',' << delta.rotation_deg << ',';
      if (valid) results << dcreg::CalculatePoseError(ndt_used, result.transform).translation_m;
      results << ',';
      if (valid) results << (delta.translation_m > 3 || delta.rotation_deg > 45);
      results << ',' << result.failure_reason;
      p12::writePose(results, reference); p12::writePose(results, initial); p12::writePose(results, result.transform);
      results << '\n'; results.flush();
      std::cout << "frame=" << row.at("frame_index") << " rank=" << rank << " converged=" << result.converged
                << " rigid=" << valid << " ms=" << call_ms << std::endl;
    }
    const double top3_end_to_end_ms = combination_timer.ElapsedMilliseconds();
    struct rusage usage{}; getrusage(RUSAGE_SELF, &usage);
    timings << row.at("frame_index") << ',' << preprocessing_ms << ',' << query.descriptor.descriptor_ms
            << ',' << query.search_ms << ',' << query_ms << ',' << local_ms << ',' << candidate_ms
            << ',' << top3_end_to_end_ms << ',' << frame_timer.ElapsedMilliseconds() << ',' << usage.ru_maxrss << '\n';
    timings.flush();
  }
  return 0;
} catch (const std::exception& error) {
  std::cerr << "P12 input/interface failure: " << error.what() << '\n'; return 1;
}
