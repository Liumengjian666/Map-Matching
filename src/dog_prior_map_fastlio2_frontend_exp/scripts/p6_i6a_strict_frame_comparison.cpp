// P6-I6A strict-vs-baseline NDT comparison from the frozen P6-I5C original
// inside/outside seeds. The existing I5C restore/preprocessing/alignment
// helpers are embedded unchanged; only its standalone entry point is omitted.
#define P6_I5C_NO_MAIN
#include "../scripts/p6_i5c_stationary_audit.cpp"
#undef P6_I5C_NO_MAIN

#include <filesystem>

namespace {
using namespace p6_i5c_app;

struct PairResult {
  std::array<AlignResult, 2> baseline;
  std::array<AlignResult, 2> strict;
};

void writeFrameRow(std::ostream& output, const SelectedCase& item,
                   const PairResult& result) {
  const double original_translation_gap =
      translationDistance(item.inside.saved_terminal, item.outside.saved_terminal);
  const double original_rotation_gap =
      rotationDistanceDeg(item.inside.saved_terminal, item.outside.saved_terminal);
  const double baseline_translation_gap =
      translationDistance(result.baseline[0].terminal, result.baseline[1].terminal);
  const double baseline_rotation_gap =
      rotationDistanceDeg(result.baseline[0].terminal, result.baseline[1].terminal);
  const double strict_translation_gap =
      translationDistance(result.strict[0].terminal, result.strict[1].terminal);
  const double strict_rotation_gap =
      rotationDistanceDeg(result.strict[0].terminal, result.strict[1].terminal);

  output << item.frame_id << ',' << item.transaction_id << ',' << item.group << ','
         << fixed(item.jump_t_m) << ',' << fixed(item.jump_r_deg) << ','
         << fixed(original_translation_gap) << ',' << fixed(original_rotation_gap) << ','
         << fixed(baseline_translation_gap) << ',' << fixed(baseline_rotation_gap) << ','
         << fixed(strict_translation_gap) << ',' << fixed(strict_rotation_gap);
  for (const AlignResult& aligned : result.baseline)
    output << ',' << (aligned.converged ? "true" : "false") << ',' << aligned.iterations
           << ',' << fixed(aligned.objective) << ',' << fixed(aligned.fitness)
           << ',' << fixed(aligned.runtime_ms) << ',' << pose7(aligned.terminal);
  for (const AlignResult& aligned : result.strict)
    output << ',' << (aligned.converged ? "true" : "false") << ',' << aligned.iterations
           << ',' << fixed(aligned.objective) << ',' << fixed(aligned.fitness)
           << ',' << fixed(aligned.runtime_ms) << ',' << pose7(aligned.terminal);
  output << '\n';
}

void runComparison(const std::string& selected_path, const std::string& manifest_path,
                   const std::string& scans_path, const std::string& packed_path,
                   const std::string& map_path, const std::string& output_directory) {
  requireOmpSingleThread();
  std::filesystem::create_directories(output_directory);
  const auto selection_hashes = verifyFrozenSelection(selected_path, manifest_path);
  verifyFrozenFiles(scans_path, packed_path, map_path);
  const std::vector<SelectedCase> selected = readSelected(selected_path);
  if (selected.size() != 7) throw std::runtime_error("frozen_selection_must_contain_7_cases");
  const auto clouds = loadCloudContexts(selected, scans_path, packed_path);
  const Cloud::Ptr target = ::loadTarget(map_path);
  if (target->size() != 549606) throw std::runtime_error("frozen_target_point_count_mismatch");

  const std::string csv_path = output_directory + "/strict_7_frame_comparison.csv";
  const std::string ledger_path = output_directory + "/call_accounting.csv";
  if (std::filesystem::exists(csv_path) || std::filesystem::exists(ledger_path))
    throw std::runtime_error("refusing_to_overwrite_existing_i6a_frame_outputs");
  std::ofstream output(csv_path);
  if (!output) throw std::runtime_error("cannot_create_strict_frame_comparison_csv");
  output << std::setprecision(17)
      << "frame_id,transaction_id,selection_group,original_inside_outside_jump_t_m,"
         "original_inside_outside_jump_r_deg,original_saved_terminal_gap_t_m,"
         "original_saved_terminal_gap_r_deg,base_direct_terminal_gap_t_m,"
         "base_direct_terminal_gap_r_deg,strict_direct_terminal_gap_t_m,"
         "strict_direct_terminal_gap_r_deg"
         ",base_inside_converged,base_inside_iterations,base_inside_objective,"
         "base_inside_fitness,base_inside_runtime_ms,base_inside_terminal_xyz_q_xyzw"
         ",base_outside_converged,base_outside_iterations,base_outside_objective,"
         "base_outside_fitness,base_outside_runtime_ms,base_outside_terminal_xyz_q_xyzw"
         ",strict_inside_converged,strict_inside_iterations,strict_inside_objective,"
         "strict_inside_fitness,strict_inside_runtime_ms,strict_inside_terminal_xyz_q_xyzw"
         ",strict_outside_converged,strict_outside_iterations,strict_outside_objective,"
         "strict_outside_fitness,strict_outside_runtime_ms,strict_outside_terminal_xyz_q_xyzw\n";

  CallLedger ledger(ledger_path, selection_hashes.first, selection_hashes.second);
  uint64_t next_call = 1;
  uint64_t calls_used = 0;
  for (const SelectedCase& item : selected) {
    const auto cloud = clouds.find(item.transaction_id);
    if (cloud == clouds.end()) throw std::runtime_error("frozen_case_cloud_missing");
    const std::array<const EndpointInput*, 2> endpoints = {&item.inside, &item.outside};
    PairResult result;
    for (std::size_t side = 0; side < endpoints.size(); ++side) {
      const EndpointInput& endpoint = *endpoints[side];
      result.baseline[side] = alignOnce(target, cloud->second.source,
          endpoint.original_seed, 40, 0.001, "I6A_BASE", item,
          endpoint.label, "DIRECT_FROM_ORIGINAL_SEED", "FROZEN_ORIGINAL_SEED",
          ledger, &next_call, &calls_used);
      result.strict[side] = alignOnce(target, cloud->second.source,
          endpoint.original_seed, 80, 1e-5, "I6A_STRICT", item,
          endpoint.label, "DIRECT_FROM_ORIGINAL_SEED", "FROZEN_ORIGINAL_SEED",
          ledger, &next_call, &calls_used);
    }
    writeFrameRow(output, item, result);
    output.flush();
    if (!output) throw std::runtime_error("strict_frame_csv_write_failed");
    std::cout << "I6A_FRAME," << item.frame_id
              << ",original_gap_m="
              << translationDistance(item.inside.saved_terminal, item.outside.saved_terminal)
              << ",base_direct_gap_m="
              << translationDistance(result.baseline[0].terminal, result.baseline[1].terminal)
              << ",strict_direct_gap_m="
              << translationDistance(result.strict[0].terminal, result.strict[1].terminal)
              << '\n';
  }
  std::cout << "I6A_FRAME_COMPARISON_COMPLETE,frames=" << selected.size()
            << ",align_calls=" << calls_used << ",selected_sha256="
            << selection_hashes.first << ",manifest_sha256=" << selection_hashes.second
            << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 7) {
      std::cerr << "usage: p6_i6a_strict_frame_comparison SELECTED.csv MANIFEST.json "
                   "SCANS.csv XYZ.bin MAP.pcd OUT_DIR\n";
      return 2;
    }
    runComparison(argv[1], argv[2], argv[3], argv[4], argv[5], argv[6]);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "P6_I6A_FRAME_COMPARISON_FAILED: " << error.what() << '\n';
    return 1;
  }
}
