#define P9_NDT_ENERGY_CONTRACT_LIBRARY
#include "p9_ndt_energy_contract.cpp"

#include <cstdio>
#include <tuple>
#include <unistd.h>

namespace {
const std::vector<std::uint64_t> r2FrameOrder{
    120, 244, 368, 616, 740, 838, 839, 864, 924, 925, 1111, 1235, 1359, 1497,
    1498, 1556, 1557, 1606, 1730, 1854, 2102, 2226, 2350, 2598, 2722, 2846,
    3094, 3217, 3341, 3631, 3796, 3962};
const std::vector<std::uint64_t> r2MajorFrames{
    368, 616, 2226, 2350, 2722, 2846, 3341, 3796, 3962};
constexpr int kR2BaseSeeds = 263;
constexpr int kR2ProbeBudget = 16;

struct R2RandomBasis {
  std::uint64_t frame = 0;
  int replicate = -1;
  Eigen::Matrix<double, 6, 2> basis = Eigen::Matrix<double, 6, 2>::Zero();
};

struct R2Probe {
  std::uint64_t frame = 0;
  std::string frame_id;
  std::string method;
  int replicate = -1;
  int rank = 0;
  int seed_index = -1;
  Eigen::Matrix4f start = Eigen::Matrix4f::Identity();
};

std::string r2MatrixText(const Eigen::Matrix4f& pose) {
  std::ostringstream out;
  out << std::setprecision(17);
  for (int i = 0; i < 16; ++i) {
    if (i) out << ';';
    out << pose(i / 4, i % 4);
  }
  return out.str();
}

std::string r2PoseText(const Eigen::Matrix4f& pose) {
  const Eigen::Quaterniond rotation(pose.block<3, 3>(0, 0).cast<double>());
  const Eigen::Quaterniond normalized = rotation.normalized();
  Eigen::Matrix<double, 7, 1> xyz_q;
  xyz_q << pose(0, 3), pose(1, 3), pose(2, 3), normalized.x(), normalized.y(),
      normalized.z(), normalized.w();
  std::ostringstream out;
  out << std::setprecision(17);
  for (int i = 0; i < xyz_q.size(); ++i) {
    if (i) out << ';';
    out << xyz_q(i);
  }
  return out.str();
}

template <typename Derived>
std::string r2VectorText(const Eigen::MatrixBase<Derived>& vector) {
  std::ostringstream out;
  out << std::setprecision(17);
  for (Eigen::Index i = 0; i < vector.size(); ++i) {
    if (i) out << ';';
    out << vector.derived().coeff(i);
  }
  return out.str();
}

std::string r2ShardPath(const std::string& directory, std::uint64_t frame,
                        const std::string& method, int replicate) {
  std::ostringstream path;
  path << directory << "/tx_" << frame << '_' << method << "_r" << replicate << ".csv";
  return path.str();
}

std::map<std::pair<std::uint64_t, int>, R2RandomBasis> r2ReadRandomBases(
    const std::string& path) {
  const CsvTable table = readCsv(path);
  std::map<std::pair<std::uint64_t, int>, R2RandomBasis> result;
  for (const auto& row : table.rows) {
    R2RandomBasis item;
    item.frame = parseU64(table.get(row, "frame"));
    item.replicate = std::stoi(table.get(row, "random_rep"));
    if (table.get(row, "rng") != "PCG64")
      throw std::runtime_error("unexpected random basis RNG");
    const std::vector<double> values = parseSemicolonDoubles(
        table.get(row, "basis_rowmajor"), 12);
    for (int i = 0; i < 12; ++i) item.basis(i / 2, i % 2) = values[i];
    if (!item.basis.allFinite() || item.replicate < 0 || item.replicate >= 3 ||
        (item.basis.transpose() * item.basis - Eigen::Matrix2d::Identity()).norm() > 1e-12 ||
        std::find(r2FrameOrder.begin(), r2FrameOrder.end(), item.frame) == r2FrameOrder.end() ||
        !result.emplace(std::make_pair(item.frame, item.replicate), item).second)
      throw std::runtime_error("invalid or duplicate R2 random basis");
  }
  if (result.size() != r2FrameOrder.size() * 3)
    throw std::runtime_error("R2 random basis manifest must contain 96 entries");
  return result;
}

void r2Prepare(const std::string& cohort_path, const std::string& uobs_path,
               const std::string& candidate_path, const std::string& random_path,
               const std::string& output_directory) {
  if (!runMathSelfTests()) throw std::runtime_error("base P9 math self-test failed");
  const auto frame_sources = readFrameSources(cohort_path);
  const auto uobs = readArchivedUObs(uobs_path);
  const CsvTable candidates = readCsv(candidate_path);
  const auto random = r2ReadRandomBases(random_path);
  if (frame_sources.size() != r2FrameOrder.size())
    throw std::runtime_error("cohort must contain exactly 32 frozen frames");

  for (std::size_t i = 0; i < r2FrameOrder.size(); ++i) {
    if (frame_sources.count(r2FrameOrder[i]) != 1 || uobs.count(r2FrameOrder[i]) != 1 ||
        !uobs.at(r2FrameOrder[i]).valid)
      throw std::runtime_error("cohort/U_obs frame coverage or validity mismatch");
    const ArchivedUObs& observation = uobs.at(r2FrameOrder[i]);
    const FrameSource& frame = frame_sources.at(r2FrameOrder[i]);
    if (observation.length_scale_m != static_cast<double>(0.8f) ||
        observation.configured_resolution_m != 0.8 || observation.step_size != 0.08 ||
        observation.epsilon != 1e-5 || observation.maximum_iterations != 80 ||
        observation.source_hash != frame.expected_hash ||
        observation.source_points != frame.expected_points ||
        (observation.eigenvectors.transpose() * observation.eigenvectors - Matrix6d::Identity()).norm() > 1e-12 ||
        observation.eigenvalues.minCoeff() <= 0.0 ||
        !observation.eigenvalues.allFinite())
      throw std::runtime_error("frozen U_obs / frame source contract mismatch");
  }
  if (candidates.rows.size() != 8800)
    throw std::runtime_error("candidate archive row count mismatch (expected 8800)");

  std::ofstream pool(output_directory + "/proposal_pool.csv");
  std::ofstream parity(output_directory + "/proposal_roundtrip.csv");
  if (!pool || !parity) throw std::runtime_error("cannot create R2 proposal artifacts");
  pool.exceptions(std::ios::badbit | std::ios::failbit);
  parity.exceptions(std::ios::badbit | std::ios::failbit);
  writeHeader(pool, {"frame", "frame_id", "method", "random_rep", "seed_index",
                     "coord0", "coord1", "eta_original", "eta_projected",
                     "nominal_pose_matrix16", "start_pose_matrix16"});
  writeHeader(parity, {"frame", "seed_index", "translation_error_m", "rotation_error_deg", "pass"});

  std::map<std::uint64_t, std::set<int>> seen;
  std::size_t base_rows = 0, pool_rows = 0;
  double max_translation_error = 0.0, max_rotation_error = 0.0;
  bool parity_pass = true;
  for (const auto& row : candidates.rows) {
    const std::uint64_t frame = parseU64(candidates.get(row, "transaction_id"));
    if (frame_sources.count(frame) == 0) continue;
    const int seed = std::stoi(candidates.get(row, "seed_index"));
    if (seed < 0 || seed >= kR2BaseSeeds) continue;  // Exclude only the archived targeted extension.
    if (!seen[frame].insert(seed).second)
      throw std::runtime_error("duplicate base seed index in frozen candidate archive");

    const FrameSource& source = frame_sources.at(frame);
    const ArchivedUObs& observation = uobs.at(frame);
    if (candidates.get(row, "frame_id") != source.frame_id ||
        candidates.get(row, "source_hash_actual") != std::to_string(source.expected_hash) ||
        candidates.get(row, "source_hash_expected") != std::to_string(source.expected_hash) ||
        candidates.get(row, "source_points") != std::to_string(source.expected_points) ||
        candidates.get(row, "target_points") != "549606")
      throw std::runtime_error("candidate/source/frame contract mismatch");

    const Eigen::Matrix4f archived_start = parsePose(candidates.get(row, "start_pose_xyz_q_xyzw"));
    const Vector6d eta = mapChartDisplacement(observation.pose, archived_start);
    const Eigen::Matrix4f round_trip = poseAtEta(observation.pose, eta);
    const double translation_error = translationDistance(archived_start, round_trip);
    const double rotation_error = rotationDistanceDeg(archived_start, round_trip);
    const bool pass = translation_error <= 1e-5 && rotation_error <= 1e-4;
    parity_pass = parity_pass && pass;
    max_translation_error = std::max(max_translation_error, translation_error);
    max_rotation_error = std::max(max_rotation_error, rotation_error);
    writeRow(parity, {std::to_string(frame), std::to_string(seed), number(translation_error),
                      number(rotation_error), pass ? "1" : "0"});
    ++base_rows;

    const auto emit = [&](const std::string& method, int replicate,
                          const Eigen::Matrix<double, 6, 2>& basis) {
      const Eigen::Vector2d coordinate = basis.transpose() * eta;
      const Vector6d projected = basis * coordinate;
      if (projected.norm() > eta.norm() + 1e-10)
        throw std::runtime_error("orthogonal projection increased eta norm");
      const Eigen::Matrix4f start = poseAtEta(observation.pose, projected);
      writeRow(pool, {std::to_string(frame), source.frame_id, method,
                      std::to_string(replicate), std::to_string(seed),
                      number(coordinate.x()), number(coordinate.y()),
                      r2VectorText(eta), r2VectorText(projected),
                      r2MatrixText(observation.pose), r2MatrixText(start)});
      ++pool_rows;
    };
    emit("WEAK2", -1, observation.eigenvectors.leftCols<2>());
    emit("STRONG2", -1, observation.eigenvectors.rightCols<2>());
    for (int replicate = 0; replicate < 3; ++replicate)
      emit("RANDOM2", replicate, random.at({frame, replicate}).basis);
  }
  if (base_rows != r2FrameOrder.size() * kR2BaseSeeds || pool_rows != base_rows * 5)
    throw std::runtime_error("incomplete R2 base proposal pool");
  for (std::uint64_t frame : r2FrameOrder)
    if (seen[frame].size() != kR2BaseSeeds || *seen[frame].begin() != 0 ||
        *seen[frame].rbegin() != 262)
      throw std::runtime_error("base proposal indices must be exactly 0..262 per frame");
  pool.close();
  parity.close();
  std::cout << "R2_PROPOSALS=" << pool_rows << " BASE_SEEDS=" << base_rows
            << " ROUNDTRIP_PASS=" << (parity_pass ? "YES" : "NO")
            << " MAX_TRANSLATION_ERROR_M=" << std::setprecision(17) << max_translation_error
            << " MAX_ROTATION_ERROR_DEG=" << max_rotation_error << '\n';
  if (!parity_pass) throw std::runtime_error("MATCHED_PROPOSAL_CHART_PARITY_FAIL");
}

std::vector<R2Probe> r2ReadProbeManifest(const std::string& path) {
  const CsvTable table = readCsv(path);
  std::vector<R2Probe> result;
  result.reserve(table.rows.size());
  for (const auto& row : table.rows) {
    R2Probe probe;
    probe.frame = parseU64(table.get(row, "frame"));
    probe.frame_id = table.get(row, "frame_id");
    probe.method = table.get(row, "method");
    probe.replicate = std::stoi(table.get(row, "random_rep"));
    probe.rank = std::stoi(table.get(row, "probe_rank"));
    probe.seed_index = std::stoi(table.get(row, "seed_index"));
    probe.start = parseMatrix16(table.get(row, "start_pose_matrix16"));
    if (!probe.start.allFinite() || probe.rank < 1 || probe.rank > kR2ProbeBudget ||
        probe.seed_index < 0 || probe.seed_index >= kR2BaseSeeds ||
        (probe.method != "WEAK2" && probe.method != "STRONG2" && probe.method != "RANDOM2") ||
        (probe.method == "RANDOM2" ? (probe.replicate < 0 || probe.replicate >= 3)
                                    : probe.replicate != -1))
      throw std::runtime_error("invalid R2 selected probe row");
    result.push_back(probe);
  }
  if (result.size() != r2FrameOrder.size() * 5 * kR2ProbeBudget)
    throw std::runtime_error("R2 probe manifest must contain exactly 2560 selected probes");
  return result;
}

bool r2ValidateExistingShard(const std::string& path, std::uint64_t frame,
                             const std::string& method, int replicate) {
  std::ifstream check(path);
  if (!check.good()) return false;
  check.close();
  const CsvTable table = readCsv(path);
  if (table.rows.size() != kR2ProbeBudget)
    throw std::runtime_error("existing R2 shard is incomplete; refusing to repeat NDT calls: " + path);
  std::set<int> ranks, seeds;
  for (const auto& row : table.rows) {
    if (parseU64(table.get(row, "frame")) != frame || table.get(row, "method") != method ||
        std::stoi(table.get(row, "random_rep")) != replicate)
      throw std::runtime_error("existing R2 shard identity mismatch: " + path);
    const int rank = std::stoi(table.get(row, "probe_rank"));
    const int seed = std::stoi(table.get(row, "seed_index"));
    if (rank < 1 || rank > kR2ProbeBudget || seed < 0 || seed >= kR2BaseSeeds ||
        !ranks.insert(rank).second || !seeds.insert(seed).second ||
        !parseMatrix16(table.get(row, "terminal_pose_matrix16")).allFinite() ||
        !std::isfinite(std::stod(table.get(row, "raw_ndt_score_sum"))))
      throw std::runtime_error("existing R2 shard contains invalid or duplicate result: " + path);
  }
  if (ranks.size() != kR2ProbeBudget)
    throw std::runtime_error("existing R2 shard rank coverage mismatch: " + path);
  return true;
}

void r2AlignAll(const std::string& map_path, const std::string& cohort_path,
                const std::string& uobs_path, const std::string& probe_path,
                const std::string& output_directory) {
  const auto frames = readFrameSources(cohort_path);
  const auto uobs = readArchivedUObs(uobs_path);
  const std::vector<R2Probe> probes = r2ReadProbeManifest(probe_path);
  std::map<std::tuple<std::uint64_t, std::string, int>, std::vector<R2Probe>> grouped;
  for (const R2Probe& probe : probes)
    grouped[{probe.frame, probe.method, probe.replicate}].push_back(probe);
  for (auto& entry : grouped) {
    auto& group = entry.second;
    std::sort(group.begin(), group.end(), [](const R2Probe& a, const R2Probe& b) {
      return a.rank < b.rank;
    });
    if (group.size() != kR2ProbeBudget)
      throw std::runtime_error("R2 task does not contain 16 probes");
    for (int i = 0; i < kR2ProbeBudget; ++i)
      if (group[static_cast<std::size_t>(i)].rank != i + 1)
        throw std::runtime_error("R2 selection rank sequence is incomplete");
  }
  if (grouped.size() != r2FrameOrder.size() * 5)
    throw std::runtime_error("R2 method/frame task coverage must be 160");

  Cloud::Ptr target = loadTarget(map_path);
  if (target->size() != 549606) throw std::runtime_error("official frozen map target count mismatch");
  ExactPclNdt ndt;
  ndt.setResolution(0.8f);
  ndt.setOulierRatio(0.55);
  ndt.configureScoreConstants();
  ndt.setInputTarget(target);
  ndt.setStepSize(0.08);
  ndt.setTransformationEpsilon(1e-5);
  ndt.setMaximumIterations(80);
  for (float leaf : ndt.actualGridLeaf())
    if (std::abs(leaf - 0.8f) > 1e-6)
      throw std::runtime_error("actual target NDT grid differs from 0.8m");

  std::size_t completed = 0, skipped = 0;
  for (std::uint64_t frame : r2FrameOrder) {
    const auto source_info = frames.find(frame);
    if (source_info == frames.end()) throw std::runtime_error("missing frame in frozen cohort");
    Cloud::Ptr source = preprocessSource(loadPackedSource(source_info->second.path));
    if (source->size() != source_info->second.expected_points ||
        sourceHash(*source) != source_info->second.expected_hash)
      throw std::runtime_error("prepared source hash/count mismatch for frame " + std::to_string(frame));
    ndt.setInputSource(source);
    if (uobs.count(frame) != 1 || !uobs.at(frame).valid)
      throw std::runtime_error("missing valid nominal U_obs for frame " + std::to_string(frame));
    const Eigen::Matrix4f nominal_pose = uobs.at(frame).pose;
    const double nominal_score = ndt.dynamicValueOnly(source, nominal_pose);
    if (!std::isfinite(nominal_score))
      throw std::runtime_error("invalid nominal NDT objective for frame " + std::to_string(frame));

    for (const auto& task : grouped) {
      if (std::get<0>(task.first) != frame) continue;
      const std::string method = std::get<1>(task.first);
      const int replicate = std::get<2>(task.first);
      const std::string final_path = r2ShardPath(output_directory + "/runs", frame, method, replicate);
      if (r2ValidateExistingShard(final_path, frame, method, replicate)) {
        ++skipped;
        continue;
      }
      const std::string temporary_path = final_path + ".partial." + std::to_string(getpid());
      std::ofstream output(temporary_path);
      if (!output) throw std::runtime_error("cannot create R2 result shard: " + temporary_path);
      output.exceptions(std::ios::badbit | std::ios::failbit);
      writeHeader(output, {"frame", "frame_id", "method", "random_rep", "probe_rank", "seed_index",
          "start_pose_matrix16", "terminal_pose_matrix16", "terminal_pose_xyz_q_xyzw",
          "terminal_disp_0", "terminal_disp_1", "terminal_disp_2", "terminal_disp_3",
          "terminal_disp_4", "terminal_disp_5", "terminal_disp_norm", "translation_from_nominal_m",
          "rotation_from_nominal_deg", "converged", "iterations", "runtime_ms", "raw_ndt_score_sum",
          "nominal_ndt_score_sum", "delta_score_sum", "objective_eval_ms", "source_points", "source_hash",
          "target_points", "status"});

      for (const R2Probe& probe : task.second) {
        Cloud aligned;
        const auto start = std::chrono::steady_clock::now();
        ndt.align(aligned, probe.start);
        const double runtime_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
        const Eigen::Matrix4f terminal = ndt.getFinalTransformation();
        const bool converged = ndt.hasConverged();
        const int iterations = ndt.getFinalNumIteration();
        if (!terminal.allFinite() || iterations < 0 || iterations > 80 ||
            !std::isfinite(runtime_ms) || !std::isfinite(translationDistance(probe.start, terminal)) ||
            !std::isfinite(rotationDistanceDeg(probe.start, terminal)))
          throw std::runtime_error("PCL returned an invalid R2 terminal");
        const Vector6d displacement = mapChartDisplacement(nominal_pose, terminal);
        const auto objective_begin = std::chrono::steady_clock::now();
        const double terminal_score = ndt.dynamicValueOnly(source, terminal);
        const double objective_eval_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - objective_begin).count();
        if (!displacement.allFinite() || !std::isfinite(terminal_score) ||
            !std::isfinite(nominal_score) || !std::isfinite(objective_eval_ms))
          throw std::runtime_error("invalid R2 objective/displacement value");
        const std::string status = iterations >= 80 ? "ITERATION_LIMIT" :
            (converged ? "SUCCESS" : "NOT_CONVERGED");
        writeRow(output, {std::to_string(frame), probe.frame_id, method, std::to_string(replicate),
            std::to_string(probe.rank), std::to_string(probe.seed_index), r2MatrixText(probe.start),
            r2MatrixText(terminal), r2PoseText(terminal), number(displacement(0)), number(displacement(1)),
            number(displacement(2)), number(displacement(3)), number(displacement(4)), number(displacement(5)),
            number(displacement.norm()), number(translationDistance(nominal_pose, terminal)),
            number(rotationDistanceDeg(nominal_pose, terminal)),
            converged ? "1" : "0", std::to_string(iterations), number(runtime_ms), number(terminal_score),
            number(nominal_score), number(terminal_score - nominal_score), number(objective_eval_ms),
            std::to_string(source->size()), std::to_string(sourceHash(*source)), std::to_string(target->size()), status});
      }
      output.close();
      if (std::rename(temporary_path.c_str(), final_path.c_str()) != 0)
        throw std::runtime_error("cannot atomically finalize R2 shard: " + final_path);
      ++completed;
      std::cerr << "R2_SHARD_COMPLETE frame=" << frame << " method=" << method
                << " replicate=" << replicate << " probes=16\n";
    }
  }
  std::cout << "R2_ALIGN_TASKS_COMPLETED=" << completed << " SKIPPED=" << skipped
            << " TOTAL_TASKS=160 NEW_NDT_CALLS=" << completed * kR2ProbeBudget << '\n';
}

bool r2SelfTest() {
  if (!runMathSelfTests()) return false;
  const Eigen::Matrix<double, 6, 2> basis = Matrix6d::Identity().leftCols<2>();
  Vector6d eta; eta << 1.0, -2.0, 0.5, 0.03, -0.04, 0.02;
  const Eigen::Vector2d coordinates = basis.transpose() * eta;
  const Vector6d projected = basis * coordinates;
  if (std::abs(projected.norm() - coordinates.norm()) > 1e-12 ||
      (projected - basis * (basis.transpose() * projected)).norm() > 1e-12 ||
      (basis.transpose() * basis - Eigen::Matrix2d::Identity()).norm() > 1e-12)
    return false;
  const Eigen::Matrix4f nominal = Eigen::Matrix4f::Identity();
  const Eigen::Matrix4f start = poseAtEta(nominal, projected);
  const Vector6d recovered = mapChartDisplacement(nominal, start);
  return (recovered - projected).norm() < 2e-6;
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--self-test") {
      if (!r2SelfTest()) throw std::runtime_error("R2 terminal stability self-test failed");
      std::cout << "P9_R2_TERMINAL_STABILITY_SELF_TEST=PASS\n";
      return 0;
    }
    if (argc == 7 && std::string(argv[1]) == "--prepare") {
      r2Prepare(argv[2], argv[3], argv[4], argv[5], argv[6]);
      return 0;
    }
    if (argc == 7 && std::string(argv[1]) == "--align-all") {
      r2AlignAll(argv[2], argv[3], argv[4], argv[5], argv[6]);
      return 0;
    }
    throw std::runtime_error(
        "usage: --prepare COHORT UOBS CANDIDATES RANDOM OUTDIR | "
        "--align-all MAP COHORT UOBS PROBES OUTDIR | --self-test");
  } catch (const std::exception& error) {
    std::cerr << "P9_R2_CONTRACT_FAIL=" << error.what() << '\n';
    return 1;
  }
}
