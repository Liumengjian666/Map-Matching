// Offline-only P6-I5C targeted terminal audit.
#define main p5_i1_unused_entry_point
#include "p5_i1_ndt_mode_landscape.cpp"
#undef main

#include "p6_i5c_stationary_math.hpp"

#include <Eigen/Eigenvalues>
#include <pcl/common/transforms.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace p6_i5c_app {
using Cloud = ::Cloud;
using Point = ::Point;
using Pose = p6_i5c::Pose;

// Read-only P6-I4 packed-scan adapter. Kept local because directly including
// the P6-I4 translation unit nests main-renaming macros from P4-I2/P5-I1.
// Field meanings, sequence validation, packed XYZ reads, and cloud metadata
// match P6-I4's ScanAsset/readScanAssets/loadRawCloudAt implementation.
struct CsvTable {
  std::vector<std::string> header;
  std::vector<std::vector<std::string>> rows;
  std::map<std::string, std::size_t> columns;

  const std::string& get(const std::vector<std::string>& row,
                         const std::string& name) const {
    const auto found = columns.find(name);
    if (found == columns.end() || found->second >= row.size())
      throw std::runtime_error("missing_csv_column:" + name);
    return row[found->second];
  }
};

struct ScanAsset {
  uint64_t transaction_id = 0;
  uint64_t stamp_ns = 0;
  double time_s = 0.0;
  uint64_t cloud_byte_offset = 0;
  uint64_t cloud_point_count = 0;
  uint64_t expected_source_hash = 0;
  uint64_t request_cloud_hash = 0;
};

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) fields.push_back(field);
  return fields;
}

CsvTable readCsv(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot_open_csv:" + path);
  CsvTable table;
  std::string line;
  if (!std::getline(input, line)) throw std::runtime_error("empty_csv:" + path);
  table.header = splitCsv(line);
  for (std::size_t index = 0; index < table.header.size(); ++index)
    table.columns[table.header[index]] = index;
  while (std::getline(input, line))
    if (!line.empty()) table.rows.push_back(splitCsv(line));
  return table;
}

uint64_t parseU64(const CsvTable& table, const std::vector<std::string>& row,
                  const std::string& field) {
  return std::stoull(table.get(row, field));
}

double parseCsvDouble(const CsvTable& table, const std::vector<std::string>& row,
                      const std::string& field) {
  const double value = std::stod(table.get(row, field));
  if (!std::isfinite(value)) throw std::runtime_error("nonfinite_csv_value:" + field);
  return value;
}

std::vector<ScanAsset> readScanAssets(const std::string& path) {
  const CsvTable table = readCsv(path);
  std::vector<ScanAsset> assets;
  assets.reserve(table.rows.size());
  uint64_t expected_transaction = 1;
  for (const auto& row : table.rows) {
    ScanAsset asset;
    asset.transaction_id = parseU64(table, row, "transaction_id");
    asset.stamp_ns = parseU64(table, row, "stamp_ns");
    asset.time_s = parseCsvDouble(table, row, "time_s");
    asset.cloud_byte_offset = parseU64(table, row, "cloud_byte_offset");
    asset.cloud_point_count = parseU64(table, row, "cloud_point_count");
    asset.expected_source_hash = parseU64(table, row, "ndt_source_cloud_hash");
    asset.request_cloud_hash = parseU64(table, row, "request_cloud_hash");
    if (asset.transaction_id != expected_transaction || asset.stamp_ns == 0 ||
        asset.cloud_point_count == 0)
      throw std::runtime_error("invalid_scan_asset_sequence");
    ++expected_transaction;
    assets.push_back(asset);
  }
  if (assets.size() != 4127) throw std::runtime_error("scan_metadata_count_not_4127");
  return assets;
}

Cloud::Ptr loadRawCloudAt(const std::string& path, const ScanAsset& asset) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("cannot_open_packed_xyz_file");
  input.seekg(static_cast<std::streamoff>(asset.cloud_byte_offset));
  if (!input) throw std::runtime_error("cannot_seek_packed_xyz_file");
  Cloud::Ptr cloud(new Cloud);
  cloud->reserve(static_cast<std::size_t>(asset.cloud_point_count));
  for (uint64_t index = 0; index < asset.cloud_point_count; ++index) {
    float xyz[3];
    input.read(reinterpret_cast<char*>(xyz), sizeof(xyz));
    if (input.gcount() != static_cast<std::streamsize>(sizeof(xyz)))
      throw std::runtime_error("truncated_packed_xyz_file");
    cloud->push_back(Point(xyz[0], xyz[1], xyz[2]));
  }
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = false;
  return cloud;
}

Pose poseFromPcl(const Eigen::Matrix4f& pose) { return pose.cast<double>(); }

constexpr char kExpectedPackedSha[] =
    "f7b5262552fe8d52f383e50813de2b568fa8a5990c69e2bba55231df8547188f";
constexpr char kExpectedScansSha[] =
    "9371e593c0e625611f053e3ef0a581c52481ccaed392aa8d74d74313caa9938f";
constexpr char kExpectedSelectedCasesSha[] =
    "94e858a39a61f4a653e6b5562e8efac706b12fe314bcdbffddd597f38cb090eb";
constexpr char kExpectedSelectionManifestSha[] =
    "ce68154111c95998b1acb2cca9c9703af18d96cd6e4ca4094c60ce6f201d5ed1";
constexpr char kSelectionFreezeCommitSha[] =
    "a9d841571661d62c7d64b2eeb4eb959a3664ed42";
constexpr uint64_t kAlignBudget = 240;
constexpr double kReplayTranslationToleranceM = 0.002;
constexpr double kReplayRotationToleranceDeg = 0.02;
constexpr double kOperationalTranslationReferenceM = 0.20;
constexpr double kOperationalRotationReferenceDeg = 2.0;

struct EndpointInput {
  std::string label;
  Pose original_seed = Pose::Identity();
  Pose saved_terminal = Pose::Identity();
  double saved_objective = std::numeric_limits<double>::quiet_NaN();
  double saved_fitness = std::numeric_limits<double>::quiet_NaN();
  int saved_iterations = -1;
  bool saved_converged = false;
};

struct SelectedCase {
  std::string frame_id;
  uint64_t transaction_id = 0;
  std::string group;
  std::string ray_type;
  int ray_id = -1;
  int sign = 0;
  double alpha_inside = 0.0;
  double alpha_outside = 0.0;
  double jump_t_m = 0.0;
  double jump_r_deg = 0.0;
  EndpointInput inside;
  EndpointInput outside;
};

struct EndpointAuditResult {
  uint64_t transaction_id = 0;
  std::string endpoint_label;
  Pose original_seed = Pose::Identity();
  Pose saved_terminal = Pose::Identity();
  Pose replay_terminal = Pose::Identity();
  Pose refined_terminal_80 = Pose::Identity();
  Pose refined_terminal_160 = Pose::Identity();
  bool replay_converged = false;
  bool refined_converged_80 = false;
  bool refined_converged = false;
  int replay_iterations = 0;
  int refined_iterations_80 = 0;
  int refined_iterations = 0;
  double objective_replay = std::numeric_limits<double>::quiet_NaN();
  double objective_refined_80 = std::numeric_limits<double>::quiet_NaN();
  double objective_refined = std::numeric_limits<double>::quiet_NaN();
  double fitness_replay = std::numeric_limits<double>::quiet_NaN();
  double fitness_refined_80 = std::numeric_limits<double>::quiet_NaN();
  double fitness_refined = std::numeric_limits<double>::quiet_NaN();
  double replay_translation_error_m = std::numeric_limits<double>::quiet_NaN();
  double replay_rotation_error_deg = std::numeric_limits<double>::quiet_NaN();
  double replay_objective_error = std::numeric_limits<double>::quiet_NaN();
  double replay_objective_tolerance = std::numeric_limits<double>::quiet_NaN();
};

struct AlignResult {
  Pose terminal = Pose::Identity();
  bool converged = false;
  int iterations = 0;
  double objective = std::numeric_limits<double>::quiet_NaN();
  double fitness = std::numeric_limits<double>::quiet_NaN();
  double runtime_ms = 0.0;
};

struct CloudContext {
  ScanAsset asset;
  Cloud::Ptr source;
};

struct DerivativeNdt : public ::AuditedNdt {
  double derivatives(Cloud& transformed, Eigen::Matrix<double, 6, 1>& p,
                     Eigen::Matrix<double, 6, 1>& gradient,
                     Eigen::Matrix<double, 6, 6>& hessian) {
    return this->computeDerivatives(gradient, hessian, transformed, p, true);
  }
};

std::string csvQuote(const std::string& input) {
  if (input.find_first_of(",\"\r\n") == std::string::npos) return input;
  std::string quoted = "\"";
  for (char ch : input) {
    if (ch == '"') quoted += '"';
    quoted += ch;
  }
  quoted += '"';
  return quoted;
}

std::string vecCsv(const Eigen::VectorXd& vector) {
  std::ostringstream out;
  out << std::setprecision(17);
  for (Eigen::Index i = 0; i < vector.size(); ++i) {
    if (i) out << ';';
    out << vector(i);
  }
  return out.str();
}

template <typename Derived>
std::string matrixCsv(const Eigen::MatrixBase<Derived>& matrix) {
  Eigen::VectorXd flattened(matrix.size());
  Eigen::Index index = 0;
  for (Eigen::Index row = 0; row < matrix.rows(); ++row)
    for (Eigen::Index col = 0; col < matrix.cols(); ++col)
      flattened(index++) = matrix(row, col);
  return vecCsv(flattened);
}

double translationDistance(const Pose& a, const Pose& b) {
  return p6_i5c::translationDistance(a, b);
}

double rotationDistanceDeg(const Pose& a, const Pose& b) {
  return p6_i5c::rotationDistanceRad(a.block<3, 3>(0, 0), b.block<3, 3>(0, 0)) *
         180.0 / M_PI;
}

std::string pose7(const Pose& pose) { return p6_i5c::serializePose7(pose); }

std::string joinPoseFields(const CsvTable& table, const std::vector<std::string>& row,
                           const std::string& prefix) {
  std::ostringstream out;
  for (const char* field : {"x", "y", "z", "qx", "qy", "qz", "qw"}) {
    if (out.tellp() > 0) out << ';';
    out << table.get(row, prefix + "_" + field);
  }
  return out.str();
}

double parseDouble(const CsvTable& table, const std::vector<std::string>& row,
                   const std::string& field) {
  return parseCsvDouble(table, row, field);
}

int parseInt(const CsvTable& table, const std::vector<std::string>& row,
             const std::string& field) {
  return std::stoi(table.get(row, field));
}

bool parseBool(const CsvTable& table, const std::vector<std::string>& row,
               const std::string& field) {
  const std::string value = table.get(row, field);
  if (value == "true" || value == "1") return true;
  if (value == "false" || value == "0") return false;
  throw std::runtime_error("invalid_selected_bool:" + field);
}

EndpointInput endpointFromRow(const CsvTable& table,
                              const std::vector<std::string>& row,
                              const std::string& side) {
  EndpointInput result;
  result.label = side;
  result.original_seed = p6_i5c::parsePose7(joinPoseFields(table, row, "seed_" + side));
  result.saved_terminal = p6_i5c::parsePose7(joinPoseFields(table, row, "terminal_" + side));
  result.saved_objective = parseDouble(table, row, "original_objective_" + side);
  result.saved_fitness = parseDouble(table, row, "original_fitness_" + side);
  result.saved_iterations = parseInt(table, row, "original_iterations_" + side);
  result.saved_converged = parseBool(table, row, "original_convergence_" + side);
  return result;
}

std::vector<SelectedCase> readSelected(const std::string& path) {
  const CsvTable table = readCsv(path);
  std::vector<SelectedCase> selected;
  std::vector<std::pair<std::string, uint64_t>> identities;
  for (const auto& row : table.rows) {
    SelectedCase item;
    item.frame_id = table.get(row, "frame_id");
    item.transaction_id = parseU64(table, row, "transaction_id");
    item.group = table.get(row, "selection_group");
    item.ray_type = table.get(row, "ray_type");
    item.ray_id = parseInt(table, row, "ray_id");
    item.sign = parseInt(table, row, "sign");
    item.alpha_inside = parseDouble(table, row, "alpha_inside");
    item.alpha_outside = parseDouble(table, row, "alpha_outside");
    item.jump_t_m = parseDouble(table, row, "jump_t_m");
    item.jump_r_deg = parseDouble(table, row, "jump_r_deg");
    item.inside = endpointFromRow(table, row, "inside");
    item.outside = endpointFromRow(table, row, "outside");
    if (table.get(row, "source_endpoint_integrity") != "EXACT_LOGICAL_KEY_AND_POSE_MATCH")
      throw std::runtime_error("selected_endpoint_source_gate_failed");
    identities.emplace_back(item.frame_id, item.transaction_id);
    selected.push_back(item);
  }
  p6_i5c::validateSelectionIdentity(identities);
  return selected;
}

void requireOmpSingleThread() {
  const char* value = std::getenv("OMP_NUM_THREADS");
  if (!value || std::string(value) != "1")
    throw std::runtime_error("OMP_NUM_THREADS_must_equal_1");
}

void verifyFrozenFiles(const std::string& scans_path, const std::string& packed_path,
                       const std::string& map_path) {
  const std::string scans_sha = p5_i1::sha256File(scans_path);
  const std::string packed_sha = p5_i1::sha256File(packed_path);
  if (scans_sha != kExpectedScansSha)
    throw std::runtime_error("scan_assets_sha_mismatch:" + scans_sha);
  if (packed_sha != kExpectedPackedSha)
    throw std::runtime_error("packed_source_sha_mismatch:" + packed_sha);
  p5_i1::requireFrozenMapSha256(map_path);
}

std::pair<std::string, std::string> verifyFrozenSelection(
    const std::string& selected_path, const std::string& manifest_path) {
  const std::string selected_sha = p5_i1::sha256File(selected_path);
  const std::string manifest_sha = p5_i1::sha256File(manifest_path);
  if (!p6_i5c::sha256Matches(selected_sha, kExpectedSelectedCasesSha))
    throw std::runtime_error("FROZEN_SELECTION_HASH_MISMATCH:expected=" +
        std::string(kExpectedSelectedCasesSha) + ":actual=" + selected_sha);
  if (!p6_i5c::sha256Matches(manifest_sha, kExpectedSelectionManifestSha))
    throw std::runtime_error("FROZEN_SELECTION_MANIFEST_HASH_MISMATCH:expected=" +
        std::string(kExpectedSelectionManifestSha) + ":actual=" + manifest_sha);
  return {selected_sha, manifest_sha};
}

std::map<uint64_t, CloudContext> loadCloudContexts(
    const std::vector<SelectedCase>& selected, const std::string& scans_path,
    const std::string& packed_path) {
  const auto assets = readScanAssets(scans_path);
  std::map<uint64_t, ScanAsset> by_tx;
  for (const auto& asset : assets) by_tx.emplace(asset.transaction_id, asset);
  std::map<uint64_t, CloudContext> contexts;
  for (const SelectedCase& item : selected) {
    const auto found = by_tx.find(item.transaction_id);
    if (found == by_tx.end()) throw std::runtime_error("selected_transaction_not_in_scans");
    Cloud::Ptr raw = loadRawCloudAt(packed_path, found->second);
    Cloud::Ptr source = ::preprocessSource(raw);
    const uint64_t source_hash = ::sourceCloudHash(source);
    if (source_hash != found->second.expected_source_hash)
      throw std::runtime_error("ndt_source_cloud_hash_mismatch_tx=" +
                               std::to_string(item.transaction_id));
    if (std::to_string(source_hash) !=
        std::to_string(static_cast<uint64_t>(found->second.expected_source_hash)))
      throw std::runtime_error("ndt_source_cloud_hash_serialization_mismatch");
    contexts.emplace(item.transaction_id, CloudContext{found->second, source});
    std::cout << "SOURCE_GATE_PASS,tx=" << item.transaction_id
              << ",raw_points=" << raw->size() << ",ndt_points=" << source->size()
              << ",ndt_source_hash=" << source_hash << '\n';
  }
  return contexts;
}

std::string readFirstLine(const std::string& path) {
  std::ifstream stream(path);
  std::string line;
  if (!stream || !std::getline(stream, line))
    throw std::runtime_error("cannot_read_existing_call_accounting");
  return line;
}

uint64_t startedCallCount(const std::string& path) {
  std::ifstream input(path);
  if (!input) return 0;
  std::string line;
  if (!std::getline(input, line)) return 0;
  uint64_t count = 0;
  while (std::getline(input, line)) {
    if (line.find(",START,") != std::string::npos) ++count;
  }
  return count;
}

class CallLedger {
 public:
  CallLedger(std::string path, std::string selected_sha, std::string manifest_sha)
      : path_(std::move(path)), selected_sha_(std::move(selected_sha)),
        manifest_sha_(std::move(manifest_sha)) {
    if (!std::ifstream(path_).good()) {
      std::ofstream output(path_);
      if (!output) throw std::runtime_error("cannot_create_call_accounting");
      output << "call_id,event,phase,frame_id,transaction_id,endpoint,operation,seed_label,"
                "iterations,converged,runtime_ms,selected_cases_sha256,"
                "selection_manifest_sha256\n";
    } else if (readFirstLine(path_) !=
        "call_id,event,phase,frame_id,transaction_id,endpoint,operation,seed_label,"
        "iterations,converged,runtime_ms,selected_cases_sha256,selection_manifest_sha256") {
      throw std::runtime_error("call_accounting_header_mismatch");
    }
  }

  void started(uint64_t call_id, const std::string& phase, const SelectedCase& item,
               const std::string& endpoint, const std::string& operation,
               const std::string& seed_label) {
    if (startedCallCount(path_) >= kAlignBudget)
      throw std::runtime_error("P6_I5C_NDT_ALIGN_BUDGET_240_EXHAUSTED");
    append(call_id, "START", phase, item, endpoint, operation, seed_label, "", "", "");
  }

  void completed(uint64_t call_id, const std::string& phase, const SelectedCase& item,
                 const std::string& endpoint, const std::string& operation,
                 const std::string& seed_label, int iterations, bool converged,
                 double runtime_ms) {
    append(call_id, "END", phase, item, endpoint, operation, seed_label,
           std::to_string(iterations), converged ? "true" : "false", number(runtime_ms));
  }

 private:
  static std::string number(double value) {
    std::ostringstream out;
    out << std::setprecision(17) << value;
    return out.str();
  }

  void append(uint64_t call_id, const std::string& event, const std::string& phase,
              const SelectedCase& item, const std::string& endpoint,
              const std::string& operation, const std::string& seed_label,
              const std::string& iterations, const std::string& converged,
              const std::string& runtime) {
    std::ofstream output(path_, std::ios::app);
    if (!output) throw std::runtime_error("cannot_append_call_accounting");
    output << call_id << ',' << event << ',' << phase << ',' << item.frame_id << ','
           << item.transaction_id << ',' << endpoint << ',' << operation << ','
           << seed_label << ',' << iterations << ',' << converged << ',' << runtime << ','
           << selected_sha_ << ',' << manifest_sha_ << '\n';
    output.flush();
    if (!output) throw std::runtime_error("call_accounting_write_failed");
  }
  std::string path_;
  std::string selected_sha_;
  std::string manifest_sha_;
};

void appendRow(const std::string& path, const std::string& header,
               const std::string& row) {
  const bool exists = std::ifstream(path).good();
  std::ofstream output(path, std::ios::app);
  if (!output) throw std::runtime_error("cannot_append_output:" + path);
  if (!exists) output << header << '\n';
  output << row << '\n';
}

std::string fixed(double value) {
  std::ostringstream out;
  out << std::setprecision(17) << value;
  return out.str();
}

AlignResult alignOnce(const Cloud::Ptr& target, const Cloud::Ptr& source,
                      const Pose& seed, int max_iterations, double epsilon,
                      const std::string& phase, const SelectedCase& item,
                      const std::string& endpoint, const std::string& operation,
                      const std::string& seed_label, CallLedger& ledger,
                      uint64_t* call_id, uint64_t* calls_used) {
  const uint64_t id = *call_id;
  ledger.started(id, phase, item, endpoint, operation, seed_label);
  ++(*call_id);
  ++(*calls_used);
  ::AuditedNdt ndt;
  ::configureNdt(ndt, target);
  ndt.setInputSource(source);
  ndt.setMaximumIterations(max_iterations);
  ndt.setTransformationEpsilon(epsilon);
  Cloud aligned;
  const auto begin = std::chrono::steady_clock::now();
  ndt.align(aligned, seed.cast<float>());
  const double runtime_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - begin).count();
  AlignResult result;
      result.terminal = poseFromPcl(ndt.getFinalTransformation());
  result.converged = ndt.hasConverged();
  result.iterations = ndt.getFinalNumIteration();
  result.fitness = ndt.getFitnessScore();
  result.objective = ::fixedPclScore(ndt, source, result.terminal.cast<float>());
  result.runtime_ms = runtime_ms;
  ledger.completed(id, phase, item, endpoint, operation, seed_label,
                   result.iterations, result.converged, result.runtime_ms);
  return result;
}

bool replayPass(const EndpointInput& endpoint, const AlignResult& replay,
                double* translation_error, double* rotation_error,
                double* objective_error, double* objective_tolerance) {
  *translation_error = translationDistance(replay.terminal, endpoint.saved_terminal);
  *rotation_error = rotationDistanceDeg(replay.terminal, endpoint.saved_terminal);
  *objective_error = std::abs(replay.objective - endpoint.saved_objective);
  *objective_tolerance = std::max(1e-3, 1e-4 * std::abs(endpoint.saved_objective));
  return replay.converged && *translation_error <= kReplayTranslationToleranceM &&
         *rotation_error <= kReplayRotationToleranceDeg &&
         *objective_error <= *objective_tolerance;
}

void recordReplay(const std::string& path, const std::string& phase,
                  const SelectedCase& item, const EndpointInput& endpoint,
                  const AlignResult& result, bool pass, double dt, double dr,
                  double ds, double score_tolerance) {
  appendRow(path,
      "phase,frame_id,transaction_id,endpoint,seed_map_T_lidar,saved_terminal_map_T_lidar,"
      "replay_terminal_map_T_lidar,replay_converged,replay_iterations,replay_objective,"
      "saved_objective,objective_abs_diff,objective_tolerance,replay_fitness,saved_fitness,"
      "translation_error_m,rotation_error_deg,runtime_ms,replay_status",
      phase + "," + item.frame_id + "," + std::to_string(item.transaction_id) + "," +
      endpoint.label + "," + pose7(endpoint.original_seed) + "," +
      pose7(endpoint.saved_terminal) + "," + pose7(result.terminal) + "," +
      (result.converged ? "true" : "false") + "," + std::to_string(result.iterations) + "," +
      fixed(result.objective) + "," + fixed(endpoint.saved_objective) + "," + fixed(ds) + "," +
      fixed(score_tolerance) + "," + fixed(result.fitness) + "," + fixed(endpoint.saved_fitness) + "," +
      fixed(dt) + "," + fixed(dr) + "," + fixed(result.runtime_ms) + "," + (pass ? "PASS" : "REPLAY_INVALID"));
}

std::vector<const SelectedCase*> smokeCases(const std::vector<SelectedCase>& cases) {
  const SelectedCase* high = nullptr;
  const SelectedCase* low = nullptr;
  for (const auto& item : cases) {
    if (!high && item.group == "A_HIGH_JUMP") high = &item;
    if (!low && item.group == "B_LOW_JUMP_CONTROL") low = &item;
  }
  if (!high || !low) throw std::runtime_error("smoke_requires_A_and_B_cases");
  return {high, low};
}

void requireSuccessfulSmoke(const std::vector<SelectedCase>& selected,
                            const CsvTable& ledger_table,
                            const CsvTable& replay_table,
                            const std::string& selected_sha,
                            const std::string& manifest_sha) {
  using RecordKey = std::tuple<std::string, std::string, std::string>;
  std::set<RecordKey> expected_replays;
  for (const SelectedCase* item : smokeCases(selected)) {
    expected_replays.emplace(item->frame_id, std::to_string(item->transaction_id), "inside");
    expected_replays.emplace(item->frame_id, std::to_string(item->transaction_id), "outside");
  }

  std::set<RecordKey> actual_replays;
  for (const auto& row : replay_table.rows) {
    if (replay_table.get(row, "phase") != "SMOKE") continue;
    if (replay_table.get(row, "replay_status") != "PASS")
      throw std::runtime_error("formal_smoke_replay_contains_failure");
    const RecordKey key{replay_table.get(row, "frame_id"),
                        replay_table.get(row, "transaction_id"),
                        replay_table.get(row, "endpoint")};
    if (!actual_replays.insert(key).second)
      throw std::runtime_error("duplicate_smoke_replay_row");
  }
  if (actual_replays != expected_replays)
    throw std::runtime_error("formal_smoke_replay_identity_mismatch");

  std::map<std::string, RecordKey> starts;
  std::map<std::string, RecordKey> ends;
  std::size_t formal_starts = 0;
  for (const auto& row : ledger_table.rows) {
    const std::string phase = ledger_table.get(row, "phase");
    const std::string event = ledger_table.get(row, "event");
    if (phase == "FORMAL" && event == "START") ++formal_starts;
    if (phase != "SMOKE") continue;
    if (!p6_i5c::sameFrozenSelectionIdentity(
            ledger_table.get(row, "selected_cases_sha256"),
            ledger_table.get(row, "selection_manifest_sha256"),
            selected_sha, manifest_sha))
      throw std::runtime_error("smoke_formal_frozen_selection_hash_mismatch");
    const RecordKey key{ledger_table.get(row, "frame_id"),
                        ledger_table.get(row, "transaction_id"),
                        ledger_table.get(row, "endpoint")};
    const std::string call_id = ledger_table.get(row, "call_id");
    auto& destination = event == "START" ? starts : ends;
    if (event != "START" && event != "END")
      throw std::runtime_error("invalid_smoke_ledger_event");
    if (!destination.emplace(call_id, key).second)
      throw std::runtime_error("duplicate_smoke_ledger_event");
  }
  if (starts.size() != 4 || ends.size() != 4 || starts != ends)
    throw std::runtime_error("formal_smoke_ledger_incomplete_or_mismatched");
  if (formal_starts != 0)
    throw std::runtime_error("formal_already_attempted_refusing_duplicate_experiment");
}

std::map<uint64_t, std::array<EndpointAuditResult, 2>> runReplayPhase(
    const std::vector<const SelectedCase*>& cases, const std::map<uint64_t, CloudContext>& clouds,
    const Cloud::Ptr& target, const std::string& phase, const std::string& output_dir,
    CallLedger& ledger, uint64_t* next_call, uint64_t* calls_used) {
  std::map<uint64_t, std::array<EndpointAuditResult, 2>> results;
  const std::string replay_path = output_dir + "/replay_parity.csv";
  for (const SelectedCase* item : cases) {
    const auto cloud = clouds.find(item->transaction_id);
    if (cloud == clouds.end()) throw std::runtime_error("source_context_missing_for_replay");
    std::array<EndpointAuditResult, 2> pair;
    const std::array<const EndpointInput*, 2> endpoints = {&item->inside, &item->outside};
    for (std::size_t index = 0; index < endpoints.size(); ++index) {
      const EndpointInput& endpoint = *endpoints[index];
      const AlignResult aligned = alignOnce(target, cloud->second.source,
          endpoint.original_seed, 40, 0.001, phase, *item, endpoint.label,
          "ORIGINAL_REPLAY", "ORIGINAL_SEED", ledger, next_call, calls_used);
      double dt, dr, ds, score_tol;
      const bool pass = replayPass(endpoint, aligned, &dt, &dr, &ds, &score_tol);
      recordReplay(replay_path, phase, *item, endpoint, aligned, pass, dt, dr, ds, score_tol);
      EndpointAuditResult& audit = pair[index];
      audit.transaction_id = item->transaction_id;
      audit.endpoint_label = endpoint.label;
      audit.original_seed = endpoint.original_seed;
      audit.saved_terminal = endpoint.saved_terminal;
      audit.replay_terminal = aligned.terminal;
      audit.replay_converged = aligned.converged;
      audit.replay_iterations = aligned.iterations;
      audit.objective_replay = aligned.objective;
      audit.fitness_replay = aligned.fitness;
      audit.replay_translation_error_m = dt;
      audit.replay_rotation_error_deg = dr;
      audit.replay_objective_error = ds;
      audit.replay_objective_tolerance = score_tol;
      if (phase == "FORMAL") {
        // Formal run's replay is the P6-I5C gate; original seed lineage is frozen.
      }
      std::cout << "REPLAY_" << phase << ',' << item->frame_id << ',' << endpoint.label
                << ",status=" << (pass ? "PASS" : "REPLAY_INVALID")
                << ",dt=" << dt << ",dr_deg=" << dr << ",dscore=" << ds
                << ",tol=" << score_tol << '\n';
      if (phase == "SMOKE" && !pass)
        throw std::runtime_error("SMOKE_REPLAY_PARITY_FAILED:" + item->frame_id + ":" + endpoint.label);
    }
    results.emplace(item->transaction_id, pair);
  }
  return results;
}

double scoreAtP(DerivativeNdt& ndt, const Cloud::Ptr& source,
                const Eigen::Matrix<double, 6, 1>& p,
                Eigen::Matrix<double, 6, 1>* gradient = nullptr,
                Eigen::Matrix<double, 6, 6>* hessian = nullptr) {
  const Eigen::Matrix4f pose = p6_i5c::poseFromPclVector(p);
  Cloud transformed;
  pcl::transformPointCloud(*source, transformed, pose);
  Eigen::Matrix<double, 6, 1> parameter = p;
  Eigen::Matrix<double, 6, 1> local_gradient;
  Eigen::Matrix<double, 6, 6> local_hessian;
  const double score = ndt.derivatives(transformed, parameter, local_gradient, local_hessian);
  if (gradient) *gradient = local_gradient;
  if (hessian) *hessian = local_hessian;
  return score;
}

std::string relativeError(double numerator, double denominator) {
  return fixed(numerator / std::max(denominator, 1e-15));
}

struct StationarityResult {
  bool finite = false;
  bool repeatable = false;
  bool roundtrip_pass = false;
  bool gradient_fd_consistent = false;
  bool small_gradient = false;
  bool hessian_maximum_compatible = false;
  bool no_positive_neighbor = false;
  bool determinate = false;
  std::size_t neighbor_finite_count = 0;
  std::size_t neighbor_nonfinite_count = 0;
  std::size_t center_repeat_finite_count = 0;
  std::size_t center_repeat_nonfinite_count = 0;
  std::string nonfinite_neighbor_locations;
  std::string nonfinite_center_repeat_locations;
  std::string neighbor_audit_status = "INDETERMINATE";
  double objective = std::numeric_limits<double>::quiet_NaN();
  double repeat_range = std::numeric_limits<double>::quiet_NaN();
  double gradient_scaled_norm = std::numeric_limits<double>::quiet_NaN();
  double hessian_max_eigenvalue = std::numeric_limits<double>::quiet_NaN();
  double hessian_spectral_norm = std::numeric_limits<double>::quiet_NaN();
  std::string gradient_fd_status;
};

StationarityResult auditStationarity(const SelectedCase& item,
                                     const EndpointAuditResult& endpoint,
                                     const Cloud::Ptr& target,
                                     const Cloud::Ptr& source,
                                     const std::string& output_dir) {
  StationarityResult result;
  DerivativeNdt ndt;
  ::configureNdt(ndt, target);
  ndt.setInputSource(source);
  const Eigen::Matrix4f refined = endpoint.refined_terminal_160.cast<float>();
  const Eigen::Matrix<double, 6, 1> p = p6_i5c::pclVector(refined);
  const Eigen::Matrix4f roundtrip = p6_i5c::poseFromPclVector(p);
  const double rt = (roundtrip.block<3, 1>(0, 3) - refined.block<3, 1>(0, 3)).norm();
  const double rr = p6_i5c::rotationDistanceRad(
      roundtrip.block<3, 3>(0, 0).cast<double>(),
      refined.block<3, 3>(0, 0).cast<double>()) * 180.0 / M_PI;
  result.roundtrip_pass = rt <= 1e-5 && rr <= 1e-4;

  Eigen::Matrix<double, 6, 1> analytic_gradient;
  Eigen::Matrix<double, 6, 6> hessian;
  const double center_score = scoreAtP(ndt, source, p, &analytic_gradient, &hessian);
  result.objective = center_score;
  std::array<double, 3> repeats{};
  repeats[0] = center_score;
  repeats[1] = scoreAtP(ndt, source, p);
  repeats[2] = scoreAtP(ndt, source, p);
  const p6_i5c::RepeatedScoreAudit repeat_audit = p6_i5c::auditRepeatedScores(repeats);
  result.center_repeat_finite_count = repeat_audit.finite_count;
  result.center_repeat_nonfinite_count = repeat_audit.nonfinite_count;
  for (std::size_t index = 0; index < repeat_audit.nonfinite_locations.size(); ++index) {
    if (index) result.nonfinite_center_repeat_locations += ";";
    result.nonfinite_center_repeat_locations += repeat_audit.nonfinite_locations[index];
  }
  result.repeat_range = repeat_audit.range;
  const double center_scale = std::isfinite(center_score) ? std::max(1.0, std::abs(center_score)) : 1.0;
  const double repeat_tolerance = std::max(1e-8, 1e-10 * center_scale);
  result.repeatable = repeat_audit.determinate() && result.repeat_range <= repeat_tolerance;

  const Eigen::Matrix<double, 6, 6> symmetric_hessian = 0.5 * (hessian + hessian.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigen(symmetric_hessian);
  if (eigen.info() == Eigen::Success && hessian.allFinite()) {
    result.hessian_max_eigenvalue = eigen.eigenvalues().maxCoeff();
    result.hessian_spectral_norm = p6_i5c::symmetricSpectralNorm(symmetric_hessian);
    result.hessian_maximum_compatible = result.hessian_max_eigenvalue <=
        1e-8 * std::max(1.0, result.hessian_spectral_norm);
  }

  Eigen::Matrix<double, 6, 1> local_scale;
  local_scale << 0.005, 0.005, 0.005, 0.1 * M_PI / 180.0,
                 0.1 * M_PI / 180.0, 0.1 * M_PI / 180.0;
  result.gradient_scaled_norm = (analytic_gradient.array() * local_scale.array()).matrix().norm();
  result.small_gradient = result.gradient_scaled_norm <= 1e-5 * std::max(1.0, std::abs(center_score));
  result.finite = analytic_gradient.allFinite() && hessian.allFinite() &&
                  repeat_audit.determinate() && eigen.info() == Eigen::Success;

  std::ofstream fd(output_dir + "/gradient_fd_audit.csv", std::ios::app);
  if (!fd) throw std::runtime_error("cannot_append_gradient_fd_audit");
  if (fd.tellp() == 0)
    fd << "frame_id,transaction_id,endpoint,component,pcl_parameter,analytic_gradient,"
          "fd_gradient_h,fd_gradient_half_h,scaled_rel_disagreement_h,"
          "scaled_rel_disagreement_half_h,scaled_rel_step_convergence,cosine_h,cosine_half_h\n";

  const std::array<double, 6> base_steps = {1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4};
  const char* names[6] = {"x", "y", "z", "angle_x", "angle_y", "angle_z"};
  bool gradient_pass = result.finite;
  double max_disagreement = 0.0;
  for (int axis = 0; axis < 6; ++axis) {
    const double h = base_steps[axis];
    Eigen::Matrix<double, 6, 1> plus = p, minus = p;
    plus(axis) += h; minus(axis) -= h;
    const double fd_h = (scoreAtP(ndt, source, plus) - scoreAtP(ndt, source, minus)) / (2.0 * h);
    plus = p; minus = p;
    plus(axis) += 0.5 * h; minus(axis) -= 0.5 * h;
    const double fd_half = (scoreAtP(ndt, source, plus) - scoreAtP(ndt, source, minus)) / h;
    Eigen::Matrix<double, 6, 1> ga = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> gh = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> gh2 = Eigen::Matrix<double, 6, 1>::Zero();
    ga(axis) = analytic_gradient(axis); gh(axis) = fd_h; gh2(axis) = fd_half;
    const double denom_h = std::max({(ga.array() * local_scale.array()).matrix().norm(),
                                     (gh.array() * local_scale.array()).matrix().norm(), 1e-10});
    const double denom_h2 = std::max({(ga.array() * local_scale.array()).matrix().norm(),
                                      (gh2.array() * local_scale.array()).matrix().norm(), 1e-10});
    const double disagreement_h = ((ga - gh).array() * local_scale.array()).matrix().norm() / denom_h;
    const double disagreement_h2 = ((ga - gh2).array() * local_scale.array()).matrix().norm() / denom_h2;
    const double denominator_steps = std::max({(gh.array() * local_scale.array()).matrix().norm(),
        (gh2.array() * local_scale.array()).matrix().norm(), 1e-10});
    const double step_convergence = ((gh - gh2).array() * local_scale.array()).matrix().norm() /
                                    denominator_steps;
    const double cosine_h = analytic_gradient(axis) * fd_h >= 0.0 ? 1.0 : -1.0;
    const double cosine_h2 = analytic_gradient(axis) * fd_half >= 0.0 ? 1.0 : -1.0;
    max_disagreement = std::max({max_disagreement, disagreement_h,
                                 disagreement_h2, step_convergence});
    gradient_pass = gradient_pass && disagreement_h <= 0.10 &&
                    disagreement_h2 <= 0.10 && step_convergence <= 0.10 &&
                    cosine_h >= 0.99 && cosine_h2 >= 0.99;
    fd << item.frame_id << ',' << item.transaction_id << ',' << endpoint.endpoint_label << ','
       << names[axis] << ',' << fixed(p(axis)) << ',' << fixed(analytic_gradient(axis)) << ','
       << fixed(fd_h) << ',' << fixed(fd_half) << ',' << fixed(disagreement_h) << ','
       << fixed(disagreement_h2) << ',' << fixed(step_convergence) << ','
       << fixed(cosine_h) << ',' << fixed(cosine_h2) << '\n';
  }
  result.gradient_fd_consistent = gradient_pass;
  result.gradient_fd_status = gradient_pass ? "PASS" : "INDETERMINATE";

  static const char* kAxisNames[6] = {"x", "y", "z", "angle_x", "angle_y", "angle_z"};
  static const double kNeighborStep[6] = {0.005, 0.005, 0.005,
      0.1 * M_PI / 180.0, 0.1 * M_PI / 180.0, 0.1 * M_PI / 180.0};
  struct NeighborObjective {
    int axis;
    int sign;
    double step;
    double score;
    double delta;
  };
  std::vector<NeighborObjective> neighbors;
  neighbors.reserve(12);
  p6_i5c::NeighborScoreAudit neighbor_audit;
  for (int axis = 0; axis < 6; ++axis) {
    for (int sign : {-1, 1}) {
      Eigen::Matrix<double, 6, 1> neighbor = p;
      neighbor(axis) += sign * kNeighborStep[axis];
      const double neighbor_score = scoreAtP(ndt, source, neighbor);
      const double delta_score = neighbor_score - center_score;
      const std::string location = std::string(kAxisNames[axis]) +
          (sign > 0 ? "+" : "-");
      neighbor_audit.observe(location, center_score, neighbor_score, repeat_tolerance);
      neighbors.push_back({axis, sign, kNeighborStep[axis], neighbor_score, delta_score});
    }
  }
  result.neighbor_finite_count = neighbor_audit.finite_count;
  result.neighbor_nonfinite_count = neighbor_audit.nonfinite_count;
  for (std::size_t index = 0; index < neighbor_audit.nonfinite_locations.size(); ++index) {
    if (index) result.nonfinite_neighbor_locations += ";";
    result.nonfinite_neighbor_locations += neighbor_audit.nonfinite_locations[index];
  }
  result.no_positive_neighbor = result.repeatable && neighbor_audit.determinate(12) &&
                                neighbor_audit.no_positive_increase;
  result.neighbor_audit_status = !result.repeatable ||
      !neighbor_audit.determinate(12) ? "INDETERMINATE" :
      (neighbor_audit.no_positive_increase ? "NO_POSITIVE_NEIGHBOR" :
                                             "POSITIVE_NEIGHBOR_FOUND");
  result.determinate = result.finite && result.repeatable && result.roundtrip_pass &&
                       result.gradient_fd_consistent && neighbor_audit.determinate(12);

  std::ofstream objective(output_dir + "/objective_stationarity.csv", std::ios::app);
  if (!objective) throw std::runtime_error("cannot_append_objective_stationarity");
  if (objective.tellp() == 0)
    objective << "frame_id,transaction_id,endpoint,record_type,axis,sign,step,objective_value,"
                 "delta_score,repeat_min,repeat_max,repeat_range,repeat_tolerance,"
                 "pcl_parameter_xyz_angles,analytic_gradient_xyz_angles,hessian_eigenvalues,"
                 "hessian_max_eigenvalue,hessian_spectral_norm,gradient_scaled_norm,"
                 "roundtrip_translation_m,roundtrip_rotation_deg,repeatability_status,"
                 "gradient_fd_status,hessian_maximum_compatible,neighbor_score_finite_count,"
                 "neighbor_score_nonfinite_count,nonfinite_neighbor_locations,evaluation_finite,"
                 "neighbor_audit_status,center_score_repeat_finite_count,"
                 "center_score_repeat_nonfinite_count,nonfinite_center_score_repeats\n";
  Eigen::VectorXd p_dynamic = p;
  Eigen::VectorXd gradient_dynamic = analytic_gradient;
  Eigen::VectorXd eigenvalues(6);
  if (eigen.info() == Eigen::Success) eigenvalues = eigen.eigenvalues();
  const double repeat_min = repeat_audit.minimum;
  const double repeat_max = repeat_audit.maximum;
  objective << item.frame_id << ',' << item.transaction_id << ',' << endpoint.endpoint_label
      << ",CENTER,,0,0," << fixed(center_score) << ",0," << fixed(repeat_min) << ','
      << fixed(repeat_max) << ',' << fixed(result.repeat_range) << ',' << fixed(repeat_tolerance)
      << ',' << vecCsv(p_dynamic) << ',' << vecCsv(gradient_dynamic) << ','
      << vecCsv(eigenvalues) << ',' << fixed(result.hessian_max_eigenvalue) << ','
      << fixed(result.hessian_spectral_norm) << ',' << fixed(result.gradient_scaled_norm) << ','
      << fixed(rt) << ',' << fixed(rr) << ',' << (result.repeatable ? "PASS" :
          (repeat_audit.determinate() ? "FAIL" : "INDETERMINATE")) << ','
      << result.gradient_fd_status << ',' << (result.hessian_maximum_compatible ? "true" : "false")
      << ',' << result.neighbor_finite_count << ',' << result.neighbor_nonfinite_count << ','
      << csvQuote(result.nonfinite_neighbor_locations) << ','
      << (result.finite ? "true" : "false") << ','
      << result.neighbor_audit_status << ',' << result.center_repeat_finite_count << ','
      << result.center_repeat_nonfinite_count << ','
      << csvQuote(result.nonfinite_center_repeat_locations) << '\n';

  for (const NeighborObjective& neighbor : neighbors) {
    objective << item.frame_id << ',' << item.transaction_id << ',' << endpoint.endpoint_label
        << ",NEIGHBOR," << kAxisNames[neighbor.axis] << ',' << neighbor.sign << ','
        << fixed(neighbor.step) << ',' << fixed(neighbor.score) << ','
        << fixed(neighbor.delta) << ",,,,," << vecCsv(p_dynamic) << ","
        << vecCsv(gradient_dynamic) << ',' << vecCsv(eigenvalues) << ','
        << fixed(result.hessian_max_eigenvalue) << ',' << fixed(result.hessian_spectral_norm) << ','
        << fixed(result.gradient_scaled_norm) << ',' << fixed(rt) << ',' << fixed(rr)
        << ",,,," << result.neighbor_finite_count << ','
        << result.neighbor_nonfinite_count << ',' << csvQuote(result.nonfinite_neighbor_locations)
        << ',' << (repeat_audit.determinate() && std::isfinite(neighbor.score) &&
                   std::isfinite(neighbor.delta) ? "true" : "false")
        << ',' << result.neighbor_audit_status << ',' << result.center_repeat_finite_count << ','
        << result.center_repeat_nonfinite_count << ','
        << csvQuote(result.nonfinite_center_repeat_locations) << '\n';
  }
  return result;
}

Pose perturbMap(const Pose& nominal, int axis, int sign) {
  Eigen::Vector3d dp = Eigen::Vector3d::Zero();
  Eigen::Vector3d dphi = Eigen::Vector3d::Zero();
  const double translation_delta = 0.005;
  const double rotation_delta = 0.1 * M_PI / 180.0;
  if (axis < 3) dp(axis) = sign * translation_delta;
  else dphi(axis - 3) = sign * rotation_delta;
  return p6_i5c::leftPerturbMap(nominal, dp, dphi);
}

void writeRefinement(const std::string& path, const SelectedCase& item,
                     const EndpointAuditResult& endpoint, int stage,
                     const AlignResult& aligned) {
  appendRow(path,
      "frame_id,transaction_id,endpoint,stage,max_iterations,epsilon,input_seed_map_T_lidar,"
      "terminal_map_T_lidar,converged,iterations,objective_fixed_pcl_score,fitness_score,runtime_ms",
      item.frame_id + "," + std::to_string(item.transaction_id) + "," + endpoint.endpoint_label + "," +
      std::to_string(stage) + "," + (stage == 80 ? "80" : "160") + "," +
      (stage == 80 ? "1e-5" : "1e-6") + "," +
      pose7(stage == 80 ? endpoint.saved_terminal : endpoint.refined_terminal_80) + "," +
      pose7(aligned.terminal) + "," + (aligned.converged ? "true" : "false") + "," +
      std::to_string(aligned.iterations) + "," + fixed(aligned.objective) + "," +
      fixed(aligned.fitness) + "," + fixed(aligned.runtime_ms));
}

void runFormal(const std::vector<SelectedCase>& selected,
               const std::map<uint64_t, CloudContext>& clouds,
               const std::map<uint64_t, std::array<EndpointAuditResult, 2>>& replay,
               const Cloud::Ptr& target, const std::string& output_dir,
               CallLedger& ledger, uint64_t* next_call, uint64_t* calls_used) {
  const std::string refinement_path = output_dir + "/refinement_results.csv";
  const std::string separation_path = output_dir + "/terminal_separation.csv";
  const std::string perturb_path = output_dir + "/local_perturbation_results.csv";
  const std::string verdict_path = output_dir + "/per_case_verdict.csv";
  const std::string stationarity_path = output_dir + "/objective_stationarity.csv";
  (void)stationarity_path;
  for (const SelectedCase& item : selected) {
    const auto replay_it = replay.find(item.transaction_id);
    if (replay_it == replay.end()) throw std::runtime_error("formal_replay_record_missing");
    auto pair = replay_it->second;
    const bool replay_ok = pair[0].replay_converged && pair[1].replay_converged &&
        pair[0].replay_translation_error_m <= kReplayTranslationToleranceM &&
        pair[1].replay_translation_error_m <= kReplayTranslationToleranceM &&
        pair[0].replay_rotation_error_deg <= kReplayRotationToleranceDeg &&
        pair[1].replay_rotation_error_deg <= kReplayRotationToleranceDeg &&
        pair[0].replay_objective_error <= pair[0].replay_objective_tolerance &&
        pair[1].replay_objective_error <= pair[1].replay_objective_tolerance;
    if (!replay_ok) {
      appendRow(verdict_path,
          "frame_id,transaction_id,selection_group,replay_status,refined_separation_t_m,"
          "refined_separation_r_deg,inside_own_return_count,outside_own_return_count,"
          "inside_cross_return_count,outside_cross_return_count,classification,reason",
          item.frame_id + "," + std::to_string(item.transaction_id) + "," + item.group +
          ",REPLAY_INVALID,,,,,,,REPLAY_INVALID,one_or_both_endpoints_failed_replay_gate");
      continue;
    }
    const auto cloud = clouds.find(item.transaction_id);
    if (cloud == clouds.end()) throw std::runtime_error("formal_source_missing");
    std::array<const EndpointInput*, 2> endpoint_input = {&item.inside, &item.outside};
    for (std::size_t side = 0; side < pair.size(); ++side) {
      EndpointAuditResult& endpoint = pair[side];
      const EndpointInput& input = *endpoint_input[side];
      AlignResult r80 = alignOnce(target, cloud->second.source, endpoint.saved_terminal,
          80, 1e-5, "FORMAL", item, endpoint.endpoint_label, "REFINEMENT_80",
          "SAVED_TERMINAL", ledger, next_call, calls_used);
      endpoint.refined_terminal_80 = r80.terminal;
      endpoint.refined_converged_80 = r80.converged;
      endpoint.refined_iterations_80 = r80.iterations;
      endpoint.objective_refined_80 = r80.objective;
      endpoint.fitness_refined_80 = r80.fitness;
      writeRefinement(refinement_path, item, endpoint, 80, r80);
      AlignResult r160 = alignOnce(target, cloud->second.source, r80.terminal,
          160, 1e-6, "FORMAL", item, endpoint.endpoint_label, "REFINEMENT_160",
          "REFINED_80_TERMINAL", ledger, next_call, calls_used);
      endpoint.refined_terminal_160 = r160.terminal;
      endpoint.refined_converged = r160.converged;
      endpoint.refined_iterations = r160.iterations;
      endpoint.objective_refined = r160.objective;
      endpoint.fitness_refined = r160.fitness;
      writeRefinement(refinement_path, item, endpoint, 160, r160);
      (void)input;
    }

    const double original_dt = translationDistance(item.inside.saved_terminal,
                                                    item.outside.saved_terminal);
    const double original_dr = rotationDistanceDeg(item.inside.saved_terminal,
                                                    item.outside.saved_terminal);
    const double replay_dt = translationDistance(pair[0].replay_terminal,
                                                  pair[1].replay_terminal);
    const double replay_dr = rotationDistanceDeg(pair[0].replay_terminal,
                                                  pair[1].replay_terminal);
    const double ref80_dt = translationDistance(pair[0].refined_terminal_80,
                                                 pair[1].refined_terminal_80);
    const double ref80_dr = rotationDistanceDeg(pair[0].refined_terminal_80,
                                                 pair[1].refined_terminal_80);
    const double ref160_dt = translationDistance(pair[0].refined_terminal_160,
                                                  pair[1].refined_terminal_160);
    const double ref160_dr = rotationDistanceDeg(pair[0].refined_terminal_160,
                                                  pair[1].refined_terminal_160);

    std::array<StationarityResult, 2> stationarity;
    for (std::size_t side = 0; side < pair.size(); ++side)
      stationarity[side] = auditStationarity(item, pair[side], target,
          cloud->second.source, output_dir);

    int own_returns[2] = {0, 0};
    int cross_returns[2] = {0, 0};
    const char* axes[6] = {"X", "Y", "Z", "RX", "RY", "RZ"};
    for (std::size_t side = 0; side < pair.size(); ++side) {
      for (int axis = 0; axis < 6; ++axis) {
        for (int sign : {-1, 1}) {
          const Pose seed = perturbMap(pair[side].refined_terminal_160, axis, sign);
          AlignResult result = alignOnce(target, cloud->second.source, seed,
              160, 1e-6, "FORMAL", item,
              pair[side].endpoint_label, "SMALL_PERTURBATION_160",
              std::string(axes[axis]) + (sign > 0 ? "+" : "-"),
              ledger, next_call, calls_used);
          const double own_dt = translationDistance(result.terminal,
                                                      pair[side].refined_terminal_160);
          const double own_dr = rotationDistanceDeg(result.terminal,
                                                      pair[side].refined_terminal_160);
          const std::size_t other = 1 - side;
          const double other_dt = translationDistance(result.terminal,
                                                       pair[other].refined_terminal_160);
          const double other_dr = rotationDistanceDeg(result.terminal,
                                                       pair[other].refined_terminal_160);
          const bool own = p6_i5c::terminalReturn(result.converged, own_dt, own_dr,
                                                   0.02, 0.2);
          const bool cross = p6_i5c::terminalReturn(result.converged, other_dt, other_dr,
                                                     0.02, 0.2);
          own_returns[side] += own ? 1 : 0;
          cross_returns[side] += cross ? 1 : 0;
          appendRow(perturb_path,
              "frame_id,transaction_id,source_side,axis,sign,seed_semantics,seed_map_T_lidar,"
              "result_map_T_lidar,converged,iterations,objective,fitness,runtime_ms,"
              "return_to_own,own_translation_error_m,own_rotation_error_deg,"
              "return_to_other,other_translation_error_m,other_rotation_error_deg",
              item.frame_id + "," + std::to_string(item.transaction_id) + "," +
              pair[side].endpoint_label + "," + axes[axis] + "," + std::to_string(sign) +
              ",MAP_FRAME_LEFT_ROTATION_AND_ADDITIVE_MAP_TRANSLATION," + pose7(seed) + "," +
              pose7(result.terminal) + "," + (result.converged ? "true" : "false") + "," +
              std::to_string(result.iterations) + "," + fixed(result.objective) + "," +
              fixed(result.fitness) + "," + fixed(result.runtime_ms) + "," + (own ? "true" : "false") +
              "," + fixed(own_dt) + "," + fixed(own_dr) + "," + (cross ? "true" : "false") +
              "," + fixed(other_dt) + "," + fixed(other_dr));
        }
      }
    }
    const bool collapsed = ref160_dt <= kOperationalTranslationReferenceM &&
                          ref160_dr <= kOperationalRotationReferenceDeg;
    const bool both_refined_converged = pair[0].refined_converged && pair[1].refined_converged;
    const bool stationarity_determinate = stationarity[0].determinate && stationarity[1].determinate;
    const bool stationarity_pass = both_refined_converged && stationarity_determinate &&
        stationarity[0].small_gradient && stationarity[1].small_gradient &&
        stationarity[0].hessian_maximum_compatible && stationarity[1].hessian_maximum_compatible &&
        stationarity[0].no_positive_neighbor && stationarity[1].no_positive_neighbor &&
        own_returns[0] == 12 && own_returns[1] == 12 &&
        cross_returns[0] == 0 && cross_returns[1] == 0;
    const std::string classification = collapsed ? "REFINED_COLLAPSE" :
        (!stationarity_determinate ? "INDETERMINATE" :
         (stationarity_pass ? "REFINED_DISTINCT_STATIONARITY_COMPATIBLE" :
                              "REFINED_DISTINCT_NOT_STATIONARY"));
    appendRow(separation_path,
        "frame_id,transaction_id,selection_group,original_delta_t_m,original_delta_r_deg,"
        "replay_delta_t_m,replay_delta_r_deg,refined80_delta_t_m,refined80_delta_r_deg,"
        "refined160_delta_t_m,refined160_delta_r_deg,translation_merged_by_reference,"
        "rotation_merged_by_reference,pairwise_merged_by_reference",
        item.frame_id + "," + std::to_string(item.transaction_id) + "," + item.group + "," +
        fixed(original_dt) + "," + fixed(original_dr) + "," + fixed(replay_dt) + "," +
        fixed(replay_dr) + "," + fixed(ref80_dt) + "," + fixed(ref80_dr) + "," +
        fixed(ref160_dt) + "," + fixed(ref160_dr) + "," +
        (ref160_dt <= kOperationalTranslationReferenceM ? "true" : "false") + "," +
        (ref160_dr <= kOperationalRotationReferenceDeg ? "true" : "false") + "," +
        (collapsed ? "true" : "false"));
    appendRow(verdict_path,
        "frame_id,transaction_id,selection_group,replay_status,refined_separation_t_m,"
        "refined_separation_r_deg,inside_own_return_count,outside_own_return_count,"
        "inside_cross_return_count,outside_cross_return_count,classification,reason",
        item.frame_id + "," + std::to_string(item.transaction_id) + "," + item.group +
        ",PASS," + fixed(ref160_dt) + "," + fixed(ref160_dr) + "," +
        std::to_string(own_returns[0]) + "," + std::to_string(own_returns[1]) + "," +
        std::to_string(cross_returns[0]) + "," + std::to_string(cross_returns[1]) + "," +
        classification + "," + (collapsed ? "both_reference_tolerances_met" :
            (stationarity_pass ? "all_frozen_stationarity_and_return_gates_pass" :
             (!stationarity_determinate ? "objective_or_gradient_audit_indeterminate" :
                                         "one_or_more_stationarity_or_return_gates_failed"))));
    std::cout << "CASE_RESULT," << item.frame_id << ',' << classification
              << ",delta_t=" << ref160_dt << ",delta_r_deg=" << ref160_dr
              << ",returns=" << own_returns[0] << "/12," << own_returns[1] << "/12"
              << ",cross=" << cross_returns[0] << ',' << cross_returns[1] << '\n';
  }
}

void run(const std::string& mode, const std::string& selected_path,
         const std::string& scans_path, const std::string& packed_path,
         const std::string& map_path, const std::string& output_dir) {
  std::filesystem::create_directories(output_dir);
  requireOmpSingleThread();
  const std::string selection_manifest_path = output_dir + "/selection_manifest.json";
  const auto selection_identity = verifyFrozenSelection(selected_path,
                                                         selection_manifest_path);
  verifyFrozenFiles(scans_path, packed_path, map_path);
  const std::vector<SelectedCase> selected = readSelected(selected_path);
  const std::map<uint64_t, CloudContext> clouds =
      loadCloudContexts(selected, scans_path, packed_path);
  const Cloud::Ptr target = ::loadTarget(map_path);
  if (target->size() != 549606) throw std::runtime_error("frozen_target_point_count_mismatch");
  const std::string ledger_path = output_dir + "/call_accounting.csv";
  CallLedger ledger(ledger_path, selection_identity.first, selection_identity.second);
  uint64_t call_id = startedCallCount(ledger_path) + 1;
  uint64_t calls_used = 0;
  if (mode == "smoke") {
    if (call_id != 1 || std::ifstream(output_dir + "/replay_parity.csv").good())
      throw std::runtime_error("smoke_already_attempted_or_output_not_fresh");
    const auto smoke = smokeCases(selected);
    runReplayPhase(smoke, clouds, target, "SMOKE", output_dir, ledger,
                   &call_id, &calls_used);
    std::cout << "SMOKE_PASS,align_calls=" << calls_used << '\n';
    return;
  }
  if (mode != "formal") throw std::runtime_error("mode_must_be_smoke_or_formal");
  const std::string replay_path = output_dir + "/replay_parity.csv";
  const CsvTable ledger_table = readCsv(ledger_path);
  const CsvTable replay_table = readCsv(replay_path);
  requireSuccessfulSmoke(selected, ledger_table, replay_table,
                         selection_identity.first, selection_identity.second);
  std::vector<const SelectedCase*> all;
  all.reserve(selected.size());
  for (const SelectedCase& item : selected) all.push_back(&item);
  const auto replay = runReplayPhase(all, clouds, target, "FORMAL", output_dir,
                                     ledger, &call_id, &calls_used);
  runFormal(selected, clouds, replay, target, output_dir, ledger,
            &call_id, &calls_used);
  const uint64_t total = startedCallCount(ledger_path);
  if (total > kAlignBudget) throw std::runtime_error("P6_I5C_NDT_ALIGN_BUDGET_EXCEEDED");
  std::cout << "FORMAL_COMPLETE,run_align_calls=" << calls_used
            << ",cumulative_align_calls=" << total << ",remaining_budget="
            << (kAlignBudget - total) << '\n';
}

}  // namespace p6_i5c_app

int main(int argc, char** argv) {
  try {
    if (argc != 7) {
      std::cerr << "usage: p6_i5c_stationary_audit smoke|formal SELECTED.csv SCANS.csv "
                   "XYZ.bin MAP.pcd OUT_DIR\n";
      return 2;
    }
    p6_i5c_app::run(argv[1], argv[2], argv[3], argv[4], argv[5], argv[6]);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "P6_I5C_ERROR: " << error.what() << '\n';
    return 1;
  }
}
