#pragma once

// Diagnostic-only observer for A3G-R3. This header is included from the P6
// producer translation unit, after its production types have been declared.

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

class A3gR3EvidenceCapture {
 public:
  explicit A3gR3EvidenceCapture(
      const std::string& root,
      std::set<std::uint64_t> selected = {159,160,163,165,166,173,174,
                                          176,182,183,185,187,188,190})
      : root_(root), selected_(std::move(selected)) {
    const std::filesystem::path root_path(root_);
    if (!std::filesystem::is_directory(root_path))
      throw std::runtime_error("A3G_R3_CAPTURE_ROOT_NOT_DIRECTORY");
    const auto states_path = root_path / "SELECTED_WINDOW_STATES.csv";
    if (std::filesystem::exists(states_path))
      throw std::runtime_error("A3G_R3_CAPTURE_OUTPUT_ALREADY_EXISTS:SELECTED_WINDOW_STATES.csv");
    states_.open(states_path, std::ios::out | std::ios::binary);
    if (!states_) throw std::runtime_error("A3G_R3_CAPTURE_STATES_OPEN_FAILED");
    states_ << std::setprecision(17)
        << "transaction_id,state_role,stamp_ns,px,py,pz,qx,qy,qz,qw,"
           "vx,vy,vz,bgx,bgy,bgz,bax,bay,baz\n";
    states_.flush();
    if (!states_) throw std::runtime_error("A3G_R3_CAPTURE_STATES_HEADER_FAILED");
  }

  bool selected(std::uint64_t transaction_id) const {
    return selected_.count(transaction_id) != 0;
  }

  void captureInputs(
      const fixed_lag::RawTimedScan& raw,
      const fixed_lag::WindowState& scan_start,
      const fixed_lag::WindowState& scan_end_pre_measurement,
      const fixed_lag::WindowDeskewResult& deskew,
      const Cloud& ndt_source,
      const Pose3d& T_imu_lidar,
      const Eigen::Matrix4d& ndt_initial,
      const fixed_lag::WindowSummary& pre_measurement_summary) {
    if (!selected(raw.transaction_id)) return;
    if (input_written_.count(raw.transaction_id))
      throw std::runtime_error("A3G_R3_CAPTURE_DUPLICATE_TRANSACTION");
    const auto dir = transactionDirectory(raw.transaction_id);
    std::error_code ec;
    if (!std::filesystem::create_directory(dir, ec) || ec)
      throw std::runtime_error("A3G_R3_CAPTURE_TRANSACTION_DIRECTORY_CREATE_FAILED");
    input_written_.insert(raw.transaction_id);
    appendState(raw.transaction_id, "SCAN_START", scan_start);
    appendState(raw.transaction_id, "SCAN_END_PRE_MEASUREMENT",
                scan_end_pre_measurement);
    writeRaw(raw, dir);
    writeDeskew(deskew, dir);
    writeKnots(deskew, dir);
    writeNdtSource(ndt_source, dir);
    writeInputMetadata(raw, scan_start, scan_end_pre_measurement, deskew,
                        ndt_source, T_imu_lidar, ndt_initial,
                        pre_measurement_summary, dir);
  }

  void captureTerminalMetadata(
      const fixed_lag::RawTimedScan& raw,
      const fixed_lag::WindowState& scan_start,
      const fixed_lag::WindowState& scan_end_pre_measurement,
      const fixed_lag::WindowDeskewResult& deskew,
      const Cloud& ndt_source,
      const Pose3d& T_imu_lidar,
      const Eigen::Matrix4d& ndt_initial,
      const Candidate& ndt_terminal,
      const reliability::LocalObservability& uobs,
      const reliability::LocalRisk& risk,
      const fixed_lag::FrozenLidarEvent& lidar,
      bool measurement_preview_valid, int selected_rank,
      bool nis_valid, double nis_value, double nis_threshold,
      bool nis_accepted, bool lidar_attempted, bool lidar_committed,
      const std::string& event_disposition, const std::string& event_reason,
      const fixed_lag::WindowSummary& pre_measurement_summary,
      const fixed_lag::WindowSummary& after_lidar_summary) {
    if (!selected(raw.transaction_id)) return;
    if (!input_written_.count(raw.transaction_id))
      throw std::runtime_error("A3G_R3_CAPTURE_METADATA_WITHOUT_INPUTS");
    const auto dir = transactionDirectory(raw.transaction_id);
    const auto path = dir / "METADATA.json";
    if (std::filesystem::exists(path))
      throw std::runtime_error("A3G_R3_CAPTURE_OUTPUT_ALREADY_EXISTS:METADATA.json");
    std::ofstream out(path, std::ios::out | std::ios::binary);
    if (!out) throw std::runtime_error("A3G_R3_CAPTURE_METADATA_OPEN_FAILED");
    std::uint64_t point_stamp_min = std::numeric_limits<std::uint64_t>::max();
    std::uint64_t point_stamp_max = 0;
    for (const auto& point : raw.points) {
      point_stamp_min = std::min(point_stamp_min, point.stamp_ns);
      point_stamp_max = std::max(point_stamp_max, point.stamp_ns);
    }
    out << std::setprecision(17) << "{\n"
        << "  \"transaction_id\": " << raw.transaction_id << ",\n"
        << "  \"scan_start_ns\": " << raw.scan_start_ns << ",\n"
        << "  \"scan_end_ns\": " << raw.scan_end_ns << ",\n"
        << "  \"raw_point_count\": " << raw.points.size() << ",\n"
        << "  \"raw_point_stamp_min_ns\": " << point_stamp_min << ",\n"
        << "  \"raw_point_stamp_max_ns\": " << point_stamp_max << ",\n"
        << "  \"deskewed_point_count\": " << deskew.cloud_end_frame.size() << ",\n"
        << "  \"ndt_source_point_count\": " << ndt_source.size() << ",\n"
        << "  \"lidar_provenance\": \"" << jsonEscape(toString(deskew.provenance)) << "\",\n"
        << "  \"raw_sensor_frame\": \"cmu_rc2_velodyne\",\n"
        << "  \"deskew_status\": \"" << jsonEscape(deskew.status) << "\",\n"
        << "  \"deskew_frame_convention\": \"" << jsonEscape(deskewFrameConvention()) << "\",\n"
        << "  \"deskew_reference_stamp_ns\": " << deskew.scan_end_ns << ",\n"
        << "  \"ndt_source_frame\": \"LIDAR_END_FRAME_SENSOR_LOCAL\",\n"
        << "  \"preprocess_source\": \"finite xyz; range [0.5,80.0] m; voxelDown leaf 0.25 m; max points 1400\",\n"
        << "  \"deskew_knots_status\": \""
        << (deskew.imu_trajectory.empty() ? "NOT_EXPLICITLY_MATERIALIZED" : "EXPLICITLY_MATERIALIZED") << "\",\n"
        << "  \"deskew_knot_count\": " << deskew.imu_trajectory.size() << ",\n"
        << "  \"scan_start_state_csv_role\": \"SCAN_START\",\n"
        << "  \"scan_end_state_csv_role\": \"SCAN_END_PRE_MEASUREMENT\",\n"
        << "  \"scan_start_state\": "; writeStateJson(out, scan_start); out << ",\n"
        << "  \"scan_end_pre_measurement_state\": "; writeStateJson(out, scan_end_pre_measurement); out << ",\n"
        << "  \"T_imu_lidar\": "; writePoseJson(out, T_imu_lidar); out << ",\n"
        << "  \"predicted_map_T_imu\": "; writeStatePoseJson(out, scan_end_pre_measurement); out << ",\n"
        << "  \"predicted_map_T_lidar\": "; writeMatrixJson(out, ndt_initial); out << ",\n"
        << "  \"ndt_initial_map_T_lidar\": "; writeMatrixJson(out, ndt_initial); out << ",\n"
        << "  \"ndt_terminal_map_T_lidar\": "; writeMatrixJson(out, ndt_terminal.pose); out << ",\n"
        << "  \"ndt_converged\": " << (ndt_terminal.converged ? "true" : "false") << ",\n"
        << "  \"ndt_iterations\": " << ndt_terminal.iterations << ",\n"
        << "  \"ndt_objective\": " << jsonNumber(ndt_terminal.objective) << ",\n"
        << "  \"ndt_fitness\": " << jsonNumber(ndt_terminal.fitness) << ",\n"
        << "  \"ndt_runtime_ms\": " << jsonNumber(ndt_terminal.runtime_ms) << ",\n"
        << "  \"uobs_valid\": " << (uobs.valid ? "true" : "false") << ",\n"
        << "  \"uobs_status\": \"" << jsonEscape(uobs.status) << "\",\n"
        << "  \"uobs_weak_dimension\": " << risk.weak_dimension << ",\n"
        << "  \"uobs_reliable_dimension\": " << risk.reliable_dimension << ",\n"
        << "  \"map_support_sufficient\": " << (risk.map_support_sufficient ? "true" : "false") << ",\n"
        << "  \"map_support_valid_for_measurement\": " << (lidar.map_support_valid ? "true" : "false") << ",\n"
        << "  \"map_support_status\": \"" << jsonEscape(risk.status) << "\",\n"
        << "  \"map_support_correspondences\": " << risk.map_support_correspondences << ",\n"
        << "  \"measurement_preview_valid\": " << (measurement_preview_valid ? "true" : "false") << ",\n"
        << "  \"selected_rank\": " << selected_rank << ",\n"
        << "  \"selected_nis_valid\": " << (nis_valid ? "true" : "false") << ",\n"
        << "  \"selected_nis\": " << jsonNumber(nis_value) << ",\n"
        << "  \"selected_nis_threshold\": " << jsonNumber(nis_threshold) << ",\n"
        << "  \"selected_nis_accepted\": " << (nis_accepted ? "true" : "false") << ",\n"
        << "  \"lidar_attempted\": " << (lidar_attempted ? "true" : "false") << ",\n"
        << "  \"lidar_committed\": " << (lidar_committed ? "true" : "false") << ",\n"
        << "  \"event_disposition\": \"" << jsonEscape(event_disposition) << "\",\n"
        << "  \"event_reason\": \"" << jsonEscape(event_reason) << "\",\n"
        << "  \"window_revision_pre_measurement\": " << pre_measurement_summary.window_revision << ",\n"
        << "  \"optimized_revision_pre_measurement\": " << pre_measurement_summary.optimized_revision << ",\n"
        << "  \"window_revision_after_lidar\": " << after_lidar_summary.window_revision << ",\n"
        << "  \"optimized_revision_after_lidar\": " << after_lidar_summary.optimized_revision << ",\n"
        << "  \"window_nodes_pre_measurement\": " << pre_measurement_summary.window_node_count << ",\n"
        << "  \"window_span_pre_measurement_s\": " << jsonNumber(pre_measurement_summary.window_time_span_s) << ",\n"
        << "  \"prior_rows_pre_measurement\": " << pre_measurement_summary.square_root_prior_rows << ",\n"
        << "  \"prior_columns_pre_measurement\": " << pre_measurement_summary.square_root_prior_columns << ",\n"
        << "  \"imu_factors_pre_measurement\": " << pre_measurement_summary.imu_factor_count << ",\n"
        << "  \"lidar_factors_pre_measurement\": " << pre_measurement_summary.lidar_factor_count << ",\n"
        << "  \"visual_factors_pre_measurement\": " << pre_measurement_summary.visual_factor_count << ",\n"
        << "  \"lidar_factors_after_lidar\": " << after_lidar_summary.lidar_factor_count << ",\n"
        << "  \"visual_factors_after_lidar\": " << after_lidar_summary.visual_factor_count << ",\n"
        << "  \"window_revision_captured_with_inputs\": " << input_revision_ << ",\n"
        << "  \"artifacts\": {\n"
        << "    \"raw_timed_points.bin\": "; writeFileIdentity(out, dir / "RAW_TIMED_POINTS.bin"); out << ",\n"
        << "    \"DESKEWED_END_FRAME.pcd\": "; writeFileIdentity(out, dir / "DESKEWED_END_FRAME.pcd"); out << ",\n"
        << "    \"NDT_SOURCE.pcd\": "; writeFileIdentity(out, dir / "NDT_SOURCE.pcd"); out << ",\n"
        << "    \"DESKEW_KNOTS.csv\": "; writeFileIdentity(out, dir / "DESKEW_KNOTS.csv"); out << "\n"
        << "  }\n}\n";
    out.flush();
    if (!out) throw std::runtime_error("A3G_R3_CAPTURE_METADATA_WRITE_FAILED");
    completed_.insert(raw.transaction_id);
  }

  std::size_t completedSelected() const { return completed_.size(); }

 private:
  std::filesystem::path transactionDirectory(std::uint64_t tx) const {
    std::ostringstream name;
    name << "TX" << std::setw(4) << std::setfill('0') << tx;
    return std::filesystem::path(root_) / name.str();
  }

  static std::string jsonEscape(const std::string& value) {
    std::string out;
    for (const char c : value) {
      if (c == '"' || c == '\\') { out.push_back('\\'); out.push_back(c); }
      else if (c == '\n') out += "\\n";
      else if (c == '\r') out += "\\r";
      else if (c == '\t') out += "\\t";
      else out.push_back(c);
    }
    return out;
  }

  static void writeLittleEndian64(std::ostream& out, std::uint64_t value) {
    for (unsigned shift = 0; shift < 64; shift += 8) {
      const char byte = static_cast<char>((value >> shift) & 0xffu);
      out.write(&byte, 1);
    }
  }

  static void writeFloat64(std::ostream& out, double value) {
    static_assert(sizeof(double) == sizeof(std::uint64_t), "unexpected double size");
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    writeLittleEndian64(out, bits);
  }

  static void writeFloat32(std::ostream& out, float value) {
    static_assert(sizeof(float) == sizeof(std::uint32_t), "unexpected float size");
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    for (unsigned shift = 0; shift < 32; shift += 8) {
      const char byte = static_cast<char>((bits >> shift) & 0xffu);
      out.write(&byte, 1);
    }
  }

  static std::string jsonNumber(double value) {
    if (std::isfinite(value)) {
      std::ostringstream out; out << std::setprecision(17) << value; return out.str();
    }
    return "null";
  }

  static void writeVectorJson(std::ostream& out, const Eigen::Vector3d& value) {
    out << '[' << jsonNumber(value.x()) << ',' << jsonNumber(value.y()) << ','
        << jsonNumber(value.z()) << ']';
  }

  static void writeStateJson(std::ostream& out, const fixed_lag::WindowState& state) {
    const Eigen::Quaterniond q(state.rotation);
    out << "{\"stamp_ns\":" << state.stamp_ns << ",\"position_xyz\":";
    writeVectorJson(out, state.position);
    out << ",\"rotation_xyzw\":[" << jsonNumber(q.x()) << ',' << jsonNumber(q.y()) << ','
        << jsonNumber(q.z()) << ',' << jsonNumber(q.w()) << "],\"velocity_xyz\":";
    writeVectorJson(out, state.velocity);
    out << ",\"gyro_bias_xyz\":"; writeVectorJson(out, state.gyro_bias);
    out << ",\"accel_bias_xyz\":"; writeVectorJson(out, state.accel_bias); out << '}';
  }

  static void writeStatePoseJson(std::ostream& out, const fixed_lag::WindowState& state) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.block<3,3>(0,0) = state.rotation;
    matrix.block<3,1>(0,3) = state.position;
    writeMatrixJson(out, matrix);
  }

  static void writePoseJson(std::ostream& out, const Pose3d& pose) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.block<3,3>(0,0) = pose.orientation.toRotationMatrix();
    matrix.block<3,1>(0,3) = pose.position;
    writeMatrixJson(out, matrix);
  }

  static void writeMatrixJson(std::ostream& out, const Eigen::Matrix4d& matrix) {
    out << '[';
    for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
      if (row) out << ',';
      out << '[';
      for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
        if (col) out << ',';
        out << jsonNumber(matrix(row,col));
      }
      out << ']';
    }
    out << ']';
  }

  static std::string deskewFrameConvention() {
    return "PRODUCTION_WINDOW_OWNED_OUTPUT; map_T_lidar_end^-1 * map_T_lidar(point_time) * p_sensor; reference=scan_end";
  }

  void appendState(std::uint64_t tx, const char* role,
                   const fixed_lag::WindowState& state) {
    const Eigen::Quaterniond q(state.rotation);
    states_ << std::setprecision(17) << tx << ',' << role << ',' << state.stamp_ns << ','
        << state.position.x() << ',' << state.position.y() << ',' << state.position.z() << ','
        << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ','
        << state.velocity.x() << ',' << state.velocity.y() << ',' << state.velocity.z() << ','
        << state.gyro_bias.x() << ',' << state.gyro_bias.y() << ',' << state.gyro_bias.z() << ','
        << state.accel_bias.x() << ',' << state.accel_bias.y() << ',' << state.accel_bias.z() << '\n';
    states_.flush();
    if (!states_) throw std::runtime_error("A3G_R3_CAPTURE_STATE_ROW_FAILED");
  }

  static void writeRaw(const fixed_lag::RawTimedScan& raw,
                       const std::filesystem::path& dir) {
    const auto path = dir / "RAW_TIMED_POINTS.bin";
    if (std::filesystem::exists(path))
      throw std::runtime_error("A3G_R3_CAPTURE_OUTPUT_ALREADY_EXISTS:RAW_TIMED_POINTS.bin");
    std::ofstream out(path, std::ios::binary | std::ios::out);
    if (!out) throw std::runtime_error("A3G_R3_CAPTURE_RAW_OPEN_FAILED");
    for (const auto& point : raw.points) {
      writeFloat64(out, point.position.x()); writeFloat64(out, point.position.y());
      writeFloat64(out, point.position.z()); writeFloat64(out, point.intensity);
      writeLittleEndian64(out, point.stamp_ns);
    }
    out.flush();
    if (!out) throw std::runtime_error("A3G_R3_CAPTURE_RAW_WRITE_FAILED");
  }

  static void writeDeskew(const fixed_lag::WindowDeskewResult& deskew,
                          const std::filesystem::path& dir) {
    const auto path = dir / "DESKEWED_END_FRAME.pcd";
    if (std::filesystem::exists(path))
      throw std::runtime_error("A3G_R3_CAPTURE_OUTPUT_ALREADY_EXISTS:DESKEWED_END_FRAME.pcd");
    std::ofstream out(path, std::ios::binary | std::ios::out);
    if (!out) throw std::runtime_error("A3G_R3_CAPTURE_DESKEW_PCD_OPEN_FAILED");
    out << "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
        << "FIELDS x y z intensity stamp_ns\nSIZE 8 8 8 8 8\n"
        << "TYPE F F F F U\nCOUNT 1 1 1 1 1\nWIDTH " << deskew.cloud_end_frame.size()
        << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS "
        << deskew.cloud_end_frame.size() << "\nDATA binary\n";
    for (const auto& point : deskew.cloud_end_frame) {
      writeFloat64(out, point.position.x()); writeFloat64(out, point.position.y());
      writeFloat64(out, point.position.z()); writeFloat64(out, point.intensity);
      writeLittleEndian64(out, point.stamp_ns);
    }
    out.flush();
    if (!out) throw std::runtime_error("A3G_R3_CAPTURE_DESKEW_PCD_WRITE_FAILED");
  }

  static void writeKnots(const fixed_lag::WindowDeskewResult& deskew,
                         const std::filesystem::path& dir) {
    const auto path = dir / "DESKEW_KNOTS.csv";
    if (std::filesystem::exists(path))
      throw std::runtime_error("A3G_R3_CAPTURE_OUTPUT_ALREADY_EXISTS:DESKEW_KNOTS.csv");
    std::ofstream out(path, std::ios::out | std::ios::binary);
    if (!out) throw std::runtime_error("A3G_R3_CAPTURE_KNOTS_OPEN_FAILED");
    out << std::setprecision(17)
        << "stamp_ns,r00,r01,r02,r10,r11,r12,r20,r21,r22,px,py,pz,vx,vy,vz,"
           "world_ax,world_ay,world_az,unbiased_gx,unbiased_gy,unbiased_gz\n";
    for (const auto& knot : deskew.imu_trajectory) {
      out << knot.stamp_ns;
      for (Eigen::Index row = 0; row < 3; ++row)
        for (Eigen::Index col = 0; col < 3; ++col) out << ',' << knot.rotation(row,col);
      out << ',' << knot.position.x() << ',' << knot.position.y() << ',' << knot.position.z()
          << ',' << knot.velocity.x() << ',' << knot.velocity.y() << ',' << knot.velocity.z()
          << ',' << knot.world_acceleration.x() << ',' << knot.world_acceleration.y() << ',' << knot.world_acceleration.z()
          << ',' << knot.unbiased_gyro.x() << ',' << knot.unbiased_gyro.y() << ',' << knot.unbiased_gyro.z() << '\n';
    }
    out.flush();
    if (!out) throw std::runtime_error("A3G_R3_CAPTURE_KNOTS_WRITE_FAILED");
  }

  static void writeNdtSource(const Cloud& cloud, const std::filesystem::path& dir) {
    const auto path = dir / "NDT_SOURCE.pcd";
    if (std::filesystem::exists(path))
      throw std::runtime_error("A3G_R3_CAPTURE_OUTPUT_ALREADY_EXISTS:NDT_SOURCE.pcd");
    std::ofstream out(path, std::ios::binary | std::ios::out);
    if (!out) throw std::runtime_error("A3G_R3_CAPTURE_NDT_PCD_OPEN_FAILED");
    out << "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
        << "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH "
        << cloud.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS "
        << cloud.size() << "\nDATA binary\n";
    for (const auto& point : cloud.points) {
      writeFloat32(out, point.x); writeFloat32(out, point.y); writeFloat32(out, point.z);
    }
    out.flush();
    if (!out) throw std::runtime_error("A3G_R3_CAPTURE_NDT_PCD_WRITE_FAILED");
  }

  void writeInputMetadata(
      const fixed_lag::RawTimedScan& raw,
      const fixed_lag::WindowState& scan_start,
      const fixed_lag::WindowState& scan_end,
      const fixed_lag::WindowDeskewResult& deskew, const Cloud& source,
      const Pose3d& T_imu_lidar, const Eigen::Matrix4d& ndt_initial,
      const fixed_lag::WindowSummary& pre_measurement_summary,
      const std::filesystem::path& dir) {
    const auto path = dir / "RAW_TIMED_POINTS_SCHEMA.json";
    if (std::filesystem::exists(path))
      throw std::runtime_error("A3G_R3_CAPTURE_OUTPUT_ALREADY_EXISTS:RAW_TIMED_POINTS_SCHEMA.json");
    std::ofstream schema(path, std::ios::out | std::ios::binary);
    if (!schema) throw std::runtime_error("A3G_R3_CAPTURE_SCHEMA_OPEN_FAILED");
    schema << "{\n  \"record_size_bytes\": 40,\n  \"endianness\": \"little\",\n"
        << "  \"fields\": [\"float64 x\",\"float64 y\",\"float64 z\","
           "\"float64 intensity\",\"uint64 absolute_sensor_point_stamp_ns\"],\n"
        << "  \"point_count\": " << raw.points.size() << ",\n"
        << "  \"scan_start_ns\": " << raw.scan_start_ns << ",\n"
        << "  \"scan_end_ns\": " << raw.scan_end_ns << ",\n"
        << "  \"provenance\": \"" << toString(raw.provenance) << "\",\n"
        << "  \"file_sha256\": \"" << p5_i1::sha256File((dir / "RAW_TIMED_POINTS.bin").string()) << "\"\n}\n";
    schema.flush();
    if (!schema) throw std::runtime_error("A3G_R3_CAPTURE_SCHEMA_WRITE_FAILED");
    input_revision_ = pre_measurement_summary.window_revision;
    (void)scan_start; (void)scan_end; (void)deskew; (void)source;
    (void)T_imu_lidar; (void)ndt_initial;
  }

  static void writeFileIdentity(std::ostream& out, const std::filesystem::path& path) {
    const auto size = std::filesystem::file_size(path);
    out << "{\"bytes\":" << size << ",\"sha256\":\""
        << p5_i1::sha256File(path.string()) << "\"}";
  }

  std::string root_;
  std::set<std::uint64_t> selected_;
  mutable std::ofstream states_;
  std::set<std::uint64_t> input_written_;
  std::set<std::uint64_t> completed_;
  std::uint64_t input_revision_ = 0;
};
