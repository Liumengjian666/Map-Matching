#include "dog_prior_map_localization/core/imu_deskew.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <vector>

namespace
{
using dog_prior_map_localization::FilterStateSnapshot;
using dog_prior_map_localization::ImuDeskewPose;
using dog_prior_map_localization::ImuKinematicsConfig;
using dog_prior_map_localization::ImuSample;
using dog_prior_map_localization::StateHistory;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

struct CsvTable
{
  std::vector<std::string> header;
  std::map<std::string, std::size_t> column;
  std::vector<std::vector<std::string>> rows;
  std::string get(const std::vector<std::string> &row, const std::string &key) const
  {
    auto found = column.find(key);
    if (found == column.end() || found->second >= row.size())
      throw std::runtime_error("missing CSV column: " + key);
    return row[found->second];
  }
};

std::vector<std::string> splitCsv(const std::string &line)
{
  std::vector<std::string> fields;
  std::stringstream input(line);
  std::string field;
  while (std::getline(input, field, ',')) fields.push_back(field);
  return fields;
}

CsvTable readCsv(const std::string &path)
{
  std::ifstream input(path.c_str());
  if (!input) throw std::runtime_error("cannot open CSV: " + path);
  CsvTable table;
  std::string line;
  if (!std::getline(input, line)) throw std::runtime_error("empty CSV: " + path);
  table.header = splitCsv(line);
  for (std::size_t i = 0; i < table.header.size(); ++i)
    table.column.emplace(table.header[i], i);
  while (std::getline(input, line))
  {
    if (line.empty()) continue;
    auto row = splitCsv(line);
    if (row.size() != table.header.size())
      throw std::runtime_error("malformed CSV row in " + path);
    table.rows.push_back(std::move(row));
  }
  return table;
}

struct ImuRecord
{
  ImuSample sample;
};

std::vector<ImuRecord> readImuCsv(const std::string &path)
{
  std::ifstream input(path.c_str());
  if (!input) throw std::runtime_error("cannot open Ouster IMU CSV: " + path);
  std::vector<ImuRecord> result;
  std::string line;
  double previous = -std::numeric_limits<double>::infinity();
  while (std::getline(input, line))
  {
    if (line.empty() || line[0] == '#') continue;
    const auto fields = splitCsv(line);
    if (fields.size() != 9) throw std::runtime_error("unexpected Ouster IMU CSV field count");
    const uint64_t sec = std::stoull(fields[1]);
    const uint64_t nsec = std::stoull(fields[2]);
    if (nsec >= 1000000000ULL) throw std::runtime_error("invalid IMU nsec");
    ImuRecord r;
    r.sample.stamp = static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9;
    r.sample.gyro = Eigen::Vector3d(std::stod(fields[3]), std::stod(fields[4]), std::stod(fields[5]));
    r.sample.acc = Eigen::Vector3d(std::stod(fields[6]), std::stod(fields[7]), std::stod(fields[8]));
    if (!(r.sample.stamp > previous) || !r.sample.gyro.allFinite() || !r.sample.acc.allFinite())
      throw std::runtime_error("nonmonotonic or nonfinite Ouster IMU input");
    previous = r.sample.stamp;
    result.push_back(r);
  }
  if (result.empty()) throw std::runtime_error("Ouster IMU CSV has no measurements");
  return result;
}

const pcl::PCLPointField &field(const pcl::PCLPointCloud2 &cloud, const std::string &name,
                                uint8_t datatype)
{
  auto found = std::find_if(cloud.fields.begin(), cloud.fields.end(), [&](const auto &f) {
    return f.name == name;
  });
  if (found == cloud.fields.end() || found->datatype != datatype || found->count != 1 ||
      found->offset + 4 > cloud.point_step)
    throw std::runtime_error("invalid required PCD field: " + name);
  return *found;
}

uint64_t stableCloudHash(const Cloud::Ptr &cloud)
{
  constexpr uint64_t kOffset = 1469598103934665603ULL;
  constexpr uint64_t kPrime = 1099511628211ULL;
  uint64_t hash = kOffset;
  auto mixByte = [&](uint8_t b) { hash ^= b; hash *= kPrime; };
  auto mixU32 = [&](uint32_t v) {
    for (int shift = 0; shift < 32; shift += 8) mixByte(static_cast<uint8_t>((v >> shift) & 0xffU));
  };
  mixU32(cloud->width); mixU32(cloud->height); mixU32(cloud->is_dense ? 1U : 0U);
  mixU32(static_cast<uint32_t>(cloud->size()));
  for (const auto &p : cloud->points)
  {
    uint32_t bits = 0;
    std::memcpy(&bits, &p.x, sizeof(bits)); mixU32(bits);
    std::memcpy(&bits, &p.y, sizeof(bits)); mixU32(bits);
    std::memcpy(&bits, &p.z, sizeof(bits)); mixU32(bits);
  }
  return hash;
}

std::string hex64(uint64_t value)
{
  std::ostringstream out;
  out << std::hex << std::setw(16) << std::setfill('0') << value;
  return out.str();
}

Cloud::Ptr preprocess(const Cloud::Ptr &deskewed)
{
  Cloud::Ptr filtered(new Cloud());
  filtered->reserve(deskewed->size());
  for (const auto &p : deskewed->points)
  {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    const double r = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    if (r < 0.5 || r > 80.0) continue;
    filtered->push_back(p);
  }
  filtered->width = static_cast<uint32_t>(filtered->size());
  filtered->height = 1;
  filtered->is_dense = true;

  Cloud::Ptr down(new Cloud());
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setLeafSize(0.25f, 0.25f, 0.25f);
  voxel.setInputCloud(filtered);
  voxel.filter(*down);
  if (down->size() > 1400)
  {
    Cloud::Ptr sampled(new Cloud());
    sampled->reserve(1400);
    const double step = static_cast<double>(down->size() - 1) / 1399.0;
    for (int i = 0; i < 1400; ++i)
      sampled->push_back(down->points[static_cast<std::size_t>(std::round(i * step))]);
    sampled->width = static_cast<uint32_t>(sampled->size());
    sampled->height = 1;
    sampled->is_dense = true;
    return sampled;
  }
  down->width = static_cast<uint32_t>(down->size());
  down->height = 1;
  down->is_dense = true;
  return down;
}

void appendSnapshot(StateHistory &history, FilterStateSnapshot snapshot)
{
  history.insertMonotonic(snapshot);
  if (history.empty() || std::abs(history.samples().back().stamp - snapshot.stamp) > 1e-9)
    throw std::runtime_error("failed to append monotonic reconstructed IMU state");
}

Cloud::Ptr reconstruct(const std::vector<std::string> &row, const CsvTable &manifest,
                       const std::vector<ImuRecord> &imu)
{
  const std::string pcd_path = manifest.get(row, "pcd_path");
  pcl::PCLPointCloud2 raw;
  if (pcl::io::loadPCDFile(pcd_path, raw) != 0)
    throw std::runtime_error("cannot load sampled PCD: " + pcd_path);
  if (raw.is_bigendian || raw.point_step == 0 || raw.width == 0 || raw.height == 0 ||
      raw.row_step < raw.width * raw.point_step ||
      raw.data.size() < static_cast<std::size_t>(raw.row_step) * raw.height)
    throw std::runtime_error("invalid PointCloud2-compatible PCD layout");
  const auto &fx = field(raw, "x", pcl::PCLPointField::FLOAT32);
  const auto &fy = field(raw, "y", pcl::PCLPointField::FLOAT32);
  const auto &fz = field(raw, "z", pcl::PCLPointField::FLOAT32);
  const auto &ft = field(raw, "t", pcl::PCLPointField::UINT32);

  const double start = std::stod(manifest.get(row, "scan_start"));
  const double end = std::stod(manifest.get(row, "scan_end"));
  const double anchor_stamp = std::stod(manifest.get(row, "rollback_stamp"));
  if (!(end >= start) || anchor_stamp > start + 1e-9)
    throw std::runtime_error("rollback anchor does not causally precede scan start");
  Eigen::Vector3d p(std::stod(manifest.get(row, "rollback_px")),
                    std::stod(manifest.get(row, "rollback_py")),
                    std::stod(manifest.get(row, "rollback_pz")));
  Eigen::Vector3d v(std::stod(manifest.get(row, "rollback_vx")),
                    std::stod(manifest.get(row, "rollback_vy")),
                    std::stod(manifest.get(row, "rollback_vz")));
  Eigen::Quaterniond q(std::stod(manifest.get(row, "rollback_qw")),
                        std::stod(manifest.get(row, "rollback_qx")),
                        std::stod(manifest.get(row, "rollback_qy")),
                        std::stod(manifest.get(row, "rollback_qz")));
  if (!p.allFinite() || !v.allFinite() || !q.coeffs().allFinite() || q.norm() < 1e-12)
    throw std::runtime_error("invalid OOSM rollback snapshot");
  q.normalize();

  const auto anchor_after = std::upper_bound(imu.begin(), imu.end(), anchor_stamp,
      [](double stamp, const ImuRecord &sample) { return stamp < sample.sample.stamp; });
  if (anchor_after == imu.begin())
    throw std::runtime_error("OOSM rollback has no causal raw IMU sample at or before its state time");
  const auto best = anchor_after - 1;
  if (anchor_stamp - best->sample.stamp > 0.02 + 1e-9)
    throw std::runtime_error("latest causal IMU sample is outside the 20 ms P46 gap contract");
  const std::size_t anchor_index = static_cast<std::size_t>(std::distance(imu.begin(), best));

  const Eigen::Vector3d gravity(0.0, 0.0, -9.80665);
  ImuKinematicsConfig config;
  config.use_acc_for_position = true;
  config.velocity_damping = 0.98;
  config.midpoint_interval_input_enable = true;
  config.continuous_gravity_correction_enable = true;
  config.gravity_correction_expected_acc_norm = 1.0;
  config.gravity_correction_gain = 0.01;
  config.gravity_correction_max_angle = 0.5 * std::acos(-1.0) / 180.0;
  config.gravity_correction_acc_tolerance = 1.5;
  config.gravity_correction_gyro_max = 0.8;
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();

  StateHistory history;
  Eigen::Matrix3d R = q.toRotationMatrix();
  FilterStateSnapshot state;
  state.stamp = anchor_stamp;
  state.p = p; state.v = v; state.R = R;
  state.ba.setZero(); state.bg.setZero();
  state.acc_measurement = best->sample.acc;
  state.gyro_measurement = best->sample.gyro;
  state.acc_world = state.R * state.acc_measurement + gravity;
  state.gyro_unbiased = state.gyro_measurement;
  appendSnapshot(history, state);

  double previous_stamp = anchor_stamp;
  Eigen::Vector3d previous_acc = best->sample.acc;
  Eigen::Vector3d previous_gyro = best->sample.gyro;
  Eigen::Vector3d acc_world = state.acc_world;
  Eigen::Vector3d gyro_unbiased = state.gyro_unbiased;
  bool bracketed_end = false;
  for (std::size_t j = anchor_index + 1; j < imu.size(); ++j)
  {
    const auto &sample = imu[j].sample;
    const double dt = sample.stamp - previous_stamp;
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.02 + 1e-9)
      throw std::runtime_error("raw IMU interval invalid or exceeds P46 deskew gap contract");
    const auto interval = dog_prior_map_localization::makeImuIntervalInput(
        previous_acc, previous_gyro, sample.acc, sample.gyro,
        dog_prior_map_localization::ImuIntervalInputPolicy::kMidpointAverage);
    dog_prior_map_localization::propagateImuKinematics(
        p, v, R, zero, zero, interval.acc, interval.gyro, dt,
        gravity, config, acc_world, gyro_unbiased);
    state = FilterStateSnapshot{};
    state.stamp = sample.stamp; state.p = p; state.v = v;
    state.R = R;
    state.ba.setZero(); state.bg.setZero();
    state.acc_measurement = sample.acc; state.gyro_measurement = sample.gyro;
    state.interval_acc_input = interval.acc; state.interval_gyro_input = interval.gyro;
    state.has_interval_input = true; state.acc_world = acc_world; state.gyro_unbiased = gyro_unbiased;
    appendSnapshot(history, state);
    previous_stamp = sample.stamp; previous_acc = sample.acc; previous_gyro = sample.gyro;
    if (sample.stamp > end)
    {
      bracketed_end = true;
      break;
    }
  }
  if (!bracketed_end) throw std::runtime_error("raw IMU does not bracket scan end");

  dog_prior_map_localization::ImuDeskewCoverage coverage;
  std::string reason;
  if (!dog_prior_map_localization::inspectImuDeskewCoverage(history, start, end, 0.02,
                                                            coverage, reason))
    throw std::runtime_error("reconstructed history fails P46 deskew coverage: " + reason);
  ImuDeskewPose reference;
  if (!dog_prior_map_localization::interpolateImuDeskewPose(
          history, start, 0.02, gravity, config, reference, reason, end))
    throw std::runtime_error("cannot reconstruct P46 reference pose: " + reason);

  Eigen::Isometry3d T_imu_lidar = Eigen::Isometry3d::Identity();
  T_imu_lidar.linear() = (Eigen::Vector3d(-1.0, -1.0, 1.0).asDiagonal()).toDenseMatrix();
  T_imu_lidar.translation() = Eigen::Vector3d(-0.006253, 0.011775, -0.028535);
  Cloud::Ptr deskewed(new Cloud());
  deskewed->reserve(static_cast<std::size_t>(raw.width) * raw.height);
  double max_float_offset = 0.0;
  for (uint32_t i = 0; i < static_cast<uint32_t>(raw.width) * raw.height; ++i)
  {
    const std::size_t row_index = i / raw.width;
    const std::size_t column_index = i % raw.width;
    const std::size_t offset = row_index * raw.row_step + column_index * raw.point_step;
    float x = 0, y = 0, z = 0;
    uint32_t offset_ns = 0;
    std::memcpy(&x, raw.data.data() + offset + fx.offset, sizeof(x));
    std::memcpy(&y, raw.data.data() + offset + fy.offset, sizeof(y));
    std::memcpy(&z, raw.data.data() + offset + fz.offset, sizeof(z));
    std::memcpy(&offset_ns, raw.data.data() + offset + ft.offset, sizeof(offset_ns));
    const float point_time = static_cast<float>(static_cast<double>(offset_ns) * 1e-9);
    max_float_offset = std::max(max_float_offset, static_cast<double>(point_time));
    const double point_stamp = start + static_cast<double>(point_time);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        point_stamp < start - 1e-6 || point_stamp > end + 1e-6)
      throw std::runtime_error("invalid raw point or point-time contract");
    ImuDeskewPose point_pose;
    if (!dog_prior_map_localization::interpolateImuDeskewPose(
            history, point_stamp, 0.02, gravity, config, point_pose, reason, end))
      throw std::runtime_error("point-time interpolation failed: " + reason);
    Eigen::Vector3d output;
    if (!dog_prior_map_localization::transformPointToReferenceLidar(
            point_pose, reference, T_imu_lidar, Eigen::Vector3d(x, y, z), output, reason))
      throw std::runtime_error("point transform failed: " + reason);
    deskewed->push_back(pcl::PointXYZ(static_cast<float>(output.x()),
                                      static_cast<float>(output.y()),
                                      static_cast<float>(output.z())));
  }
  if (std::abs((start + max_float_offset) - end) > 1e-6)
    throw std::runtime_error("reconstructed float32 point-time scan end differs from P46");
  deskewed->width = static_cast<uint32_t>(deskewed->size());
  deskewed->height = 1;
  deskewed->is_dense = true;
  return preprocess(deskewed);
}

}  // namespace

int main(int argc, char **argv)
{
  if (argc != 5)
  {
    std::cerr << "usage: p47_reconstruct_frozen_sources SAMPLE_MANIFEST IMU_CSV OUTPUT_DIR PARITY_CSV\n";
    return 2;
  }
  try
  {
    struct stat info{};
    if (stat(argv[3], &info) != 0 || !S_ISDIR(info.st_mode))
      throw std::runtime_error("source output directory must already exist");
    if (std::ifstream(argv[4])) throw std::runtime_error("refusing to overwrite parity CSV");
    const CsvTable manifest = readCsv(argv[1]);
    const auto imu = readImuCsv(argv[2]);
    std::ofstream parity(argv[4]);
    if (!parity) throw std::runtime_error("cannot create parity CSV");
    parity << "frame_index,scan_ns,expected_hash,reconstructed_hash,source_points,parity,source_path\n";
    const std::string output_dir = argv[3];
    for (const auto &row : manifest.rows)
    {
      const std::string frame = manifest.get(row, "frame_index");
      const std::string expected = manifest.get(row, "expected_source_hash");
      Cloud::Ptr source = reconstruct(row, manifest, imu);
      const std::string actual = hex64(stableCloudHash(source));
      const std::string source_path = output_dir + "/source_frame_" + frame + ".pcd";
      if (actual != expected)
      {
        parity << frame << ',' << manifest.get(row, "scan_ns") << ',' << expected << ',' << actual
               << ',' << source->size() << ",FAIL," << source_path << '\n';
        parity.flush();
        std::cerr << "P47_SOURCE_HASH_MISMATCH frame=" << frame << " expected=" << expected
                  << " actual=" << actual << '\n';
        return 10;
      }
      if (std::ifstream(source_path)) throw std::runtime_error("refusing to overwrite source PCD: " + source_path);
      if (pcl::io::savePCDFileBinary(source_path, *source) != 0)
        throw std::runtime_error("failed to save reconstructed source: " + source_path);
      parity << frame << ',' << manifest.get(row, "scan_ns") << ',' << expected << ',' << actual
             << ',' << source->size() << ",PASS," << source_path << '\n';
      parity.flush();
      std::cout << "P47_SOURCE_PARITY_PASS frame=" << frame << " hash=" << actual
                << " points=" << source->size() << '\n';
    }
    return 0;
  }
  catch (const std::exception &e)
  {
    std::cerr << "P47_SOURCE_RECONSTRUCTION_FAILED: " << e.what() << '\n';
    return 1;
  }
}
