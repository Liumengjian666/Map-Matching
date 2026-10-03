#include "dog_prior_map_fastlio2_frontend_exp/p7_replay_io.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <unistd.h>

using namespace dog_prior_map_fastlio2_frontend_exp;
namespace {
void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
void rejects(const std::function<void()>& action, const char* message) {
  try { action(); } catch (const std::exception&) { return; }
  throw std::runtime_error(message);
}
struct Fixture {
  std::string directory;
  std::vector<std::string> files;
  Fixture() {
    char path[] = "/tmp/p7_io_fixture_XXXXXX";
    require(mkdtemp(path) != nullptr, "cannot create fixture directory");
    directory = path;
  }
  std::string path(const std::string& name) {
    const std::string result = directory + "/" + name;
    files.push_back(result);
    return result;
  }
  ~Fixture() {
    for (const auto& file : files) std::remove(file.c_str());
    rmdir(directory.c_str());
  }
};
void textFile(const std::string& path, const std::string& contents) {
  std::ofstream out(path); out << contents;
  require(bool(out), "fixture write failed");
}

void testImu(Fixture& fixture) {
  const auto path = fixture.path("imu.csv");
  const std::string header = "stamp_ns,ax,ay,az,gx,gy,gz\n";
  textFile(path, header + "10,0,0,9.809,0,0,0\n20,0,0,9.809,0,0,0\n30,0,0,9.809,0,0,0\n40,0,0,9.809,0,0,0\n");
  const auto imu = readP7Imu(path);
  require(imu.size() == 4, "IMU input count");
  const auto causal = imuWindow(imu, 15, 35);
  require(causal.size() == 3 && causal.front().stamp_ns == 10 && causal.back().stamp_ns == 30,
          "causal IMU interval changed");
  rejects([&] { imuWindow(imu, 5, 35); }, "missing causal anchor accepted");
  rejects([&] { imuWindow(imu, 10, 15); }, "one-sample interval accepted");
  textFile(path, header + "20,0,0,9.809,0,0,0\n10,0,0,9.809,0,0,0\n");
  rejects([&] { readP7Imu(path); }, "nonmonotonic IMU accepted");
  textFile(path, header + "10,nan,0,9.809,0,0,0\n");
  rejects([&] { readP7Imu(path); }, "nonfinite IMU accepted");
}

void testScans(Fixture& fixture) {
  const auto filter = fixture.path("filter.csv"), assets = fixture.path("scans.csv");
  const std::string header = "transaction_id,stamp_ns,time_s,cloud_byte_offset,cloud_point_count,ndt_source_cloud_hash,ndt_source_cloud_hash_available,raw_x,used_x,fitness,iterations\n";
  const std::string first = "1,100,0.1,0,2,123,1,NOT_A_NUMBER,NOT_A_NUMBER,NOT_A_NUMBER,NOT_A_NUMBER\n";
  const std::string second = "2,200,0.2,24,1,0,0,NOT_A_NUMBER,NOT_A_NUMBER,NOT_A_NUMBER,NOT_A_NUMBER\n";
  std::string historical_fields;
  for (int i = 0; i < 21; ++i) historical_fields += ",NOT_A_NUMBER";
  textFile(filter, "transaction_id,stamp_ns\n1,100" + historical_fields + "\n2,200\n");
  textFile(assets, header + first + second);
  const auto scans = readP7Scans(filter, assets);
  require(scans.size() == 2 && scans[0].expected_source_hash_available &&
      scans[0].expected_source_hash == 123 && !scans[1].expected_source_hash_available,
      "minimal scan/hash fields not read");
  textFile(assets, header + first + "3,200,0.2,24,1,0,0,bad,bad,bad,bad\n");
  rejects([&] { readP7Scans(filter, assets); }, "nonsequential transaction accepted");
  textFile(assets, header + first + "2,99,0.2,24,1,0,0,bad,bad,bad,bad\n");
  rejects([&] { readP7Scans(filter, assets); }, "nonmonotonic scan accepted");
  textFile(assets, header + first + second);
  textFile(filter, "transaction_id,stamp_ns\n1,100\n2,201\n");
  rejects([&] { readP7Scans(filter, assets); }, "metadata stamp disagreement accepted");
}

void testPacked(Fixture& fixture) {
  const auto path = fixture.path("xyz.bin");
  const uint32_t bits[] = {0x3f800000, 0x40000000, 0x40400000,
                          0x80000000, 0x3f123456, 0x7fc01234};
  {
    std::ofstream out(path, std::ios::binary);
    out.write(reinterpret_cast<const char*>(bits), sizeof(bits));
  }
  P7ScanRecord scan; scan.cloud_byte_offset = 12; scan.cloud_point_count = 1;
  const auto cloud = readP7PackedCloud(path, scan);
  require(cloud.size() == 1, "packed point count");
  uint32_t actual[3];
  std::memcpy(&actual[0], &cloud[0].x, 4); std::memcpy(&actual[1], &cloud[0].y, 4);
  std::memcpy(&actual[2], &cloud[0].z, 4);
  require(std::memcmp(actual, bits + 3, 12) == 0, "packed float bits changed");
  scan.cloud_point_count = 2;
  rejects([&] { readP7PackedCloud(path, scan); }, "short binary accepted");
}

void testTimedRawScans(Fixture& fixture) {
  const auto filter = fixture.path("timed_filter.csv");
  const auto index = fixture.path("raw_scan_index.csv");
  const auto binary = fixture.path("raw_timed.bin");
  textFile(filter, "transaction_id,stamp_ns\n1,120\n2,200\n");
  const std::string header =
      "transaction_id,scan_start_ns,scan_end_ns,cloud_byte_offset,cloud_point_count\n";
  textFile(index, header + "1,100,120,0,2\n2,180,200,32,1\n");
  const auto scans = readP7TimedScans(filter, index);
  require(scans.size() == 2 && scans[0].scan_start_ns == 100 &&
      scans[0].scan_end_ns == 120 && scans[1].cloud_byte_offset == 32,
      "raw timed scan index contract changed");
  {
    std::ofstream out(binary, std::ios::binary);
    const float xyz[9] = {1.0f, 2.0f, 3.0f, -1.0f, -2.0f, -3.0f, 4.0f, 5.0f, 6.0f};
    const uint32_t offsets[3] = {0, 20, 20};
    for (int i = 0; i < 3; ++i) {
      out.write(reinterpret_cast<const char*>(xyz + 3 * i), 3 * sizeof(float));
      out.write(reinterpret_cast<const char*>(offsets + i), sizeof(uint32_t));
    }
  }
  const auto first = readP7PackedTimedCloud(binary, scans[0]);
  require(first.size() == 2 && first[0].stamp_ns == 100 &&
      first[1].stamp_ns == 120 && first[1].position == Eigen::Vector3d(-1, -2, -3),
      "raw point-time decoding changed");
  textFile(index,
      "transaction_id,scan_start_ns,scan_end_ns,byte_offset,point_count,provenance\n"
      "1,100,120,0,2,RAW_TIMED_SENSOR\n2,180,200,32,1,RAW_TIMED_SENSOR\n");
  const auto catalog_scans = readP7TimedScans(filter, index);
  require(catalog_scans.size() == 2 && catalog_scans[1].cloud_byte_offset == 32 &&
      catalog_scans[1].cloud_point_count == 1,
      "raw timed catalog field aliases were not accepted");
  textFile(index, header + "1,100,121,0,2\n2,180,200,32,1\n");
  rejects([&] { readP7TimedScans(filter, index); },
          "raw scan end inconsistent with filter schedule accepted");
}

void testParameters(Fixture& fixture) {
  const auto path = fixture.path("params.txt");
  const std::string contents = "200 9.809 0.1 0.2 0.3 0.004 0.08 0.000002 0.00004 0.2 0.1 0.5 0.05 1 2 3 0 0 0 1 0.08 0.029 0.03 0 0 0 1\n";
  textFile(path, contents);
  Pose3d initial, extrinsic;
  const auto p = readP7Parameters(path, &initial, &extrinsic);
  require(p.static_init_samples == 200 && p.initial_accel_bias == Eigen::Vector3d(0.1, 0.2, 0.3) &&
      p.pose_position_sigma_m == 0.2 && p.pose_rotation_sigma_rad == 0.1 &&
      initial.position == Eigen::Vector3d(1, 2, 3) && extrinsic.position == Eigen::Vector3d(0.08, 0.029, 0.03),
      "27-scalar schema changed");
  const auto imu = lidarMeasurementToImu(fromIsometry(asIsometry(initial) * asIsometry(extrinsic)), extrinsic);
  require((imu.position - initial.position).norm() < 1e-12, "pose/extrinsic helper changed");
  textFile(path, "200 9.809\n");
  rejects([&] { readP7Parameters(path, &initial, &extrinsic); }, "wrong parameter count accepted");
  textFile(path, contents + "junk\n");
  rejects([&] { readP7Parameters(path, &initial, &extrinsic); }, "trailing parameter corruption accepted");
  for (const std::string& suffix : {std::string("1e"), std::string("1e999")}) {
    textFile(path, contents + suffix);
    rejects([&] { readP7Parameters(path, &initial, &extrinsic); }, "EOF numeric corruption accepted");
  }
}
}  // namespace

int main() {
  try {
    Fixture fixture;
    testImu(fixture); testScans(fixture); testPacked(fixture);
    testTimedRawScans(fixture); testParameters(fixture);
    std::cout << "p7_replay_io_test PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "p7_replay_io_test FAIL: " << error.what() << '\n';
    return 1;
  }
}
