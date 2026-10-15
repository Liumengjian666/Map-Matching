// Offline packet conversion only. Uses the existing BSD-licensed ros-drivers/
// velodyne RawData implementation; no estimator, TF, GT or deskew is involved.
#include <velodyne_pointcloud/rawdata.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <stdexcept>
#include <unistd.h>

namespace {
void require(bool condition, const char* reason) {
  if (!condition) throw std::runtime_error(reason);
}
struct Point {
  float x, y, z, intensity;
  std::uint16_t ring, padding;
  float time;
};
static_assert(sizeof(Point) == 24 && offsetof(Point, time) == 20, "XYZIRT layout");

class Collector : public velodyne_rawdata::DataContainerBase {
 public:
  Collector() : DataContainerBase(1000., 0., "", "", 0, 1, true, 384, 0) {}
  std::uint64_t packet_offset_ns = 0;
  std::vector<Point> points;
  void newLine() override {}
  void addPoint(float x, float y, float z, std::uint16_t ring, std::uint16_t,
                float distance, float intensity, float packet_time) override {
    require(std::isfinite(packet_time) && packet_time >= 0, "invalid_packet_time");
    const auto firing_ns = std::llround(double(packet_time) * 1e9);
    require(firing_ns >= 0 && firing_ns <= 1306368 && ring < 16, "invalid_firing_or_ring");
    const auto firing = firing_ns / 55296;
    const auto laser = (firing_ns % 55296) / 2304;
    require(firing < 24 && laser < 16 && firing_ns == firing * 55296 + laser * 2304,
            "unexpected_firing_schedule");
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        !std::isfinite(distance) || !std::isfinite(intensity) || distance <= 0) return;
    const std::uint64_t offset = packet_offset_ns + std::uint64_t(firing_ns);
    require(offset <= UINT32_MAX, "point_time_overflow");
    points.push_back({x, y, z, intensity, ring, 0, float(double(offset) * 1e-9)});
  }
};

sensor_msgs::PointCloud2 cloud(const std_msgs::Header& header, const Collector& data) {
  sensor_msgs::PointCloud2 result;
  result.header = header;
  result.height = 1;
  result.width = data.points.size();
  result.is_bigendian = false;
  result.is_dense = true;
  result.point_step = sizeof(Point);
  result.row_step = result.width * result.point_step;
  const char* names[] = {"x", "y", "z", "intensity", "ring", "time"};
  const std::uint32_t offsets[] = {0, 4, 8, 12, 16, 20};
  for (int i = 0; i < 6; ++i) {
    sensor_msgs::PointField field;
    field.name = names[i]; field.offset = offsets[i]; field.count = 1;
    field.datatype = i == 4 ? sensor_msgs::PointField::UINT16 : sensor_msgs::PointField::FLOAT32;
    result.fields.push_back(field);
  }
  result.data.resize(result.row_step);
  if (!result.data.empty()) std::memcpy(result.data.data(), data.points.data(), result.data.size());
  return result;
}

void setup(velodyne_rawdata::RawData& decoder, const std::string& calibration) {
  require(decoder.setupOffline(calibration, "VLP16", 1000., 0.) == 0, "calibration_failed");
  decoder.setParameters(0., 1000., 0., 2 * M_PI);
}

void selfTest(const std::string& calibration) {
  velodyne_rawdata::RawData decoder; setup(decoder, calibration);
  velodyne_msgs::VelodynePacket packet;
  packet.stamp.fromNSec(1517157219000000123ULL);
  std::fill(packet.data.begin(), packet.data.end(), 0);
  for (int block = 0; block < 12; ++block) {
    packet.data[block * 100] = 0xff; packet.data[block * 100 + 1] = 0xee;
    for (int j = 0; j < 32; ++j) {
      packet.data[block * 100 + 4 + j * 3] = 0x88;
      packet.data[block * 100 + 5 + j * 3] = 0x13;
      packet.data[block * 100 + 6 + j * 3] = 42;
    }
  }
  Collector data; data.packet_offset_ns = 2000000;
  decoder.unpack(packet, data, packet.stamp);
  require(data.points.size() == 384, "self_test_point_count");
  for (int j = 0; j < 384; ++j) {
    const auto& p = data.points[j];
    const float expected = float(double(2000000 + j / 16 * 55296 + j % 16 * 2304) * 1e-9);
    require(p.time == expected && p.intensity == 42 && p.ring < 16, "self_test_fields");
    require(std::abs(std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z) - 10) < 1e-5,
            "self_test_metric_xyz");
  }
  std_msgs::Header header; header.stamp = packet.stamp; header.frame_id = "cmu_rc2_velodyne";
  const auto message = cloud(header, data);
  require(message.data.size() == 384 * 24 && message.fields[5].offset == 20,
          "self_test_cloud_layout");
  std::cout << "DECODER_PACKET_CONTRACT_PASS\n";
}

void convert(const char* input, const char* calibration, const char* partial_output) {
  const std::uint16_t endian = 1;
  require(*reinterpret_cast<const unsigned char*>(&endian) == 1, "little_endian_required");
  velodyne_rawdata::RawData decoder; setup(decoder, calibration);
  rosbag::Bag source(input, rosbag::bagmode::Read);
  // Reserve a new partial filename exclusively. Never truncate an existing bag.
  const int fd = open(partial_output, O_CREAT | O_EXCL | O_WRONLY, 0644);
  require(fd >= 0, "output_exists_or_unwritable");
  require(close(fd) == 0, "output_reservation_close_failed");
  rosbag::Bag output(partial_output, rosbag::bagmode::Write);
  Collector data;
  std::uint64_t scans = 0, points = 0, imus = 0, images = 0;
  std::uint64_t previous_start = 0, previous_end = 0;
  for (const auto& item : rosbag::View(source)) {
    if (item.getTopic() == "/imu/data") {
      const auto msg = item.instantiate<sensor_msgs::Imu>();
      require(bool(msg) && !msg->header.stamp.isZero(), "invalid_imu");
      output.write(item.getTopic(), msg->header.stamp, *msg, item.getConnectionHeader());
      ++imus;
    } else if (item.getTopic() == "/camera_1/image_raw") {
      const auto msg = item.instantiate<sensor_msgs::Image>();
      require(bool(msg) && !msg->header.stamp.isZero(), "invalid_image");
      output.write(item.getTopic(), msg->header.stamp, *msg, item.getConnectionHeader());
      ++images;
    } else if (item.getTopic() == "/velodyne_packets") {
      const auto scan = item.instantiate<velodyne_msgs::VelodyneScan>();
      require(bool(scan) && !scan->packets.empty() && scan->header.frame_id == "cmu_rc2_velodyne",
              "invalid_scan");
      const auto start = scan->header.stamp.toNSec();
      require(start > previous_start && start == scan->packets.front().stamp.toNSec(),
              "invalid_scan_start");
      data.points.clear();
      std::uint64_t previous_packet = 0;
      for (const auto& packet : scan->packets) {
        const auto stamp = packet.stamp.toNSec();
        require(stamp >= start && stamp > previous_packet &&
                packet.data[1205] == 0x22 && packet.data[1204] == 0x37,
                "unsupported_packet_time_model_or_return");
        for (int block = 0; block < 12; ++block)
          require(packet.data[100*block] == 0xff && packet.data[100*block+1] == 0xee,
                  "invalid_packet_block");
        data.packet_offset_ns = stamp - start;
        decoder.unpack(packet, data, packet.stamp);
        previous_packet = stamp;
      }
      const auto end = previous_packet + 1306368;
      require(!data.points.empty() && end > start && end > previous_end, "invalid_scan_end");
      for (const auto& p : data.points)
        require(double(p.time) <= double(end - start) * 1e-9 + 1e-8, "point_after_scan_end");
      ros::Time available; available.fromNSec(end);
      output.write("/p10/corridor01/raw_timed_points", available, cloud(scan->header, data));
      previous_start = start; previous_end = end;
      points += data.points.size(); ++scans;
      if (scans % 100 == 0)
        std::cout << "DECODED scans=" << scans << " points=" << points
                  << " imu=" << imus << " images=" << images << std::endl;
    } else {
      throw std::runtime_error("unexpected_original_topic: " + item.getTopic());
    }
  }
  require(scans == 2777 && imus == 55957 && images == 6720 && points == 79932911,
          "full_sequence_count_mismatch");
  output.close(); source.close();
  std::cout << "DECODE_COMPLETE scans=" << scans << " points=" << points
            << " imu=" << imus << " images=" << images << std::endl;
}
}  // namespace

int main(int argc, char** argv) {
  try {
    ros::Time::init();
    if (argc == 3 && std::string(argv[1]) == "--self-test") selfTest(argv[2]);
    else if (argc == 4) convert(argv[1], argv[2], argv[3]);
    else throw std::runtime_error("usage: decoder INPUT_BAG CALIBRATION NEW_PARTIAL_BAG | --self-test CALIBRATION");
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "DECODE_FAILED: " << e.what() << std::endl;
    return 1;
  }
}
