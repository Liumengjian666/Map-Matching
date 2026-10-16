#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLPointField.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
void ndtReadyCallback(const nav_msgs::Odometry::ConstPtr &) {}

struct ImuRecord
{
  uint64_t stamp_ns = 0;
  double gx = 0.0;
  double gy = 0.0;
  double gz = 0.0;
  double ax = 0.0;
  double ay = 0.0;
  double az = 0.0;
};

std::vector<std::string> splitCsv(const std::string &line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) fields.push_back(field);
  return fields;
}

const pcl::PCLPointField *findPclField(const pcl::PCLPointCloud2 &cloud,
                                      const std::string &name)
{
  const auto it = std::find_if(cloud.fields.begin(), cloud.fields.end(),
      [&name](const pcl::PCLPointField &field) { return field.name == name; });
  return it == cloud.fields.end() ? nullptr : &(*it);
}

std::vector<ImuRecord> loadImuCsv(const std::string &path)
{
  std::ifstream input(path.c_str());
  if (!input) throw std::runtime_error("cannot open Ouster IMU CSV: " + path);
  std::vector<ImuRecord> records;
  std::string line;
  uint64_t previous_stamp = 0;
  while (std::getline(input, line))
  {
    if (line.empty() || line[0] == '#') continue;
    const auto fields = splitCsv(line);
    if (fields.size() != 9) continue;  // Ignore a CSV header, if present.
    ImuRecord record;
    const uint64_t sec = std::stoull(fields[1]);
    const uint64_t nsec = std::stoull(fields[2]);
    if (nsec >= 1000000000ULL) throw std::runtime_error("invalid IMU nanoseconds field");
    record.stamp_ns = sec * 1000000000ULL + nsec;
    record.gx = std::stod(fields[3]);
    record.gy = std::stod(fields[4]);
    record.gz = std::stod(fields[5]);
    record.ax = std::stod(fields[6]);
    record.ay = std::stod(fields[7]);
    record.az = std::stod(fields[8]);
    if (previous_stamp != 0 && record.stamp_ns <= previous_stamp)
      throw std::runtime_error("Ouster IMU timestamps are not strictly increasing");
    if (!std::isfinite(record.gx) || !std::isfinite(record.gy) || !std::isfinite(record.gz) ||
        !std::isfinite(record.ax) || !std::isfinite(record.ay) || !std::isfinite(record.az))
      throw std::runtime_error("nonfinite Ouster IMU record");
    previous_stamp = record.stamp_ns;
    records.push_back(record);
  }
  if (records.empty()) throw std::runtime_error("Ouster IMU CSV has no data records");
  return records;
}

sensor_msgs::PointCloud2 makeTimedCloud(const pcl::PCLPointCloud2 &pcl_cloud,
                                       uint64_t scan_stamp_ns,
                                       const std::string &lidar_frame,
                                       uint32_t &max_offset_ns)
{
  sensor_msgs::PointCloud2 cloud;
  pcl_conversions::fromPCL(pcl_cloud, cloud);
  const auto *time_field = findPclField(pcl_cloud, "t");
  if (!time_field || time_field->datatype != sensor_msgs::PointField::UINT32 ||
      time_field->count != 1 || pcl_cloud.is_bigendian || pcl_cloud.point_step == 0 ||
      time_field->offset + sizeof(uint32_t) > pcl_cloud.point_step)
    throw std::runtime_error("official PCD field t must be one little-endian uint32 per point");
  if (cloud.width == 0 || cloud.height == 0 || cloud.point_step != pcl_cloud.point_step ||
      cloud.row_step < cloud.width * cloud.point_step ||
      cloud.data.size() < static_cast<std::size_t>(cloud.row_step) * cloud.height)
    throw std::runtime_error("PCL returned an invalid organized PointCloud2 layout");
  if (std::any_of(cloud.fields.begin(), cloud.fields.end(),
      [](const sensor_msgs::PointField &field) { return field.name == "time"; }))
    throw std::runtime_error("unexpected pre-existing time field in official PCD");

  const uint32_t old_point_step = cloud.point_step;
  const uint32_t old_row_step = cloud.row_step;
  const uint32_t new_point_step = old_point_step + sizeof(float);
  const uint32_t new_row_step = cloud.width * new_point_step;
  std::vector<uint8_t> expanded(static_cast<std::size_t>(new_row_step) * cloud.height, 0);
  max_offset_ns = 0;
  for (uint32_t row = 0; row < cloud.height; ++row)
  {
    for (uint32_t column = 0; column < cloud.width; ++column)
    {
      const std::size_t source_offset = static_cast<std::size_t>(row) * old_row_step +
          static_cast<std::size_t>(column) * old_point_step;
      const std::size_t target_offset = static_cast<std::size_t>(row) * new_row_step +
          static_cast<std::size_t>(column) * new_point_step;
      std::memcpy(expanded.data() + target_offset, cloud.data.data() + source_offset, old_point_step);
      uint32_t offset_ns = 0;
      std::memcpy(&offset_ns, cloud.data.data() + source_offset + time_field->offset, sizeof(offset_ns));
      max_offset_ns = std::max(max_offset_ns, offset_ns);
      const float offset_sec = static_cast<float>(static_cast<double>(offset_ns) * 1e-9);
      std::memcpy(expanded.data() + target_offset + old_point_step, &offset_sec, sizeof(offset_sec));
    }
  }
  sensor_msgs::PointField point_time;
  point_time.name = "time";
  point_time.offset = old_point_step;
  point_time.datatype = sensor_msgs::PointField::FLOAT32;
  point_time.count = 1;
  cloud.fields.push_back(point_time);
  cloud.data.swap(expanded);
  cloud.point_step = new_point_step;
  cloud.row_step = new_row_step;
  cloud.header.seq = 0;
  cloud.header.stamp.fromNSec(scan_stamp_ns);
  cloud.header.frame_id = lidar_frame;
  return cloud;
}

sensor_msgs::Imu makeImuMessage(const ImuRecord &record, const std::string &frame)
{
  sensor_msgs::Imu message;
  message.header.stamp.fromNSec(record.stamp_ns);
  message.header.frame_id = frame;
  message.orientation_covariance[0] = -1.0;  // No orientation measurement in this stream.
  message.angular_velocity.x = record.gx;
  message.angular_velocity.y = record.gy;
  message.angular_velocity.z = record.gz;
  message.linear_acceleration.x = record.ax;
  message.linear_acceleration.y = record.ay;
  message.linear_acceleration.z = record.az;
  return message;
}

}  // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "newer_college_first_scan_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  try
  {
    std::string pcd_path, imu_csv, lidar_frame, imu_frame, cloud_topic, imu_topic;
    std::string ndt_ready_topic;
    int scan_sec = -1, scan_nsec = -1;
    double readiness_timeout_sec = 120.0;
    pnh.param<std::string>("scan_pcd", pcd_path, "");
    pnh.param<std::string>("imu_csv", imu_csv, "");
    pnh.param<std::string>("lidar_frame", lidar_frame, "os1_lidar");
    pnh.param<std::string>("imu_frame", imu_frame, "os1_imu");
    pnh.param<std::string>("cloud_topic", cloud_topic, "/newer_college/raw_points");
    pnh.param<std::string>("imu_topic", imu_topic, "/newer_college/imu");
    pnh.param<std::string>("ndt_ready_topic", ndt_ready_topic, "/dog_livo/ndt_odom");
    pnh.param("readiness_timeout_sec", readiness_timeout_sec, 120.0);
    pnh.param("scan_stamp_sec", scan_sec, -1);
    pnh.param("scan_stamp_nsec", scan_nsec, -1);
    if (pcd_path.empty() || imu_csv.empty() || scan_sec < 0 || scan_nsec < 0 ||
        scan_nsec >= 1000000000 || lidar_frame.empty() || imu_frame.empty() ||
        ndt_ready_topic.empty() || !std::isfinite(readiness_timeout_sec) || readiness_timeout_sec <= 0.0)
      throw std::runtime_error("missing P41 scan/IMU publisher parameters");
    const uint64_t scan_stamp_ns = static_cast<uint64_t>(scan_sec) * 1000000000ULL +
        static_cast<uint64_t>(scan_nsec);

    pcl::PCLPointCloud2 pcl_cloud;
    if (pcl::io::loadPCDFile(pcd_path, pcl_cloud) != 0)
      throw std::runtime_error("failed to load official scan PCD: " + pcd_path);
    uint32_t max_offset_ns = 0;
    sensor_msgs::PointCloud2 cloud = makeTimedCloud(pcl_cloud, scan_stamp_ns,
        lidar_frame, max_offset_ns);
    const uint64_t scan_end_ns = scan_stamp_ns + max_offset_ns;
    const auto imu_records = loadImuCsv(imu_csv);

    const ImuRecord *boundary = nullptr;
    std::vector<ImuRecord> replay;
    bool found_bracket = false;
    uint64_t previous_selected_stamp = 0;
    for (const auto &record : imu_records)
    {
      if (record.stamp_ns <= scan_stamp_ns) boundary = &record;
      if (record.stamp_ns <= scan_stamp_ns) continue;
      if (!boundary)
        throw std::runtime_error("Ouster IMU has no sample at or before scan start");
      const uint64_t previous_stamp = previous_selected_stamp == 0 ?
          boundary->stamp_ns : previous_selected_stamp;
      if (record.stamp_ns - previous_stamp > 50000000ULL)
        throw std::runtime_error("Ouster IMU interval exceeds the frozen 50 ms propagation contract");
      replay.push_back(record);
      previous_selected_stamp = record.stamp_ns;
      if (record.stamp_ns > scan_end_ns)
      {
        found_bracket = true;
        break;
      }
    }
    if (!boundary || !found_bracket || replay.empty())
      throw std::runtime_error("Ouster IMU does not provide a prior boundary sample and post-scan bracket");
    if (scan_stamp_ns - boundary->stamp_ns > 50000000ULL)
      throw std::runtime_error("initialization boundary IMU sample is older than 50 ms");

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(imu_topic, 500, false);
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_topic, 2, false);
    // The EKF's deskew output is not latched. Waiting only for its raw-cloud
    // subscriber can publish before NDT finishes loading the multi-million
    // point map. Observe NDT's output publisher as a readiness handshake.
    ros::Subscriber ndt_ready_sub = nh.subscribe<nav_msgs::Odometry>(
        ndt_ready_topic, 1, &ndtReadyCallback);
    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(readiness_timeout_sec);
    while (ros::ok() && ros::WallTime::now() < deadline &&
           (imu_pub.getNumSubscribers() == 0 || cloud_pub.getNumSubscribers() == 0 ||
            ndt_ready_sub.getNumPublishers() == 0))
      ros::WallDuration(0.05).sleep();
    if (!ros::ok() || imu_pub.getNumSubscribers() == 0 || cloud_pub.getNumSubscribers() == 0 ||
        ndt_ready_sub.getNumPublishers() == 0)
      throw std::runtime_error("timed out waiting for EKF inputs and the map-loaded NDT output publisher");

    ROS_INFO("P41 ready: raw_cloud_subscribers=%u imu_subscribers=%u NDT_publishers=%u on %s",
             cloud_pub.getNumSubscribers(), imu_pub.getNumSubscribers(),
             ndt_ready_sub.getNumPublishers(), ndt_ready_topic.c_str());
    ROS_INFO("P41 publishing one original scan: points=%u x %u point_step=%u t=[0,%u] ns, stamp=%d.%09d",
             cloud.width, cloud.height, cloud.point_step, max_offset_ns, scan_sec, scan_nsec);
    ROS_INFO("P41 Ouster IMU boundary=%llu ns (age %.3f ms); publishing %zu real samples through %.3f ms after scan start",
             static_cast<unsigned long long>(boundary->stamp_ns),
             static_cast<double>(scan_stamp_ns - boundary->stamp_ns) * 1e-6,
             replay.size(), static_cast<double>(replay.back().stamp_ns - scan_stamp_ns) * 1e-6);
    for (const auto &record : replay)
    {
      imu_pub.publish(makeImuMessage(record, imu_frame));
      ros::WallDuration(0.003).sleep();
    }
    ros::WallDuration(0.25).sleep();
    cloud_pub.publish(cloud);
    ROS_INFO("P41 published exactly one raw PointCloud2 scan; point t was preserved as uint32 ns and adapted to float32 seconds_from_scan_start without changing XYZ.");
    ros::WallDuration(3.0).sleep();
    ros::spin();
  }
  catch (const std::exception &error)
  {
    ROS_FATAL("P41 single-scan publisher failed: %s", error.what());
    return 1;
  }
  return 0;
}
