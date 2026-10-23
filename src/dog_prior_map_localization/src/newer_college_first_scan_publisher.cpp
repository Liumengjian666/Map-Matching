#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLPointField.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Header.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
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
                                       uint32_t &max_offset_ns,
                                       uint32_t sequence = 0)
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
  cloud.header.seq = sequence;
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

struct ScanFile
{
  uint64_t stamp_ns = 0;
  std::string path;
};

bool allDigits(const std::string &text)
{
  return !text.empty() && std::all_of(text.begin(), text.end(),
      [](unsigned char c) { return c >= '0' && c <= '9'; });
}

bool parseCanonicalScanName(const std::string &name, uint64_t *stamp_ns)
{
  if (name.compare(0, 6, "cloud_") != 0 || name.size() < 16 ||
      name.compare(name.size() - 4, 4, ".pcd") != 0)
    return false;
  const std::string stem = name.substr(6, name.size() - 10);
  const std::size_t separator = stem.find('_');
  if (separator == std::string::npos || stem.find('_', separator + 1) != std::string::npos)
    return false;
  const std::string seconds = stem.substr(0, separator);
  const std::string nanoseconds = stem.substr(separator + 1);
  if (!allDigits(seconds) || nanoseconds.size() != 9 || !allDigits(nanoseconds))
    return false;
  const uint64_t sec = std::stoull(seconds);
  const uint64_t nsec = std::stoull(nanoseconds);
  if (nsec >= 1000000000ULL || sec > (std::numeric_limits<uint64_t>::max() - nsec) / 1000000000ULL)
    return false;
  *stamp_ns = sec * 1000000000ULL + nsec;
  return true;
}

std::vector<ScanFile> loadCanonicalScans(const std::string &directory,
                                         std::size_t *ignored_names)
{
  DIR *dir = opendir(directory.c_str());
  if (!dir) throw std::runtime_error("cannot open scan directory: " + directory);
  std::vector<ScanFile> scans;
  *ignored_names = 0;
  while (dirent *entry = readdir(dir))
  {
    const std::string name(entry->d_name);
    if (name.compare(0, 6, "cloud_") != 0) continue;
    uint64_t stamp_ns = 0;
    if (!parseCanonicalScanName(name, &stamp_ns))
    {
      ++*ignored_names;
      continue;
    }
    const std::string path = directory + "/" + name;
    struct stat info;
    if (stat(path.c_str(), &info) != 0 || !S_ISREG(info.st_mode))
    {
      closedir(dir);
      throw std::runtime_error("canonical scan is not a readable regular file: " + path);
    }
    scans.push_back(ScanFile{stamp_ns, path});
  }
  closedir(dir);
  std::sort(scans.begin(), scans.end(), [](const ScanFile &a, const ScanFile &b) {
    return a.stamp_ns < b.stamp_ns;
  });
  for (std::size_t i = 1; i < scans.size(); ++i)
  {
    if (scans[i].stamp_ns == scans[i - 1].stamp_ns)
      throw std::runtime_error("duplicate canonical LiDAR timestamp: " +
                               std::to_string(scans[i].stamp_ns));
  }
  return scans;
}

uint64_t parseUnsignedNs(const std::string &value, const std::string &name)
{
  if (value.empty() || !allDigits(value)) throw std::runtime_error("invalid " + name);
  return std::stoull(value);
}

struct NdtReceipt
{
  bool present = false;
  bool converged = false;
  bool measurement_published_known = false;
  bool measurement_published = false;
  double fitness = std::numeric_limits<double>::quiet_NaN();
  double align_ms = std::numeric_limits<double>::quiet_NaN();
  int iterations = -1;
  std::string message;
};

class SequenceCapture
{
public:
  explicit SequenceCapture(bool admission_ack_enable = false)
      : admission_ack_enable_(admission_ack_enable) {}

  bool open(const std::string &receipt_path, const std::string &trajectory_path)
  {
    if (std::ifstream(receipt_path.c_str()).good() ||
        std::ifstream(trajectory_path.c_str()).good())
      throw std::runtime_error("refusing to overwrite P44 receipt or trajectory CSV");
    receipts_.open(receipt_path.c_str(), std::ios::out);
    trajectory_.open(trajectory_path.c_str(), std::ios::out);
    if (!receipts_ || !trajectory_) throw std::runtime_error("cannot create P44 output CSVs");
    receipts_ << "scan_index,scan_stamp_ns,pcd_path,point_count,max_point_offset_ns,imu_first_ns,imu_last_ns,ekf_imu_watermark_ns,ndt_imu_watermark_ns,ndt_diagnostic_ack,ndt_converged,ndt_fitness,ndt_iterations,ndt_align_ms,ekf_correction_ack,wall_publish_sec,wall_ndt_ack_sec,wall_ekf_ack_sec,ndt_measurement_published_known,ndt_measurement_published,ekf_ack_basis,ekf_ack_expected\n";
    trajectory_ << "source,stamp_ns,frame_id,child_frame_id,px,py,pz,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz,receipt_wall_sec\n";
    receipts_.flush();
    trajectory_.flush();
    return true;
  }

  void diagnosticCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr &message)
  {
    if (!message || message->header.stamp.isZero()) return;
    NdtReceipt receipt;
    for (const auto &status : message->status)
    {
      if (status.name != "dog_prior_map_ndt") continue;
      receipt.present = true;
      receipt.message = status.message;
      for (const auto &value : status.values)
      {
        if (value.key == "ndt_converged") receipt.converged = value.value == "true";
        else if (admission_ack_enable_ && value.key == "ndt_measurement_published")
        {
          if (receipt.measurement_published_known ||
              (value.value != "true" && value.value != "false"))
            throw std::runtime_error("R23 requires one explicit boolean ndt_measurement_published field");
          receipt.measurement_published_known = true;
          receipt.measurement_published = value.value == "true";
        }
        else if (value.key == "fitness_score")
        {
          try { receipt.fitness = std::stod(value.value); } catch (...) {}
        }
        else if (value.key == "align_time_ms")
        {
          try { receipt.align_ms = std::stod(value.value); } catch (...) {}
        }
        else if (value.key == "iterations")
        {
          try { receipt.iterations = std::stoi(value.value); } catch (...) {}
        }
      }
    }
    if (!receipt.present) return;
    if (admission_ack_enable_ && !receipt.measurement_published_known)
      throw std::runtime_error("R23 NDT diagnostic lacks ndt_measurement_published");
    const uint64_t stamp_ns = message->header.stamp.toNSec();
    if (!ndt_receipts_.emplace(stamp_ns, receipt).second) duplicate_ndt_diagnostic_ = true;
  }

  void ndtWatermarkCallback(const std_msgs::Header::ConstPtr &message)
  {
    if (!message) return;
    const uint64_t stamp = message->stamp.toNSec();
    if (stamp < latest_ndt_imu_ns_) watermark_order_error_ = true;
    latest_ndt_imu_ns_ = std::max(latest_ndt_imu_ns_, stamp);
  }

  void highRateCallback(const nav_msgs::Odometry::ConstPtr &message)
  {
    if (!message) return;
    recordOdom("EKF_HIGH_RATE", *message);
    const uint64_t stamp = message->header.stamp.toNSec();
    if (stamp < latest_ekf_imu_ns_) watermark_order_error_ = true;
    latest_ekf_imu_ns_ = std::max(latest_ekf_imu_ns_, stamp);
  }

  void correctedCallback(const nav_msgs::Odometry::ConstPtr &message)
  {
    if (!message) return;
    recordOdom("EKF_CORRECTED", *message);
    const uint64_t stamp = message->header.stamp.toNSec();
    latest_corrected_ns_ = std::max(latest_corrected_ns_, stamp);
    ++corrected_count_;
  }

  void ndtOdomCallback(const nav_msgs::Odometry::ConstPtr &message)
  {
    if (message) recordOdom("NDT", *message);
  }

  bool diagnostic(uint64_t stamp_ns, NdtReceipt *receipt) const
  {
    const auto it = ndt_receipts_.find(stamp_ns);
    if (it == ndt_receipts_.end()) return false;
    *receipt = it->second;
    return true;
  }

  uint64_t latestNdtImuNs() const { return latest_ndt_imu_ns_; }
  uint64_t latestEkfImuNs() const { return latest_ekf_imu_ns_; }
  uint64_t latestCorrectedNs() const { return latest_corrected_ns_; }
  uint64_t correctedCount() const { return corrected_count_; }
  bool duplicateDiagnostic() const { return duplicate_ndt_diagnostic_; }
  bool watermarkOrderError() const { return watermark_order_error_; }

  void writeReceipt(std::size_t index, const ScanFile &scan, std::size_t points,
                    uint32_t max_offset_ns, uint64_t imu_first_ns, uint64_t imu_last_ns,
                    uint64_t ekf_watermark_ns, uint64_t ndt_watermark_ns,
                    const NdtReceipt &ndt, bool correction_ack,
                    double wall_publish, double wall_ndt_ack, double wall_ekf_ack)
  {
    receipts_ << index << ',' << scan.stamp_ns << ',' << scan.path << ',' << points << ','
        << max_offset_ns << ',' << imu_first_ns << ',' << imu_last_ns << ','
        << ekf_watermark_ns << ',' << ndt_watermark_ns << ",1,"
        << (ndt.converged ? 1 : 0) << ',' << std::setprecision(17) << ndt.fitness << ','
        << ndt.iterations << ',' << ndt.align_ms << ',' << (correction_ack ? 1 : 0) << ','
        << wall_publish << ',' << wall_ndt_ack << ',' << wall_ekf_ack << ','
        << (ndt.measurement_published_known ? 1 : 0) << ',';
    if (ndt.measurement_published_known) receipts_ << (ndt.measurement_published ? 1 : 0);
    else receipts_ << "NA";
    receipts_ << ',' << (admission_ack_enable_ ? "EXPLICIT_MEASUREMENT_PUBLISHED" : "LEGACY_CONVERGED")
        << ',' << ((admission_ack_enable_ ? ndt.measurement_published : ndt.converged) ? 1 : 0) << '\n';
    receipts_.flush();
  }

private:
  void recordOdom(const char *source, const nav_msgs::Odometry &message)
  {
    const auto &p = message.pose.pose.position;
    const auto &q = message.pose.pose.orientation;
    const auto &v = message.twist.twist.linear;
    const auto &w = message.twist.twist.angular;
    trajectory_ << source << ',' << message.header.stamp.toNSec() << ','
        << message.header.frame_id << ',' << message.child_frame_id << ','
        << std::setprecision(17) << p.x << ',' << p.y << ',' << p.z << ','
        << q.x << ',' << q.y << ',' << q.z << ',' << q.w << ','
        << v.x << ',' << v.y << ',' << v.z << ','
        << w.x << ',' << w.y << ',' << w.z << ',' << ros::WallTime::now().toSec() << '\n';
    trajectory_.flush();
  }

  std::ofstream receipts_;
  std::ofstream trajectory_;
  std::map<uint64_t, NdtReceipt> ndt_receipts_;
  uint64_t latest_ndt_imu_ns_ = 0;
  uint64_t latest_ekf_imu_ns_ = 0;
  uint64_t latest_corrected_ns_ = 0;
  uint64_t corrected_count_ = 0;
  bool duplicate_ndt_diagnostic_ = false;
  bool watermark_order_error_ = false;
  bool admission_ack_enable_ = false;
};

bool waitFor(const std::function<bool()> &condition, double timeout_sec,
             const std::string &reason)
{
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(timeout_sec);
  while (ros::ok() && ros::WallTime::now() < deadline)
  {
    ros::spinOnce();
    if (condition()) return true;
    ros::WallDuration(0.002).sleep();
  }
  ROS_ERROR("P44 synchronization timeout: %s", reason.c_str());
  return false;
}

void runSequencePublisher(ros::NodeHandle &nh, ros::NodeHandle &pnh,
                          const std::string &imu_csv,
                          const std::string &lidar_frame,
                          const std::string &imu_frame,
                          const std::string &cloud_topic,
                          const std::string &imu_topic)
{
  std::string scan_directory, start_stamp_text, expected_boundary_text, imu_replay_start_text;
  std::string diagnostic_topic, ndt_odom_topic, high_rate_topic, corrected_topic;
  std::string ndt_imu_watermark_topic, receipt_csv_path, trajectory_csv_path;
  int scan_count = 0;
  bool r23_admission_ack_enable = false;
  double readiness_timeout_sec = 180.0, scan_ack_timeout_sec = 120.0;
  pnh.param<std::string>("scan_directory", scan_directory, "");
  pnh.param<std::string>("start_stamp_ns", start_stamp_text, "");
  pnh.param<std::string>("expected_boundary_imu_stamp_ns", expected_boundary_text, "");
  pnh.param<std::string>("imu_replay_start_stamp_ns", imu_replay_start_text, "");
  pnh.param("r23_admission_ack_enable", r23_admission_ack_enable, false);
  pnh.param("scan_count", scan_count, 0);
  pnh.param<std::string>("diagnostics_topic", diagnostic_topic, "/dog_livo/diagnostics");
  pnh.param<std::string>("ndt_odom_topic", ndt_odom_topic, "/dog_livo/ndt_odom");
  pnh.param<std::string>("ekf_high_rate_topic", high_rate_topic, "/dog_livo/odom_high_rate");
  pnh.param<std::string>("ekf_corrected_topic", corrected_topic, "/dog_livo/odom_corrected");
  pnh.param<std::string>("ndt_imu_watermark_topic", ndt_imu_watermark_topic,
                         "/newer_college/ndt_imu_watermark");
  pnh.param<std::string>("receipt_csv_path", receipt_csv_path, "");
  pnh.param<std::string>("trajectory_csv_path", trajectory_csv_path, "");
  pnh.param("readiness_timeout_sec", readiness_timeout_sec, 180.0);
  pnh.param("scan_ack_timeout_sec", scan_ack_timeout_sec, 120.0);
  if (scan_directory.empty() || scan_count <= 0 || start_stamp_text.empty() ||
      expected_boundary_text.empty() || diagnostic_topic.empty() || ndt_odom_topic.empty() ||
      high_rate_topic.empty() || corrected_topic.empty() || ndt_imu_watermark_topic.empty() ||
      receipt_csv_path.empty() || trajectory_csv_path.empty() ||
      !std::isfinite(readiness_timeout_sec) || readiness_timeout_sec <= 0.0 ||
      !std::isfinite(scan_ack_timeout_sec) || scan_ack_timeout_sec <= 0.0)
    throw std::runtime_error("incomplete P44 continuous publisher parameters");

  const uint64_t start_stamp_ns = parseUnsignedNs(start_stamp_text, "start_stamp_ns");
  const uint64_t imu_replay_start_ns = imu_replay_start_text.empty() ? start_stamp_ns :
      parseUnsignedNs(imu_replay_start_text, "imu_replay_start_stamp_ns");
  if (imu_replay_start_ns > start_stamp_ns)
    throw std::runtime_error("IMU replay start must not follow the first LiDAR scan");
  const uint64_t expected_boundary_ns = parseUnsignedNs(
      expected_boundary_text, "expected_boundary_imu_stamp_ns");
  std::size_t ignored_names = 0;
  const std::vector<ScanFile> catalog = loadCanonicalScans(scan_directory, &ignored_names);
  const auto start_it = std::lower_bound(catalog.begin(), catalog.end(), start_stamp_ns,
      [](const ScanFile &scan, uint64_t stamp) { return scan.stamp_ns < stamp; });
  if (start_it == catalog.end() || start_it->stamp_ns != start_stamp_ns)
    throw std::runtime_error("frozen P44 start timestamp is absent from canonical PCD names");
  if (static_cast<std::size_t>(catalog.end() - start_it) < static_cast<std::size_t>(scan_count))
    throw std::runtime_error("scan directory has fewer canonical timestamps than requested");
  const std::vector<ScanFile> scans(start_it, start_it + scan_count);
  for (std::size_t i = 1; i < scans.size(); ++i)
    if (scans[i].stamp_ns <= scans[i - 1].stamp_ns)
      throw std::runtime_error("selected P44 scan timestamps are not strictly increasing");

  const std::vector<ImuRecord> imu_records = loadImuCsv(imu_csv);
  const auto boundary_it = std::upper_bound(imu_records.begin(), imu_records.end(), imu_replay_start_ns,
      [](uint64_t stamp, const ImuRecord &record) { return stamp < record.stamp_ns; });
  if (boundary_it == imu_records.begin())
    throw std::runtime_error("P44 Ouster IMU has no sample at or before first scan");
  const std::size_t boundary_index = static_cast<std::size_t>(boundary_it - imu_records.begin() - 1);
  const ImuRecord &boundary = imu_records[boundary_index];
  if (boundary.stamp_ns != expected_boundary_ns || imu_replay_start_ns - boundary.stamp_ns > 20000000ULL)
    throw std::runtime_error("P44 first-scan IMU boundary differs from frozen P41 initialization");
  const uint64_t imu_preflight_end_ns = scans.back().stamp_ns + 150000000ULL;
  if (imu_records.back().stamp_ns <= imu_preflight_end_ns)
    throw std::runtime_error("P44 Ouster IMU does not bracket all selected scans");
  std::size_t covered_index = boundary_index;
  while (covered_index + 1 < imu_records.size() &&
         imu_records[covered_index + 1].stamp_ns <= imu_preflight_end_ns)
  {
    if (imu_records[covered_index + 1].stamp_ns - imu_records[covered_index].stamp_ns > 20000000ULL)
      throw std::runtime_error("P44 Ouster IMU gap exceeds frozen 20 ms Deskew coverage contract");
    ++covered_index;
  }
  if (imu_records[covered_index + 1].stamp_ns - imu_records[covered_index].stamp_ns > 20000000ULL)
    throw std::runtime_error("P44 Ouster IMU bracket interval exceeds frozen 20 ms contract");

  SequenceCapture capture(r23_admission_ack_enable);
  capture.open(receipt_csv_path, trajectory_csv_path);
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(imu_topic, 500, false);
  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_topic, 1, false);
  ros::Subscriber diagnostic_sub = nh.subscribe<diagnostic_msgs::DiagnosticArray>(
      diagnostic_topic, 1000, &SequenceCapture::diagnosticCallback, &capture);
  ros::Subscriber ndt_odom_sub = nh.subscribe<nav_msgs::Odometry>(
      ndt_odom_topic, 1000, &SequenceCapture::ndtOdomCallback, &capture);
  ros::Subscriber high_rate_sub = nh.subscribe<nav_msgs::Odometry>(
      high_rate_topic, 20000, &SequenceCapture::highRateCallback, &capture);
  ros::Subscriber corrected_sub = nh.subscribe<nav_msgs::Odometry>(
      corrected_topic, 1000, &SequenceCapture::correctedCallback, &capture);
  ros::Subscriber ndt_watermark_sub = nh.subscribe<std_msgs::Header>(
      ndt_imu_watermark_topic, 2000, &SequenceCapture::ndtWatermarkCallback, &capture);
  if (!waitFor([&]() {
        return imu_pub.getNumSubscribers() >= 2 && cloud_pub.getNumSubscribers() >= 1 &&
            diagnostic_sub.getNumPublishers() > 0 && ndt_odom_sub.getNumPublishers() > 0 &&
            high_rate_sub.getNumPublishers() > 0 && corrected_sub.getNumPublishers() > 0 &&
            ndt_watermark_sub.getNumPublishers() > 0;
      }, readiness_timeout_sec, "waiting for NDT map, EKF, and both IMU consumers"))
    throw std::runtime_error("P44 localization graph did not become ready");

  ROS_INFO("P44 ready: mode=CONTROL init=CONDITIONAL_BASE_FRAME/ORACLE_START_ONLY canonical_scans=%zu ignored_noncanonical=%zu selected=%d first_ns=%llu last_ns=%llu",
           catalog.size(), ignored_names, scan_count,
           static_cast<unsigned long long>(scans.front().stamp_ns),
           static_cast<unsigned long long>(scans.back().stamp_ns));
  ROS_INFO("IMU replay start_ns=%llu boundary_ns=%llu admission_ack=%d",
           static_cast<unsigned long long>(imu_replay_start_ns),
           static_cast<unsigned long long>(boundary.stamp_ns), r23_admission_ack_enable ? 1 : 0);

  std::size_t imu_cursor = boundary_index + 1;
  uint64_t last_imu_consumed_ns = boundary.stamp_ns;
  for (std::size_t scan_index = 0; scan_index < scans.size(); ++scan_index)
  {
    const ScanFile &scan = scans[scan_index];
    pcl::PCLPointCloud2 pcl_cloud;
    if (pcl::io::loadPCDFile(scan.path, pcl_cloud) != 0)
      throw std::runtime_error("failed to load selected P44 scan: " + scan.path);
    uint32_t max_offset_ns = 0;
    sensor_msgs::PointCloud2 cloud = makeTimedCloud(pcl_cloud, scan.stamp_ns,
        lidar_frame, max_offset_ns, static_cast<uint32_t>(scan_index + 1));
    if (max_offset_ns == 0 || max_offset_ns > 150000000U)
      throw std::runtime_error("selected P44 PCD point-time range violates [0,150 ms]");
    const uint64_t scan_end_ns = scan.stamp_ns + max_offset_ns;

    const std::size_t imu_begin = imu_cursor;
    uint64_t previous_imu_ns = last_imu_consumed_ns;
    while (imu_cursor < imu_records.size() && imu_records[imu_cursor].stamp_ns <= scan_end_ns)
    {
      const ImuRecord &record = imu_records[imu_cursor];
      if (record.stamp_ns <= previous_imu_ns || record.stamp_ns - previous_imu_ns > 20000000ULL)
        throw std::runtime_error("P44 IMU interval violates strict ordering/20 ms Deskew contract");
      imu_pub.publish(makeImuMessage(record, imu_frame));
      previous_imu_ns = record.stamp_ns;
      ++imu_cursor;
    }
    if (imu_cursor >= imu_records.size() || imu_records[imu_cursor].stamp_ns <= scan_end_ns)
      throw std::runtime_error("P44 IMU has no post-end bracket sample for scan");
    const ImuRecord &bracket = imu_records[imu_cursor];
    if (bracket.stamp_ns - previous_imu_ns > 20000000ULL)
      throw std::runtime_error("P44 post-end IMU bracket exceeds frozen 20 ms contract");
    imu_pub.publish(makeImuMessage(bracket, imu_frame));
    ++imu_cursor;
    last_imu_consumed_ns = bracket.stamp_ns;
    const uint64_t imu_first_ns = imu_begin < imu_cursor ? imu_records[imu_begin].stamp_ns : bracket.stamp_ns;
    const uint64_t imu_last_ns = bracket.stamp_ns;

    if (!waitFor([&]() {
          return capture.latestEkfImuNs() >= imu_last_ns &&
              capture.latestNdtImuNs() >= imu_last_ns && !capture.watermarkOrderError();
        }, scan_ack_timeout_sec, "waiting for EKF and NDT IMU consumer watermarks"))
      throw std::runtime_error("P44 IMU watermark handshake failed before cloud publication");

    const uint64_t corrected_before = capture.correctedCount();
    const double publish_wall = ros::WallTime::now().toSec();
    cloud_pub.publish(cloud);
    NdtReceipt ndt_receipt;
    if (!waitFor([&]() { return capture.diagnostic(scan.stamp_ns, &ndt_receipt); },
                 scan_ack_timeout_sec, "waiting for NDT diagnostic ack at exact scan timestamp"))
      throw std::runtime_error("P44 cloud was not acknowledged by NDT; stopping without sending later scans");
    const double ndt_ack_wall = ros::WallTime::now().toSec();
    bool correction_ack = false;
    double ekf_ack_wall = std::numeric_limits<double>::quiet_NaN();
    const bool measurement_expected = r23_admission_ack_enable ?
        ndt_receipt.measurement_published : ndt_receipt.converged;
    if (measurement_expected)
    {
      if (!waitFor([&]() {
            if (capture.correctedCount() <= corrected_before) return false;
            const uint64_t corrected_ns = capture.latestCorrectedNs();
            // The EKF publishes a ROS stamp derived from double seconds.
            // At Unix-scale timestamps, conversion back to integer ns can
            // differ from the consumed sensor stamp by a few 10s of ns.
            constexpr uint64_t kRosDoubleStampToleranceNs = 1000;
            return corrected_ns >= imu_last_ns ||
                imu_last_ns - corrected_ns <= kRosDoubleStampToleranceNs;
          }, scan_ack_timeout_sec, "waiting for OOSM-corrected EKF output at the consumed IMU watermark"))
        throw std::runtime_error("published NDT measurement was not acknowledged by EKF/OOSM");
      correction_ack = true;
      ekf_ack_wall = ros::WallTime::now().toSec();
    }
    capture.writeReceipt(scan_index + 1, scan, pcl_cloud.width * pcl_cloud.height,
        max_offset_ns, imu_first_ns, imu_last_ns, capture.latestEkfImuNs(),
        capture.latestNdtImuNs(), ndt_receipt, correction_ack,
        publish_wall, ndt_ack_wall, ekf_ack_wall);
    ROS_INFO("P44 frame=%zu/%zu stamp_ns=%llu points=%u ndt_ack=1 converged=%d iterations=%d align_ms=%.3f imu_watermark_ns=%llu correction_ack=%d",
             scan_index + 1, scans.size(), static_cast<unsigned long long>(scan.stamp_ns),
             pcl_cloud.width * pcl_cloud.height, ndt_receipt.converged ? 1 : 0,
             ndt_receipt.iterations, ndt_receipt.align_ms,
             static_cast<unsigned long long>(imu_last_ns), correction_ack ? 1 : 0);
    if (capture.duplicateDiagnostic() || capture.watermarkOrderError())
      throw std::runtime_error("P44 received duplicate NDT diagnostics or nonmonotonic consumer watermarks");
  }
  ros::spinOnce();
  ROS_INFO("P44 complete: published=%zu acknowledged=%zu EKF_high_rate_rows=%llu NDT_odom_rows=%llu EKF_corrected_rows=%llu",
           scans.size(), scans.size(),
           static_cast<unsigned long long>(capture.latestEkfImuNs()),
           static_cast<unsigned long long>(capture.latestNdtImuNs()),
           static_cast<unsigned long long>(capture.correctedCount()));
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
    std::string ndt_ready_topic, scan_directory;
    int scan_sec = -1, scan_nsec = -1;
    double readiness_timeout_sec = 120.0;
    pnh.param<std::string>("scan_pcd", pcd_path, "");
    pnh.param<std::string>("imu_csv", imu_csv, "");
    pnh.param<std::string>("scan_directory", scan_directory, "");
    pnh.param<std::string>("lidar_frame", lidar_frame, "os1_lidar");
    pnh.param<std::string>("imu_frame", imu_frame, "os1_imu");
    pnh.param<std::string>("cloud_topic", cloud_topic, "/newer_college/raw_points");
    pnh.param<std::string>("imu_topic", imu_topic, "/newer_college/imu");
    if (!scan_directory.empty())
    {
      if (imu_csv.empty() || lidar_frame.empty() || imu_frame.empty() ||
          cloud_topic.empty() || imu_topic.empty())
        throw std::runtime_error("missing P44 dataset, frame, or topic parameters");
      runSequencePublisher(nh, pnh, imu_csv, lidar_frame, imu_frame,
                           cloud_topic, imu_topic);
      return 0;
    }
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
    ROS_FATAL("Newer College scan publisher failed: %s", error.what());
    return 1;
  }
  return 0;
}
