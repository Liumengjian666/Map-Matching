#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <sstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "dog_prior_map_localization/solid_place_recognition.hpp"

namespace dog_prior_map_localization
{

class SolidShadowNode
{
public:
  SolidShadowNode() : private_nh_("~")
  {
    const std::string database_path = parameter("database_path", std::string());
    csv_path_ = parameter("csv_path", std::string());
    input_topic_ = parameter("input_topic", std::string("/dog_livo/filtered_points"));
    top_k_ = static_cast<std::size_t>(std::max(1, parameter("top_k", 5)));
    if (database_path.empty() || csv_path_.empty())
      throw std::runtime_error("database_path and csv_path are required when SOLiD shadow is enabled");
    std::string reason;
    if (!adapter_.loadDatabase(database_path, reason))
      throw std::runtime_error("SOLiD database load failed: " + reason);

    csv_.open(csv_path_, std::ios::out | std::ios::trunc);
    if (!csv_.is_open()) throw std::runtime_error("cannot open SOLiD CSV: " + csv_path_);
    writeHeader();
    subscriber_ = nh_.subscribe(input_topic_, 100,
        &SolidShadowNode::cloudCallback, this);
    ROS_INFO("[P11 SOLiD shadow] database=%s entries=%zu descriptor_dim=%d topic=%s csv=%s top_k=%zu; query results never feed NDT/EKF",
             database_path.c_str(), adapter_.databaseSize(), adapter_.descriptorSize(),
             input_topic_.c_str(), csv_path_.c_str(), top_k_);
  }

private:
  template <typename T>
  T parameter(const std::string &name, const T &fallback) const
  {
    T value = fallback;
    private_nh_.param(name, value, fallback);
    return value;
  }

  void writeHeader()
  {
    csv_ << "sequence,stamp_ns,frame_id,source_hash,input_source_points,solid_fov_range_points,"
            "solid_descriptor_points,descriptor_status,descriptor_ms,search_ms,"
            "scored_candidates,topk_reference_pose_spread_m,topk_nonlocal_count_5m,callback_compute_ms";
    for (std::size_t rank = 1; rank <= top_k_; ++rank)
    {
      csv_ << ",top" << rank << "_id,top" << rank << "_similarity,top" << rank
           << "_reference_scan_header_stamp,top" << rank << "_map_x,top" << rank << "_map_y,top" << rank
           << "_map_z,top" << rank << "_qx,top" << rank << "_qy,top" << rank
           << "_qz,top" << rank << "_qw";
    }
    csv_ << '\n';
    csv_.flush();
  }

  // Diagnostic byte carrier, identical to the existing NDT source hash.
  static std::string sourceHash(const pcl::PointCloud<pcl::PointXYZ> &cloud)
  {
    uint64_t hash = 1469598103934665603ULL;
    const auto mix = [&hash](uint32_t value) {
      for (int shift = 0; shift < 32; shift += 8) {
        hash ^= (value >> shift) & 0xffU;
        hash *= 1099511628211ULL;
      }
    };
    mix(cloud.width); mix(cloud.height); mix(cloud.is_dense ? 1U : 0U);
    mix(static_cast<uint32_t>(cloud.size()));
    for (const auto &point : cloud.points) {
      for (float coordinate : {point.x, point.y, point.z}) {
        uint32_t bits = 0;
        std::memcpy(&bits, &coordinate, sizeof(bits));
        mix(bits);
      }
    }
    std::ostringstream out;
    out << std::hex << hash;
    return out.str();
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &message)
  {
    if (!message) return;
    const auto callback_start = std::chrono::steady_clock::now();
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*message, cloud);
    const std::string source_hash = sourceHash(cloud);
    const SolidQueryResult result = adapter_.query(cloud, top_k_);
    double spread_m = 0.0;
    std::size_t nonlocal_count = 0;
    if (!result.top.empty())
    {
      const Eigen::Vector3d first = result.top.front().map_T_reference.block<3, 1>(0, 3);
      for (std::size_t i = 1; i < result.top.size(); ++i)
      {
        const double distance = (result.top[i].map_T_reference.block<3, 1>(0, 3) - first).norm();
        spread_m = std::max(spread_m, distance);
        if (distance > 5.0) ++nonlocal_count;
      }
    }

    std::string safe_frame = message->header.frame_id;
    std::replace(safe_frame.begin(), safe_frame.end(), ',', '_');
    const double callback_compute_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - callback_start).count();
    csv_ << message->header.seq << ',' << message->header.stamp.toNSec() << ',' << safe_frame << ','
         << source_hash << ','
         << result.descriptor.input_points << ',' << result.descriptor.fov_range_points << ','
         << result.descriptor.descriptor_points << ',' << result.status << ','
         << std::setprecision(17) << result.descriptor.descriptor_ms << ',' << result.search_ms << ','
         << result.scored_candidates << ',' << spread_m << ',' << nonlocal_count << ','
         << callback_compute_ms;
    for (std::size_t i = 0; i < top_k_; ++i)
    {
      if (i >= result.top.size())
      {
        csv_ << ",,,,,,,,,,";
        continue;
      }
      const SolidRankedCandidate &candidate = result.top[i];
      const Eigen::Quaterniond q(candidate.map_T_reference.block<3, 3>(0, 0));
      csv_ << ',' << candidate.id << ',' << candidate.similarity << ',' << candidate.stamp_sec
           << ',' << candidate.map_T_reference(0, 3) << ',' << candidate.map_T_reference(1, 3)
           << ',' << candidate.map_T_reference(2, 3) << ',' << q.x() << ',' << q.y()
           << ',' << q.z() << ',' << q.w();
    }
    csv_ << '\n';
    csv_.flush();
    if (!csv_.good()) ROS_ERROR_THROTTLE(5.0, "[P11 SOLiD shadow] result CSV write failed");
    if (!result.descriptor.valid)
      ROS_WARN_THROTTLE(5.0, "[P11 SOLiD shadow] invalid descriptor: %s", result.status.c_str());
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber subscriber_;
  SolidPlaceRecognitionAdapter adapter_;
  std::ofstream csv_;
  std::string csv_path_;
  std::string input_topic_;
  std::size_t top_k_ = 5;
};

}  // namespace dog_prior_map_localization

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dog_prior_map_solid_shadow");
  try
  {
    dog_prior_map_localization::SolidShadowNode node;
    ros::spin();
  }
  catch (const std::exception &exception)
  {
    ROS_FATAL("[P11 SOLiD shadow] %s", exception.what());
    return 2;
  }
  return 0;
}
