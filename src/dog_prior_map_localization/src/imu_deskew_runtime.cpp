#include "dog_prior_map_localization/dog_prior_map_ekf_node.hpp"

#include "dog_prior_map_localization/core/imu_deskew.hpp"

#include <sensor_msgs/PointField.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace dog_prior_map_localization
{
namespace
{
constexpr double kPointTimeToleranceSec = 1e-6;
constexpr double kStateSafetyMaxVelocityMps = 40.0;
constexpr double kStateSafetyMaxAccelerationMps2 = 100.0;

const sensor_msgs::PointField *findField(const sensor_msgs::PointCloud2 &cloud,
                                         const std::string &name)
{
  const auto it = std::find_if(cloud.fields.begin(), cloud.fields.end(),
      [&name](const sensor_msgs::PointField &field) { return field.name == name; });
  return it == cloud.fields.end() ? nullptr : &(*it);
}

bool readFloatField(const sensor_msgs::PointCloud2 &cloud,
                    const sensor_msgs::PointField &field,
                    std::size_t row,
                    std::size_t column,
                    float &value)
{
  if (field.datatype != sensor_msgs::PointField::FLOAT32 || field.count != 1 ||
      cloud.is_bigendian || column >= cloud.width || row >= cloud.height ||
      field.offset + sizeof(float) > cloud.point_step)
    return false;
  const std::size_t byte_offset = row * cloud.row_step + column * cloud.point_step + field.offset;
  if (byte_offset + sizeof(float) > cloud.data.size()) return false;
  std::memcpy(&value, cloud.data.data() + byte_offset, sizeof(float));
  return true;
}

void writeFloatField(sensor_msgs::PointCloud2 &cloud,
                     const sensor_msgs::PointField &field,
                     std::size_t row,
                     std::size_t column,
                     float value)
{
  const std::size_t byte_offset = row * cloud.row_step + column * cloud.point_step + field.offset;
  std::memcpy(cloud.data.data() + byte_offset, &value, sizeof(float));
}

double percentile(std::vector<double> values, double fraction)
{
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  const std::size_t index = std::min(values.size() - 1,
      static_cast<std::size_t>(std::floor(fraction * static_cast<double>(values.size() - 1))));
  std::nth_element(values.begin(), values.begin() + index, values.end());
  return values[index];
}
}  // namespace

void DogPriorMapEkfNode::imuDeskewCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  if (!imu_deskew_enable_ || !msg) return;
  std::lock_guard<std::mutex> lock(mutex_);
  const uint64_t scan_index = ++imu_deskew_scan_index_;
  ++imu_deskew_raw_cloud_received_count_;
  const double scan_start = msg->header.stamp.toSec();
  const std::size_t point_count = static_cast<std::size_t>(msg->width) * msg->height;
  const auto *x_field = findField(*msg, "x");
  const auto *y_field = findField(*msg, "y");
  const auto *z_field = findField(*msg, "z");
  const auto *time_field = findField(*msg, deskew_time_field_);
  if (!std::isfinite(scan_start) || msg->header.frame_id != deskew_expected_lidar_frame_ ||
      point_count == 0 || msg->point_step == 0 || msg->row_step < msg->width * msg->point_step ||
      msg->data.size() < static_cast<std::size_t>(msg->row_step) * msg->height ||
      !x_field || !y_field || !z_field || !time_field ||
      x_field->datatype != sensor_msgs::PointField::FLOAT32 ||
      y_field->datatype != sensor_msgs::PointField::FLOAT32 ||
      z_field->datatype != sensor_msgs::PointField::FLOAT32 ||
      time_field->datatype != sensor_msgs::PointField::FLOAT32 ||
      x_field->count != 1 || y_field->count != 1 || z_field->count != 1 ||
      time_field->count != 1 || msg->is_bigendian)
  {
    writeImuDeskewDiagnostic(scan_index, "REJECTED", "invalid_cloud_layout_or_frame",
        scan_start, scan_start, scan_start, point_count, 0,
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
        0.0, 0, 0.0, 0.0, 0.0,
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
    return;
  }
  if (last_imu_deskew_input_stamp_ >= 0.0 && scan_start <= last_imu_deskew_input_stamp_ + 1e-9)
  {
    writeImuDeskewDiagnostic(scan_index, "REJECTED", "nonmonotonic_cloud_stamp",
        scan_start, scan_start, scan_start, point_count, 0,
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
        0.0, 0, 0.0, 0.0, 0.0,
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
    return;
  }

  double min_offset = std::numeric_limits<double>::infinity();
  double max_offset = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < point_count; ++i)
  {
    const std::size_t row = i / msg->width;
    const std::size_t column = i % msg->width;
    float point_time = 0.0f;
    if (!readFloatField(*msg, *time_field, row, column, point_time) ||
        !std::isfinite(point_time))
    {
      writeImuDeskewDiagnostic(scan_index, "REJECTED", "invalid_point_time_field",
          scan_start, scan_start, scan_start, point_count, 0,
          std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
          0.0, 0, 0.0, 0.0, 0.0,
          std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
      return;
    }
    min_offset = std::min(min_offset, static_cast<double>(point_time));
    max_offset = std::max(max_offset, static_cast<double>(point_time));
  }
  const double scan_end = scan_start + max_offset;
  if (!std::isfinite(min_offset) || !std::isfinite(max_offset) ||
      min_offset < -kPointTimeToleranceSec || max_offset < 0.0 ||
      max_offset > deskew_max_scan_duration_sec_)
  {
    writeImuDeskewDiagnostic(scan_index, "REJECTED", "point_time_outside_scan_contract",
        scan_start, scan_end, scan_start, point_count, 0, min_offset, max_offset,
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
        0.0, 0, 0.0, 0.0, 0.0,
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
    return;
  }
  last_imu_deskew_input_stamp_ = scan_start;
  if (pending_imu_deskew_clouds_.size() >= deskew_max_pending_clouds_)
  {
    writeImuDeskewDiagnostic(scan_index, "REJECTED", "pending_cloud_queue_full",
        scan_start, scan_end, scan_start, point_count, 0, min_offset, max_offset,
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
        0.0, 0, 0.0, 0.0, 0.0,
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
    return;
  }

  PendingImuDeskewCloud pending;
  pending.scan_index = scan_index;
  pending.msg = msg;
  pending.scan_start = scan_start;
  pending.scan_end = scan_end;
  pending.point_time_min = min_offset;
  pending.point_time_max = max_offset;
  pending.point_count = point_count;
  pending_imu_deskew_clouds_.push_back(pending);
  max_pending_imu_deskew_clouds_size_ = std::max(
      max_pending_imu_deskew_clouds_size_, pending_imu_deskew_clouds_.size());
  processReadyImuDeskewCloudsLocked();
}

void DogPriorMapEkfNode::processReadyImuDeskewCloudsLocked()
{
  while (!pending_imu_deskew_clouds_.empty())
  {
    const PendingImuDeskewCloud pending = pending_imu_deskew_clouds_.front();
    if (std::isfinite(gravity_init_completion_stamp_) &&
        pending.scan_start < gravity_init_completion_stamp_ - kPointTimeToleranceSec)
    {
      pending_imu_deskew_clouds_.pop_front();
      writeImuDeskewDiagnostic(pending.scan_index, "REJECTED", "PREINIT_REJECTED",
          pending.scan_start, pending.scan_end, pending.scan_start,
          pending.point_count, 0, pending.point_time_min, pending.point_time_max,
          std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
          0.0, 0, 0.0, 0.0, 0.0,
          std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
      continue;
    }
    ImuDeskewCoverage coverage;
    std::string reason;
    const bool covered = inspectImuDeskewCoverage(state_history_, pending.scan_start,
        pending.scan_end, deskew_max_imu_gap_sec_, coverage, reason);
    if (!covered && (reason == "no_state_history" || reason == "waiting_for_end_coverage"))
      return;

    pending_imu_deskew_clouds_.pop_front();
    if (!covered)
    {
      writeImuDeskewDiagnostic(pending.scan_index, "REJECTED", reason,
          pending.scan_start, pending.scan_end,
          deskew_reference_time_ == "end" ? pending.scan_end : pending.scan_start,
          pending.point_count, 0, pending.point_time_min, pending.point_time_max,
          coverage.history_first_stamp, coverage.history_last_stamp,
          coverage.max_gap_sec, coverage.samples_in_scan, 0.0, 0.0, 0.0,
          std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
      continue;
    }
    if (!pending.msg || pending.msg->width == 0)
      continue;

    const auto deskew_start = std::chrono::steady_clock::now();
    const auto deskew_elapsed_ms = [&deskew_start]() {
      return std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - deskew_start).count();
    };

    const auto *x_field = findField(*pending.msg, "x");
    const auto *y_field = findField(*pending.msg, "y");
    const auto *z_field = findField(*pending.msg, "z");
    const auto *time_field = findField(*pending.msg, deskew_time_field_);
    if (!x_field || !y_field || !z_field || !time_field)
    {
      writeImuDeskewDiagnostic(pending.scan_index, "REJECTED", "point_field_disappeared",
          pending.scan_start, pending.scan_end, pending.scan_start,
          pending.point_count, 0, pending.point_time_min, pending.point_time_max,
          coverage.history_first_stamp, coverage.history_last_stamp, coverage.max_gap_sec,
          coverage.samples_in_scan, 0.0, 0.0, 0.0,
          std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
          deskew_elapsed_ms());
      continue;
    }

    const double reference_stamp = deskew_reference_time_ == "end" ?
        pending.scan_end : pending.scan_start;
    ImuDeskewPose reference_pose;
    if (!interpolateImuDeskewPose(state_history_, reference_stamp,
            deskew_max_imu_gap_sec_, g_, imu_kinematics_config_, reference_pose, reason,
            pending.scan_end))
    {
      writeImuDeskewDiagnostic(pending.scan_index, "REJECTED", "reference_" + reason,
          pending.scan_start, pending.scan_end, reference_stamp,
          pending.point_count, 0, pending.point_time_min, pending.point_time_max,
          coverage.history_first_stamp, coverage.history_last_stamp, coverage.max_gap_sec,
          coverage.samples_in_scan, 0.0, 0.0, 0.0,
          std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
          deskew_elapsed_ms());
      continue;
    }

    sensor_msgs::PointCloud2 output = *pending.msg;
    if (deskew_reference_time_ == "end") output.header.stamp.fromSec(reference_stamp);
    std::vector<double> displacements;
    displacements.reserve(pending.point_count);
    double displacement_sum = 0.0;
    double displacement_max = 0.0;
    double max_velocity = reference_pose.v.norm();
    double max_acc_world = reference_pose.acc_world.norm();
    double max_gyro = reference_pose.gyro_unbiased.norm();
    double max_imu_source_stamp = reference_pose.latest_source_stamp;
    bool valid = true;
    bool post_scan_end_imu_used = false;
    std::string failure_reason;

    for (std::size_t i = 0; i < pending.point_count; ++i)
    {
      const std::size_t row = i / pending.msg->width;
      const std::size_t column = i % pending.msg->width;
      float x = 0.0f, y = 0.0f, z = 0.0f, point_time = 0.0f;
      if (!readFloatField(*pending.msg, *x_field, row, column, x) ||
          !readFloatField(*pending.msg, *y_field, row, column, y) ||
          !readFloatField(*pending.msg, *z_field, row, column, z) ||
          !readFloatField(*pending.msg, *time_field, row, column, point_time) ||
          !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
          !std::isfinite(point_time))
      {
        valid = false;
        failure_reason = "nonfinite_point_or_time";
        break;
      }
      const double point_stamp = pending.scan_start + static_cast<double>(point_time);
      if (point_stamp < pending.scan_start - kPointTimeToleranceSec ||
          point_stamp > pending.scan_end + kPointTimeToleranceSec)
      {
        valid = false;
        failure_reason = "point_stamp_outside_scan";
        break;
      }
      ImuDeskewPose point_pose;
      if (!interpolateImuDeskewPose(state_history_, point_stamp,
              deskew_max_imu_gap_sec_, g_, imu_kinematics_config_, point_pose, reason,
              pending.scan_end))
      {
        valid = false;
        failure_reason = "point_" + reason;
        break;
      }
      max_imu_source_stamp = std::max(max_imu_source_stamp, point_pose.latest_source_stamp);
      if (point_pose.latest_source_stamp > pending.scan_end + 1e-9)
      {
        valid = false;
        post_scan_end_imu_used = true;
        failure_reason = "post_scan_end_imu_sample_used";
        break;
      }
      max_velocity = std::max(max_velocity, point_pose.v.norm());
      max_acc_world = std::max(max_acc_world, point_pose.acc_world.norm());
      max_gyro = std::max(max_gyro, point_pose.gyro_unbiased.norm());
      if (!std::isfinite(max_velocity) || !std::isfinite(max_acc_world) ||
          !std::isfinite(max_gyro) || max_velocity > kStateSafetyMaxVelocityMps ||
          max_acc_world > kStateSafetyMaxAccelerationMps2)
      {
        valid = false;
        failure_reason = "state_safety_bound_exceeded";
        break;
      }

      Eigen::Vector3d point_reference;
      if (!transformPointToReferenceLidar(point_pose, reference_pose, T_imu_lidar_,
              Eigen::Vector3d(x, y, z), point_reference, reason))
      {
        valid = false;
        failure_reason = reason;
        break;
      }
      const double displacement = (point_reference - Eigen::Vector3d(x, y, z)).norm();
      if (!std::isfinite(displacement))
      {
        valid = false;
        failure_reason = "nonfinite_point_displacement";
        break;
      }
      displacements.push_back(displacement);
      displacement_sum += displacement;
      displacement_max = std::max(displacement_max, displacement);
      writeFloatField(output, *x_field, row, column, static_cast<float>(point_reference.x()));
      writeFloatField(output, *y_field, row, column, static_cast<float>(point_reference.y()));
      writeFloatField(output, *z_field, row, column, static_cast<float>(point_reference.z()));
      if (deskew_reference_time_ == "end")
        writeFloatField(output, *time_field, row, column,
                        static_cast<float>(point_stamp - reference_stamp));
    }

    if (!valid || displacements.size() != pending.point_count)
    {
      writeImuDeskewDiagnostic(pending.scan_index, "REJECTED", failure_reason,
          pending.scan_start, pending.scan_end, reference_stamp,
          pending.point_count, 0, pending.point_time_min, pending.point_time_max,
          coverage.history_first_stamp, coverage.history_last_stamp, coverage.max_gap_sec,
          coverage.samples_in_scan, max_velocity, max_acc_world, max_gyro,
          std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
          deskew_elapsed_ms(), max_imu_source_stamp, post_scan_end_imu_used);
      continue;
    }

    const double displacement_mean = displacement_sum / static_cast<double>(displacements.size());
    const double displacement_median = percentile(displacements, 0.5);
    const double displacement_p95 = percentile(displacements, 0.95);
    pub_imu_deskew_cloud_.publish(output);
    writeImuDeskewDiagnostic(pending.scan_index, "PUBLISHED", "covered_and_deskewed",
        pending.scan_start, pending.scan_end, reference_stamp,
        pending.point_count, static_cast<std::size_t>(output.width) * output.height,
        pending.point_time_min, pending.point_time_max,
        coverage.history_first_stamp, coverage.history_last_stamp, coverage.max_gap_sec,
        coverage.samples_in_scan, max_velocity, max_acc_world, max_gyro,
        displacement_mean, displacement_median, displacement_p95, displacement_max,
        deskew_elapsed_ms(), max_imu_source_stamp, post_scan_end_imu_used);
  }
}

void DogPriorMapEkfNode::writeImuDeskewDiagnostic(uint64_t scan_index,
                                                  const std::string &status,
                                                  const std::string &reason,
                                                  double scan_start,
                                                  double scan_end,
                                                  double reference_stamp,
                                                  std::size_t point_count_in,
                                                  std::size_t point_count_out,
                                                  double point_time_min,
                                                  double point_time_max,
                                                  double history_first_stamp,
                                                  double history_last_stamp,
                                                  double max_state_gap,
                                                  std::size_t state_samples,
                                                  double max_velocity,
                                                  double max_acc_world,
                                                  double max_gyro,
                                                  double displacement_mean,
                                                  double displacement_median,
                                                  double displacement_p95,
                                                  double displacement_max,
                                                  double deskew_processing_ms,
                                                  double max_imu_source_stamp,
                                                  bool post_scan_end_imu_used)
{
  if (!imu_deskew_csv_.is_open()) return;
  imu_deskew_csv_ << scan_index << "," << status << "," << reason << ","
      << scan_start << "," << scan_end << "," << reference_stamp << ","
      << point_count_in << "," << point_count_out << ","
      << point_time_min << "," << point_time_max << ","
      << history_first_stamp << "," << history_last_stamp << ","
      << max_state_gap << "," << state_samples << ","
      << max_velocity << "," << max_acc_world << "," << max_gyro << ","
      << displacement_mean << "," << displacement_median << ","
      << displacement_p95 << "," << displacement_max << ","
      << deskew_processing_ms << ","
      << (deskew_reference_time_ == "start" ? "seconds_from_scan_start" :
                                             "seconds_from_output_reference")
      << ",0," << max_imu_source_stamp << ","
      << (post_scan_end_imu_used ? 1 : 0) << "\n";
  imu_deskew_csv_.flush();
}

}  // namespace dog_prior_map_localization
