#include "dog_prior_map_localization/skf_visual_measurement.hpp"

#include <Eigen/Eigenvalues>

#include <cmath>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace dog_prior_map_localization
{
namespace
{
std::vector<std::string> split(const std::string& line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) fields.push_back(field);
  return fields;
}

uint64_t parseUnsigned(const std::string& value)
{
  std::size_t consumed = 0;
  if (value.empty() || value.front() == '-') throw std::runtime_error("invalid visual timestamp");
  const uint64_t result = std::stoull(value, &consumed);
  if (consumed != value.size()) throw std::runtime_error("invalid visual timestamp field");
  return result;
}

bool properRotation(const Eigen::Matrix3d& rotation)
{
  return rotation.allFinite() &&
      (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() <= 1e-5 &&
      std::abs(rotation.determinant() - 1.0) <= 1e-5;
}
}  // namespace

void SkfVisualMeasurementCatalog::load(const std::string& path)
{
  if (path.empty()) throw std::runtime_error("SKF visual covariance CSV path is empty");
  std::ifstream input(path);
  if (!input) throw std::runtime_error("SKF visual measurement input unavailable: " + path);
  std::string line;
  if (!std::getline(input, line)) throw std::runtime_error("SKF visual measurement CSV is empty");
  const auto names = split(line);
  std::map<std::string, std::size_t> columns;
  for (std::size_t i = 0; i < names.size(); ++i)
    if (!columns.emplace(names[i], i).second)
      throw std::runtime_error("duplicate SKF visual measurement column: " + names[i]);
  const auto has = [&columns](const std::string& name) { return columns.count(name) != 0; };
  for (const char* required : {"scan_ns", "ref_image_ns", "cur_image_ns", "valid",
                               "depth_max_point_ns", "depth_max_imu_ns", "max_imu_ns",
                               "imu_dx", "imu_dy", "imu_dz",
                               "imu_qx", "imu_qy", "imu_qz", "imu_qw", "pnp_inliers",
                               "inlier_ratio", "reprojection_rmse_px", "grid_occupancy",
                               "hull_fraction", "skf_covariance_valid", "skf_covariance_model"})
    if (!has(required)) throw std::runtime_error(std::string("missing SKF measurement field: ") + required);
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      if (!has("imu_rel_cov_" + std::to_string(i) + std::to_string(j)))
        throw std::runtime_error("missing SKF covariance matrix field");

  std::map<uint64_t, SkfVisualMeasurement> loaded;
  std::set<uint64_t> seen;
  while (std::getline(input, line))
  {
    if (line.empty()) continue;
    const auto fields = split(line);
    const auto value = [&](const std::string& name) -> const std::string& {
      const std::size_t index = columns.at(name);
      if (index >= fields.size()) throw std::runtime_error("short SKF visual row");
      return fields[index];
    };
    const uint64_t scan_ns = parseUnsigned(value("scan_ns"));
    if (scan_ns == 0 || !seen.insert(scan_ns).second)
      throw std::runtime_error("duplicate or zero SKF visual scan timestamp");
    SkfVisualMeasurement row;
    row.scan_ns = scan_ns;
    const bool valid = parseUnsigned(value("valid")) == 1;
    if (!valid) continue;
    row.reference_ns = parseUnsigned(value("ref_image_ns"));
    row.current_ns = parseUnsigned(value("cur_image_ns"));
    row.depth_max_point_ns = parseUnsigned(value("depth_max_point_ns"));
    row.depth_max_imu_ns = parseUnsigned(value("depth_max_imu_ns"));
    row.max_imu_ns = parseUnsigned(value("max_imu_ns"));
    if (row.reference_ns == 0 || row.current_ns <= row.reference_ns ||
        row.current_ns > scan_ns || row.depth_max_point_ns > row.reference_ns ||
        row.depth_max_imu_ns > row.reference_ns ||
        row.max_imu_ns > scan_ns || parseUnsigned(value("skf_covariance_valid")) != 1)
    {
      loaded.erase(scan_ns);
      continue;
    }

    row.relative_pose.block<3, 1>(0, 3) = Eigen::Vector3d(
        std::stod(value("imu_dx")), std::stod(value("imu_dy")), std::stod(value("imu_dz")));
    Eigen::Quaterniond quaternion(std::stod(value("imu_qw")), std::stod(value("imu_qx")),
                                  std::stod(value("imu_qy")), std::stod(value("imu_qz")));
    if (!row.relative_pose.block<3, 1>(0, 3).allFinite() || !quaternion.coeffs().allFinite() ||
        quaternion.norm() < 1e-12)
    {
      loaded.erase(scan_ns);
      continue;
    }
    row.relative_pose.block<3, 3>(0, 0) = quaternion.normalized().toRotationMatrix();
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j)
        row.covariance(i, j) = std::stod(value("imu_rel_cov_" + std::to_string(i) + std::to_string(j)));
    row.covariance = 0.5 * (row.covariance + row.covariance.transpose());
    Eigen::SelfAdjointEigenSolver<Matrix6d> covariance_eigen(row.covariance, Eigen::EigenvaluesOnly);
    if (!properRotation(row.relative_pose.block<3, 3>(0, 0)) || !row.covariance.allFinite() ||
        covariance_eigen.info() != Eigen::Success || !covariance_eigen.eigenvalues().allFinite() ||
        covariance_eigen.eigenvalues().minCoeff() <= 0.0)
    {
      loaded.erase(scan_ns);
      continue;
    }
    row.pnp_inliers = static_cast<uint32_t>(parseUnsigned(value("pnp_inliers")));
    row.inlier_ratio = std::stod(value("inlier_ratio"));
    row.reprojection_rmse_px = std::stod(value("reprojection_rmse_px"));
    row.grid_occupancy = std::stod(value("grid_occupancy"));
    row.hull_fraction = std::stod(value("hull_fraction"));
    row.covariance_model = value("skf_covariance_model");
    if (row.pnp_inliers < 20 || !std::isfinite(row.inlier_ratio) ||
        !std::isfinite(row.reprojection_rmse_px) || !std::isfinite(row.grid_occupancy) ||
        !std::isfinite(row.hull_fraction) || row.covariance_model.empty())
    {
      loaded.erase(scan_ns);
      continue;
    }
    loaded[scan_ns] = row;
  }
  if (loaded.empty()) throw std::runtime_error("SKF visual CSV has no valid finite covariance rows");
  measurements_.swap(loaded);
}

const SkfVisualMeasurement* SkfVisualMeasurementCatalog::find(uint64_t scan_ns) const
{
  const auto found = measurements_.find(scan_ns);
  return found == measurements_.end() ? nullptr : &found->second;
}

}  // namespace dog_prior_map_localization
