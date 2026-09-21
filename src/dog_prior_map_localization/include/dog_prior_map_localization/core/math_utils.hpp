#pragma once

#include <algorithm>

#include <Eigen/Dense>

namespace dog_prior_map_localization
{

// Return the skew-symmetric matrix whose product with x equals v.cross(x).
inline Eigen::Matrix3d skew(const Eigen::Vector3d &v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(),
       v.z(), 0.0, -v.x(),
      -v.y(), v.x(), 0.0;
  return m;
}

// Limit a correction vector without changing its direction.
inline Eigen::Vector3d limitVector(const Eigen::Vector3d &v, double max_norm)
{
  const double n = v.norm();
  if (max_norm > 0.0 && n > max_norm)
  {
    return v * (max_norm / std::max(n, 1e-12));
  }
  return v;
}

}  // namespace dog_prior_map_localization
