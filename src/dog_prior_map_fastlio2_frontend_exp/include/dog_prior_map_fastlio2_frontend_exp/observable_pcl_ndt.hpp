#pragma once

#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_shadow.hpp"

#include <Eigen/Core>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>

namespace dog_prior_map_fastlio2_frontend_exp {

// Exposes the exact PCL NDT derivative and score kernels to the existing R6
// local refinement. It owns no cloud, map, or optimizer state of its own.
template <typename PointT>
class ObservablePclNdt : public pcl::NormalDistributionsTransform<PointT, PointT> {
 public:
  using Base = pcl::NormalDistributionsTransform<PointT, PointT>;
  using Cloud = pcl::PointCloud<PointT>;
  using TargetGrid = typename Base::TargetGrid;

  std::array<float, 3> targetGridLeafSizeMeters() const {
    const Eigen::Vector3f size = this->target_cells_.getLeafSize();
    return {{size.x(), size.y(), size.z()}};
  }

  // This is the same computeDerivatives() path already used by the R6 P9
  // backend, with its complete native Euler carrier supplied by jointPullback.
  CoupledNativeJet nativeJet(const typename Cloud::ConstPtr& source,
      const Eigen::Matrix4f& pose, const CoupledVector6& parameters) {
    CoupledNativeJet result;
    if (!source || source->empty() || !pose.allFinite() || !parameters.allFinite())
      return result;
    configureScoreConstants();
    Cloud transformed;
    pcl::transformPointCloud(*source, transformed, pose);
    CoupledVector6 native_parameters = parameters;
    result.score_sum = this->computeDerivatives(result.score_gradient,
        result.score_hessian, transformed, native_parameters, true);
    result.valid = std::isfinite(result.score_sum) &&
        result.score_gradient.allFinite() && result.score_hessian.allFinite();
    return result;
  }

  // Exact PCL score kernel evaluated against this optimizer's current target
  // grid; intentionally value-only and does not construct another NDT object.
  double dynamicScore(const typename Cloud::ConstPtr& source,
                      const Eigen::Matrix4f& pose) {
    if (!source || source->empty() || !pose.allFinite())
      return std::numeric_limits<double>::quiet_NaN();
    configureScoreConstants();
    Cloud transformed;
    pcl::transformPointCloud(*source, transformed, pose);
    double score = 0.0;
    std::vector<typename TargetGrid::LeafConstPtr> leaves;
    std::vector<float> distances;
    for (const PointT& point : transformed) {
      distances.clear();
      this->target_cells_.radiusSearch(point, this->resolution_, leaves, distances);
      const Eigen::Vector3d x(point.x, point.y, point.z);
      for (const auto& leaf : leaves) {
        const Eigen::Vector3d residual = x - leaf->getMean();
        const double exponential = std::exp(-this->gauss_d2_ *
            residual.dot(leaf->getInverseCov() * residual) / 2.0);
        const double guard = this->gauss_d2_ * exponential;
        if (std::isfinite(guard) && guard <= 1.0 && guard >= 0.0)
          score += -this->gauss_d1_ * exponential;
      }
    }
    return score;
  }

 private:
  void configureScoreConstants() {
    const double c1 = 10.0 * (1.0 - this->outlier_ratio_);
    const double c2 = this->outlier_ratio_ /
        std::pow(static_cast<double>(this->resolution_), 3.0);
    const double d3 = -std::log(c2);
    this->gauss_d1_ = -std::log(c1 + c2) - d3;
    this->gauss_d2_ = -2.0 * std::log(
        (-std::log(c1 * std::exp(-0.5) + c2) - d3) / this->gauss_d1_);
    this->point_gradient_.setZero();
    this->point_gradient_.template block<3, 3>(0, 0).setIdentity();
    this->point_hessian_.setZero();
  }
};

}  // namespace dog_prior_map_fastlio2_frontend_exp
