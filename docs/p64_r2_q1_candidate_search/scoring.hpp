#pragma once

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <unordered_map>

namespace q1 {
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
using Pose = Eigen::Matrix4f;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Key = std::array<int, 3>;  // z, y, x (PCL voxel order)
constexpr double outlierRatio = 0.55;
constexpr double alpha = 0.001;
constexpr int associationMinimum = 6;

inline Key voxel(const Point& p, float inverse) {
  return {int(std::floor(p.z * inverse)), int(std::floor(p.y * inverse)),
          int(std::floor(p.x * inverse))};
}

inline Cloud::Ptr downsample(const Cloud::ConstPtr& input) {
  pcl::VoxelGrid<Point> filter;
  filter.setLeafSize(0.1f, 0.1f, 0.1f);
  filter.setInputCloud(input);
  Cloud::Ptr result(new Cloud);
  filter.filter(*result);
  return result;
}

struct ParkCell {
  Eigen::Vector3d mean;
  Eigen::Matrix3d inverse;
  double signal;
};

class ParkMap {
 public:
  explicit ParkMap(const Cloud& target) {
    struct Moments {
      Eigen::Vector3d sum = Eigen::Vector3d::Zero();
      Eigen::Matrix3d products = Eigen::Matrix3d::Zero();
      int count = 0;
    };
    std::map<Key, Moments> map;
    for (auto& point : target) {
      auto& values = map[voxel(point, 1)];
      const Eigen::Vector3d xyz = point.getVector3fMap().cast<double>();
      values.sum += xyz;
      values.products.noalias() += xyz * xyz.transpose();
      ++values.count;
    }
    for (auto& entry : map) {
      auto& values = entry.second;
      if (values.count < associationMinimum) {
        continue;
      }
      const Eigen::Vector3d mean = values.sum / values.count;
      const Eigen::Matrix3d covariance =
          (values.products - values.count * mean * mean.transpose()) / (values.count - 1);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(covariance);
      if (eig.info() != Eigen::Success || eig.eigenvalues()[2] <= 0) {
        continue;
      }
      const auto eigenvalues = eig.eigenvalues().cwiseMax(eig.eigenvalues()[2] * 0.01).eval();
      cells_.emplace(entry.first, ParkCell{
          mean, eig.eigenvectors() * eigenvalues.cwiseInverse().asDiagonal() * eig.eigenvectors().transpose(),
          (1 - outlierRatio) / std::sqrt(std::pow(2 * std::acos(-1.0), 3) * eigenvalues.prod())});
    }
  }

  double logLikelihood(const Point& p, bool& associated) const {
    const auto it = cells_.find(voxel(p, 1));
    associated = it != cells_.end();
    if (!associated) {
      return std::log(outlierRatio);
    }
    const auto& cell = it->second;
    const Eigen::Vector3d delta = p.getVector3fMap().cast<double>() - cell.mean;
    return std::log(outlierRatio + cell.signal *
        std::exp(-0.5 * delta.dot(cell.inverse * delta)));
  }

 private:
  std::map<Key, ParkCell> cells_;
};

struct Contribution {
  int cell;
  double mahalanobis;
  double score;
};
struct PointScore {
  Point transformed;
  double total;
  double parkLog;
  bool parkAssociated;
  std::vector<Contribution> neighborhoods;
};
struct Score {
  double native = 0;
  double park = 0;
  std::size_t associated = 0;
  std::size_t parkAssociated = 0;
  std::size_t total = 0;
};

class Scorer : public pclomp::NormalDistributionsTransform<Point, Point> {
 public:
  explicit Scorer(const Cloud::ConstPtr& target) : park_(*target) {
    setResolution(1.0f);
    setTransformationEpsilon(0.01);
    setNeighborhoodSearchMethod(pclomp::DIRECT7);
    setInputTarget(target);
    setNumThreads(1);
    for (const auto& item : target_cells_.getLeaves()) {
      ids_.emplace(&item.second, item.first);
    }
  }

  const TargetGrid::Map& cells() { return target_cells_.getLeaves(); }

  Score fixed(const Cloud& source, const Pose& pose, std::vector<PointScore>* details = nullptr) const {
    Cloud transformed;
    pcl::transformPointCloud(source, transformed, pose);
    const Eigen::Matrix<float, 4, 6> zeroJ = Eigen::Matrix<float, 4, 6>::Zero();
    const Eigen::Matrix<float, 24, 6> zeroH = Eigen::Matrix<float, 24, 6>::Zero();
    Vec6 unusedGradient = Vec6::Zero();
    Mat6 unusedHessian = Mat6::Zero();
    std::vector<TargetGridLeafConstPtr> neighbors;
    neighbors.reserve(7);
    Score score;
    score.total = source.size();
    if (details) {
      details->clear();
      details->reserve(source.size());
    }
    for (const Point& p : transformed) {
      target_cells_.getNeighborhoodAtPoint7(p, neighbors);
      score.associated += !neighbors.empty();
      bool parkAssociated;
      const double park = park_.logLikelihood(p, parkAssociated);
      score.parkAssociated += parkAssociated;
      score.park += park;
      PointScore report{p, 0, park, parkAssociated, {}};
      for (const auto* cell : neighbors) {
        const Eigen::Vector3d delta = p.getVector3fMap().cast<double>() - cell->getMean();
        const Eigen::Matrix3d invCov = cell->getInverseCov();
        const double value = updateDerivatives(unusedGradient, unusedHessian, zeroJ, zeroH,
                                               delta, invCov, false);
        report.total += value;
        if (details) {
          const Eigen::Matrix<float, 1, 4> d(delta.x(), delta.y(), delta.z(), 0);
          Eigen::Matrix4f c = Eigen::Matrix4f::Zero();
          c.topLeftCorner<3, 3>() = invCov.cast<float>();
          report.neighborhoods.push_back({ids_.at(cell), d.dot(d * c), value});
        }
      }
      score.native += report.total;
      if (details) {
        details->push_back(std::move(report));
      }
    }
    return score;
  }

  double oracle(const Cloud::ConstPtr& source, const Pose& pose, int threads, Mat6* curvature = nullptr) {
    setInputSource(source);
    setNumThreads(threads);
    Cloud transformed;
    pcl::transformPointCloud(*source, transformed, pose);
    Vec6 params;
    params.head<3>() = pose.block<3, 1>(0, 3).cast<double>();
    params.tail<3>() = pose.block<3, 3>(0, 0).eulerAngles(0, 1, 2).cast<double>();
    Vec6 gradient;
    Mat6 hessian;
    const double score = computeDerivatives(gradient, hessian, transformed, params, curvature != nullptr);
    if (curvature) {
      *curvature = hessian;
    }
    return score;
  }

 private:
  ParkMap park_;
  std::unordered_map<TargetGridLeafConstPtr, int> ids_;
};
}  // namespace q1
