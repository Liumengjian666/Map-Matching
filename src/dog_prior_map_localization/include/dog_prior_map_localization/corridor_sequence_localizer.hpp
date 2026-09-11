#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <deque>
#include <vector>

namespace dog_prior_map_localization
{

// Lightweight one-dimensional temporal matcher for repetitive corridors.
// The map is indexed by the configured corridor axis (x by default).  Each
// index stores a coarse local 3-D occupancy fingerprint; online hypotheses
// are scalar positions, so this module does not duplicate NDT/FAST-LIVO2.
class CorridorSequenceLocalizer
{
public:
  struct Hypothesis
  {
    double s = 0.0;
    double probability = 0.0;
  };

  void configure(bool enabled, double bin_size, int sequence_length,
                 int max_hypotheses, double search_radius,
                 double axis_min, double axis_max);
  void buildMap(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &map);
  bool ready() const { return ready_; }

  // Returns a temporally consistent longitudinal coordinate.  y/z and R are
  // supplied by NDT and are intentionally not modified here.
  double update(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &scan_body,
                const Eigen::Matrix3d &R, const Eigen::Vector3d &pose,
                double predicted_s, bool degenerate);

  const std::vector<Hypothesis> &hypotheses() const { return hypotheses_; }
  double last_score() const { return last_score_; }

private:
  using Descriptor = Eigen::VectorXf;
  Descriptor descriptorForMapBin(int index) const;
  Descriptor descriptorForObservation(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &scan,
                                      const Eigen::Matrix3d &R,
                                      const Eigen::Vector3d &pose,
                                      double candidate_s) const;
  double descriptorDistance(const Descriptor &a, const Descriptor &b) const;
  void normalizeHypotheses();

  bool enabled_ = false;
  bool ready_ = false;
  double bin_size_ = 0.5;
  int sequence_length_ = 8;
  int max_hypotheses_ = 5;
  double search_radius_ = 8.0;
  double axis_min_ = 0.0;
  double axis_max_ = 0.0;
  int descriptor_bins_ = 0;
  int longitudinal_bins_ = 9;
  int lateral_bins_ = 7;
  int height_bins_ = 5;
  std::vector<Descriptor> map_descriptors_;
  std::deque<Descriptor> observation_history_;
  std::vector<Hypothesis> hypotheses_;
  bool has_last_prediction_ = false;
  double last_predicted_s_ = 0.0;
  double last_score_ = 0.0;
};

}  // namespace dog_prior_map_localization
