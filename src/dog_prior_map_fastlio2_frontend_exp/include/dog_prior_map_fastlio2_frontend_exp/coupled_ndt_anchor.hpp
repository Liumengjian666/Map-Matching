#pragma once
#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_shadow.hpp"

namespace dog_prior_map_fastlio2_frontend_exp {
// An experiment-only pose reference, NOT another filter or an absolute prior.
struct CoupledAnchorState {
  Eigen::Matrix4d origin = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d prediction = Eigen::Matrix4d::Identity();
  uint64_t origin_stamp_ns = 0, propagated_stamp_ns = 0, event_stamp_ns = 0, invalidated_stamp_ns = 0;
  bool valid = false, frozen = false;
  Eigen::Matrix<double,6,2> weak_basis = Eigen::Matrix<double,6,2>::Zero();
  int weak_dimension = 0, contributions = 0;
  double nominal_sum = 0, alternative_sum = 0;
  std::string status = "MISSING";
};
struct CoupledAnchorReceipt {
  Eigen::Matrix4d origin = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d prediction = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double,6,2> weak_basis = Eigen::Matrix<double,6,2>::Zero();
  uint64_t anchor_stamp_ns = 0, propagated_stamp_ns = 0;
  int weak_dimension = 0, contributions = 0;
  bool valid = false, frozen = false, evaluated = false, diagnostic_valid = false;
  double age_s = 0, nominal_cost = 0, alternative_cost = 0;
  double nominal_sum = 0, alternative_sum = 0, mean_advantage = 0;
  double weak_fraction = 0, strong_fraction = 0;
  std::string status = "NOT_EVALUATED";
};
// Fixed version-0 contract; changes require a separately frozen experiment.
constexpr double kCoupledAnchorLifetimeS = 2.0;
constexpr double kCoupledAnchorMeanMargin = .01;
constexpr double kCoupledAnchorRelativeGain = .10;

void advanceCoupledAnchor(CoupledAnchorState*, uint64_t stamp_ns,
    const Eigen::Matrix4d& causal_imu_interval);
void settleCoupledAnchor(CoupledAnchorState*, uint64_t stamp_ns,
    const Eigen::Matrix4d& corrected_lidar, bool stable_ordinary, bool feedback_used);
bool coupledAnchorChart(const Eigen::Matrix4d& candidate,
    const Eigen::Matrix4d& reference, CoupledVector6* eta);
CoupledEventResult runAnchoredCoupledNdtShadow(const Eigen::Matrix4f& nominal,
    const Eigen::Matrix4f& prediction, uint64_t stamp_ns, std::size_t source_count,
    bool nominal_effective, const CoupledNdtBackend&, const CoupledEventConfig&,
    PendingCandidate*, const Eigen::Matrix4d& causal_imu_interval,
    CoupledAnchorState*, CoupledAnchorReceipt*);
}  // namespace dog_prior_map_fastlio2_frontend_exp
