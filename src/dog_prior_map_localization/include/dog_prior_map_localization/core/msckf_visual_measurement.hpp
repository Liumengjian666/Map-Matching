#pragma once
#include "dog_prior_map_localization/core/estimator_types.hpp"
#include <cstdint>
#include <string>
#include <vector>

namespace dog_prior_map_localization {
// Independent implementation of published MSCKF/OpenVINS mathematics. No
// upstream GPL source is copied; this is not the complete OpenVINS estimator.
struct PixelObservation { std::size_t clone=0; Eigen::Vector2d pixel=Eigen::Vector2d::Zero(); };
struct RectifiedCamera {
  Eigen::Matrix3d K=Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R_imu_camera=Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_imu_camera=Eigen::Vector3d::Zero();
};
struct ReprojectionLinearization {
  bool valid=false;
  Eigen::Vector2d prediction=Eigen::Vector2d::Zero();
  Eigen::Matrix<double,2,6> H_pose=Eigen::Matrix<double,2,6>::Zero();
  Eigen::Matrix<double,2,3> H_point=Eigen::Matrix<double,2,3>::Zero();
};
ReprojectionLinearization linearizeReprojection(const VisualPoseClone&,const RectifiedCamera&,
                                               const Eigen::Vector3d& point);
bool triangulateVisualTrack(const std::vector<VisualPoseClone>&,const RectifiedCamera&,
                           const std::vector<PixelObservation>&,Eigen::Vector3d* point,std::string* reason);
bool eliminateVisualFeature(const Eigen::MatrixXd& H_feature,const Eigen::MatrixXd& H_state,
                            const Eigen::VectorXd& residual,Eigen::MatrixXd* H_out,Eigen::VectorXd* r_out);
// Exact stochastic augmentation. The rectangular factor is deliberately NOT
// required to be full row rank: cloned states are initially perfectly correlated.
bool appendPoseCloneFactor(const Eigen::MatrixXd& factor,Eigen::MatrixXd* augmented);
bool marginalizeOldestPoseCloneFactor(const Eigen::MatrixXd& factor,Eigen::MatrixXd* retained);
bool propagateClonedFactor(const Eigen::MatrixXd& factor,const Matrix15d& F,
                          const Eigen::MatrixXd& process_root,Eigen::MatrixXd* propagated);
bool josephClonedFactor(const Eigen::MatrixXd& factor,const Eigen::MatrixXd& K,
                       const Eigen::MatrixXd& H,const Eigen::MatrixXd& measurement_root,Eigen::MatrixXd* updated);
// Explicit common-noise coordinates: dx=L eta, e=B eta + V epsilon.
// The caller MUST supply B from a defined causal time/noise model, not assume
// independent interpolation errors. Returns constrained gain and posterior root.
bool correlatedVisualJoseph(const Eigen::MatrixXd& L,const Eigen::MatrixXd& B,
                            const Eigen::MatrixXd& V,const Eigen::MatrixXd& H,
                            const Eigen::MatrixXd& state_selector,
                            Eigen::MatrixXd* gain,Eigen::MatrixXd* updated);
bool causalVisualTime(uint64_t observation_ns,uint64_t target_ns);
void linearizeCausalImu(const Eigen::Matrix3d& end_rotation,const Eigen::Vector3d& acceleration,
    const Eigen::Vector3d& angular_rate,double dt,Matrix15d* F,Eigen::Matrix<double,15,6>* G);
}  // namespace dog_prior_map_localization
