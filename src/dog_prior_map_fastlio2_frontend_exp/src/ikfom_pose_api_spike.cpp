#include "dog_prior_map_fastlio2_frontend_exp/ikfom_pose_api_spike.hpp"

#include <omp.h>

// This is the only translation unit in this package allowed to include this
// pinned header: it defines non-inline free functions.
#include <use-ikfom.hpp>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>

namespace dog_prior_map_fastlio2_frontend_exp {
namespace {

MTK_BUILD_MANIFOLD(PoseMeasurement,
((vect3, position))
((SO3, rotation))
);

typedef esekfom::esekf<state_ikfom, 12, input_ikfom, PoseMeasurement, 6>
    PoseFilter;

PoseMeasurement poseModel(state_ikfom& state, bool& valid) {
  valid = true;
  return PoseMeasurement(state.pos, state.rot);
}

Eigen::Matrix<double, 6, state_ikfom::DOF> poseJacobian(
    state_ikfom&, bool& valid) {
  valid = true;
  Eigen::Matrix<double, 6, state_ikfom::DOF> h =
      Eigen::Matrix<double, 6, state_ikfom::DOF>::Zero();
  h.template block<3, 3>(0, MTK::getStartIdx(&state_ikfom::pos)).setIdentity();
  h.template block<3, 3>(3, MTK::getStartIdx(&state_ikfom::rot)).setIdentity();
  return h;
}

Eigen::Matrix<double, 6, 6> poseNoiseJacobian(state_ikfom&, bool& valid) {
  valid = true;
  return Eigen::Matrix<double, 6, 6>::Identity();
}

void enforceFixedExtrinsicConstraint(
    state_ikfom& state,
    Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF>& covariance,
    const SO3& calibrated_rotation,
    const vect3& calibrated_translation) {
  const int rotation_index = MTK::getStartIdx(&state_ikfom::offset_R_L_I);
  const int translation_index = MTK::getStartIdx(&state_ikfom::offset_T_L_I);
  const int fixed_indices[] = {rotation_index, translation_index};

  state.offset_R_L_I = calibrated_rotation;
  state.offset_T_L_I = calibrated_translation;
  for (int index : fixed_indices) {
    covariance.block(index, 0, 3, state_ikfom::DOF).setZero();
    covariance.block(0, index, state_ikfom::DOF, 3).setZero();
    covariance.block<3, 3>(index, index).diagonal().setConstant(1e-12);
  }
}

bool fail(std::string* why, const char* message) {
  if (why) *why = message;
  return false;
}

}  // namespace

bool runIkfomPoseApiSpike(std::string* failure_reason) {
  if (failure_reason) failure_reason->clear();

  state_ikfom initial;
  initial.pos = vect3(Eigen::Vector3d(0.0, 0.0, 0.0));
  initial.rot = SO3::Identity();
  const SO3 calibrated_rotation(
      Eigen::AngleAxisd(0.08, Eigen::Vector3d::UnitX()));
  const vect3 calibrated_translation(Eigen::Vector3d(0.12, -0.03, 0.05));
  initial.offset_R_L_I = calibrated_rotation;
  initial.offset_T_L_I = calibrated_translation;

  Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF> covariance =
      Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF>::Identity();
  const int position_index = MTK::getStartIdx(&state_ikfom::pos);
  const int rotation_index = MTK::getStartIdx(&state_ikfom::rot);
  const int extrinsic_rotation_index =
      MTK::getStartIdx(&state_ikfom::offset_R_L_I);
  const int extrinsic_translation_index =
      MTK::getStartIdx(&state_ikfom::offset_T_L_I);
  covariance.block<3, 3>(position_index, extrinsic_translation_index) =
      0.2 * Eigen::Matrix3d::Identity();
  covariance.block<3, 3>(extrinsic_translation_index, position_index) =
      0.2 * Eigen::Matrix3d::Identity();
  covariance.block<3, 3>(rotation_index, extrinsic_rotation_index) =
      0.15 * Eigen::Matrix3d::Identity();
  covariance.block<3, 3>(extrinsic_rotation_index, rotation_index) =
      0.15 * Eigen::Matrix3d::Identity();

  if (covariance.block<3, 3>(position_index, extrinsic_translation_index).norm() ==
      0.0) {
    return fail(failure_reason, "adversarial pose/extrinsic covariance was not injected");
  }
  enforceFixedExtrinsicConstraint(initial, covariance, calibrated_rotation,
                                  calibrated_translation);
  if (covariance.block<3, 3>(position_index, extrinsic_translation_index).norm() !=
          0.0 ||
      covariance.block<3, 3>(rotation_index, extrinsic_rotation_index).norm() !=
          0.0) {
    return fail(failure_reason, "fixed-extrinsic constraint did not clear cross covariance");
  }

  double limits[state_ikfom::DOF];
  std::fill(limits, limits + state_ikfom::DOF, 100.0);
  PoseFilter committed(initial, covariance);
  committed.init(get_f, df_dx, df_dw, poseModel, poseJacobian,
                 poseNoiseJacobian, 4, limits);
  PoseFilter shadow(committed.get_x(), committed.get_P());
  shadow.init(get_f, df_dx, df_dw, poseModel, poseJacobian,
              poseNoiseJacobian, 4, limits);

  const SO3 measurement_rotation(
      Eigen::AngleAxisd(0.18, Eigen::Vector3d::UnitZ()));
  PoseMeasurement measurement(vect3(Eigen::Vector3d(0.35, -0.08, 0.12)),
                              measurement_rotation);
  Eigen::Matrix<double, 6, 6> measurement_noise =
      0.05 * Eigen::Matrix<double, 6, 6>::Identity();
  shadow.update_iterated(measurement, measurement_noise);

  const state_ikfom& updated = shadow.get_x();
  const auto& updated_covariance = shadow.get_P();
  if (!updated.pos.allFinite() || !updated.rot.coeffs().allFinite() ||
      !updated.offset_R_L_I.coeffs().allFinite() ||
      !updated.offset_T_L_I.allFinite() || !updated.vel.allFinite() ||
      !updated.bg.allFinite() || !updated.ba.allFinite() ||
      !updated.grav.vec.allFinite() || !updated_covariance.allFinite()) {
    return fail(failure_reason, "generic pose update produced non-finite filter data");
  }
  if ((updated_covariance - updated_covariance.transpose()).cwiseAbs().maxCoeff() >
      1e-8 || updated_covariance.diagonal().minCoeff() < -1e-10) {
    return fail(failure_reason, "generic pose update violated covariance postconditions");
  }
  if ((updated.offset_R_L_I.coeffs().array() !=
       calibrated_rotation.coeffs().array()).any() ||
      (updated.offset_T_L_I.array() != calibrated_translation.array()).any()) {
    return fail(failure_reason, "generic pose update changed calibrated extrinsic values");
  }
  if (std::abs(updated.grav.vec.norm() - 9.809) > 1e-8) {
    return fail(failure_reason, "generic pose update changed the fixed S2 gravity norm");
  }
  if ((updated.pos - initial.pos).norm() < 1e-5) {
    return fail(failure_reason, "generic pose measurement did not correct position");
  }
  if (updated_covariance.block<3, 3>(position_index, extrinsic_translation_index).norm() >
          1e-10 ||
      updated_covariance.block<3, 3>(rotation_index, extrinsic_rotation_index).norm() >
          1e-10) {
    return fail(failure_reason, "generic pose update reintroduced fixed-extrinsic cross covariance");
  }
  return true;
}

}  // namespace dog_prior_map_fastlio2_frontend_exp
