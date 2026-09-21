#include "dog_prior_map_localization/core/oosm_replay_planner.hpp"

#include <cmath>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <limits>
#include <string>

namespace
{

using dog_prior_map_localization::FilterStateSnapshot;
using dog_prior_map_localization::ImuSample;
using dog_prior_map_localization::OosmPlanStatus;
using dog_prior_map_localization::OosmReplayPlan;
using dog_prior_map_localization::OosmReplayPlanner;
using dog_prior_map_localization::StateHistory;

void require(bool condition, const std::string &message)
{
  if (!condition)
  {
    std::cerr << "contract failure: " << message << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

FilterStateSnapshot snapshot(double stamp)
{
  FilterStateSnapshot value;
  value.stamp = stamp;
  value.p.x() = stamp;
  return value;
}

ImuSample imu(double stamp)
{
  ImuSample value;
  value.stamp = stamp;
  value.acc = Eigen::Vector3d(1.0, 2.0, 3.0);
  value.gyro = Eigen::Vector3d(0.1, 0.2, 0.3);
  return value;
}

void requireStatus(const OosmReplayPlan &plan,
                   OosmPlanStatus expected,
                   const std::string &case_name)
{
  require(plan.status == expected,
          case_name + " status=" +
              dog_prior_map_localization::oosmPlanStatusName(plan.status));
}

}  // namespace

int main()
{
  const OosmReplayPlanner planner;

  // Case 1: a normal delayed measurement finds the latest snapshot at or
  // before the measurement and selects the bounded replay interval.
  StateHistory history;
  history.insertMonotonic(snapshot(9.90));
  history.insertMonotonic(snapshot(10.00));
  history.insertMonotonic(snapshot(10.10));
  std::deque<ImuSample> imu_history{imu(9.90), imu(9.95), imu(10.00), imu(10.05), imu(10.10)};
  OosmReplayPlan plan = planner.makePlan(history, imu_history, 10.00, 10.10, 0.02, 0.05);
  requireStatus(plan, OosmPlanStatus::kReady, "case1");
  require(plan.rollback_index == 1 && std::abs(plan.rollback_stamp - 10.00) < 1e-12,
          "case1 rollback snapshot");
  require(plan.replay_samples.size() == 2 &&
              std::abs(plan.replay_samples[0].stamp - 10.05) < 1e-12 &&
              std::abs(plan.replay_samples[1].stamp - 10.10) < 1e-12,
          "case1 replay selection");

  // Case 2: future measurements remain explicitly rejected by the planner;
  // the callback-level bounded deferral policy remains outside this module.
  requireStatus(planner.makePlan(history, imu_history, 10.11, 10.10, 0.02, 0.05),
                OosmPlanStatus::kFutureMeasurement, "case2");

  // Case 3: no snapshot satisfies snapshot_stamp <= measurement_stamp.
  StateHistory no_history;
  no_history.insertMonotonic(snapshot(10.01));
  requireStatus(planner.makePlan(no_history, imu_history, 10.00, 10.10, 0.02, 0.05),
                OosmPlanStatus::kNoHistory, "case3");

  // Case 4: the rollback snapshot exists, but its alignment exceeds the
  // configured maximum (including the existing 1e-9 boundary allowance).
  StateHistory old_history;
  old_history.insertMonotonic(snapshot(9.00));
  requireStatus(planner.makePlan(old_history, imu_history, 10.00, 10.10, 0.50, 0.05),
                OosmPlanStatus::kAlignmentTooLarge, "case4");

  // Case 5: lower bound is strict (with the existing epsilon), upper bound
  // is inclusive.  The sample exactly at rollback is excluded and the sample
  // at state_now is included.
  StateHistory boundary_history;
  boundary_history.insertMonotonic(snapshot(10.00));
  std::deque<ImuSample> boundary_imu{
      imu(10.00), imu(10.00 + 0.5e-9), imu(10.00 + 2.0e-9), imu(10.10)};
  plan = planner.makePlan(boundary_history, boundary_imu, 10.00, 10.10, 0.02, 0.2);
  requireStatus(plan, OosmPlanStatus::kReady, "case5");
  require(plan.replay_samples.size() == 2 &&
              std::abs(plan.replay_samples.front().stamp - (10.00 + 2.0e-9)) < 1e-12 &&
              std::abs(plan.replay_samples.back().stamp - 10.10) < 1e-12,
          "case5 boundary interval");

  // Case 6: out-of-order input is sorted as in the old callback; duplicate
  // timestamps then retain the old invalid zero-dt behavior.
  std::deque<ImuSample> out_of_order{imu(10.10), imu(10.05)};
  plan = planner.makePlan(history, out_of_order, 10.00, 10.10, 0.02, 0.2);
  requireStatus(plan, OosmPlanStatus::kReady, "case6 out-of-order");
  require(plan.replay_samples.size() == 2 &&
              plan.replay_samples[0].stamp < plan.replay_samples[1].stamp,
          "case6 sorted replay");
  std::deque<ImuSample> duplicate{imu(10.05), imu(10.05)};
  requireStatus(planner.makePlan(history, duplicate, 10.00, 10.10, 0.02, 0.2),
                OosmPlanStatus::kReplayIncomplete, "case6 duplicate");

  // Case 7: a non-positive or oversized interval is rejected before any
  // caller-side state mutation.
  requireStatus(planner.makePlan(history, std::deque<ImuSample>{imu(10.10)},
                                10.00, 10.10, 0.02, 0.05),
                OosmPlanStatus::kReplayIncomplete, "case7 oversized dt");
  ImuSample invalid_dt = imu(10.05);
  invalid_dt.acc.setConstant(std::numeric_limits<double>::quiet_NaN());
  requireStatus(planner.makePlan(history, std::deque<ImuSample>{invalid_dt},
                                10.00, 10.10, 0.2, 0.05),
                OosmPlanStatus::kReplayIncomplete, "case7 invalid sample");

  // Case 8: an empty replay is valid under the existing callback semantics.
  StateHistory exact_history;
  exact_history.insertMonotonic(snapshot(10.00));
  plan = planner.makePlan(exact_history, std::deque<ImuSample>{},
                          10.00, 10.00, 0.02, 0.05);
  requireStatus(plan, OosmPlanStatus::kReady, "case8 empty replay");
  require(plan.replay_samples.empty(), "case8 empty replay samples");

  requireStatus(planner.makePlan(history, imu_history,
                                std::numeric_limits<double>::quiet_NaN(),
                                10.10, 0.02, 0.05),
                OosmPlanStatus::kInvalidTimestamp, "invalid timestamp");

  std::cout << "OOSM_REPLAY_PLANNER_CONTRACT_PASS" << std::endl;
  return EXIT_SUCCESS;
}
