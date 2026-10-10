#include "dog_prior_map_localization/core/oosm_fixed_lag_scheduler.hpp"
#include "dog_prior_map_localization/core/oosm_replay_planner.hpp"

#include <cmath>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <string>
#include <vector>

namespace
{
using namespace dog_prior_map_localization;

void require(bool value, const std::string &what)
{
  if (!value) { std::cerr << "fixed-lag contract failure: " << what << std::endl; std::exit(1); }
}

std::vector<uint64_t> releaseOrder(const std::vector<int> &arrival_order,
    double arrival_state, double *first_event_release)
{
  OosmFixedLagScheduler scheduler;
  require(scheduler.configure(0.15, 8), "configure");
  const double base = 1517157219.0;
  const double measurement_stamps[] = {base + 0.693280, base + 0.694000};
  for (int i : arrival_order)
    require(scheduler.enqueue(static_cast<uint64_t>(i + 1), measurement_stamps[i],
                              arrival_state, 500.0 + i) == FixedLagEnqueueStatus::kQueued,
            "pre-deadline enqueue");
  require(scheduler.advance(arrival_state).ready.empty(), "no early release");
  const double imu_nodes[] = {base + 0.824208, base + 0.829216, base + 0.834208,
      base + 0.839216, base + 0.844208, base + 0.849216};
  std::vector<uint64_t> released;
  for (double stamp : imu_nodes)
  {
    const auto result = scheduler.advance(stamp);
    require(result.valid, "monotonic watermark");
    for (const auto &event : result.ready)
    {
      released.push_back(event.id);
      if (event.id == 1) *first_event_release = event.release_state_stamp;
    }
  }
  require(scheduler.pendingCount() == 0, "all scheduled events released");
  return released;
}

FilterStateSnapshot snapshot(double stamp)
{
  FilterStateSnapshot state;
  state.stamp = stamp;
  state.p.x() = stamp;
  return state;
}

ImuSample imu(double stamp)
{
  ImuSample sample;
  sample.stamp = stamp;
  sample.acc = Eigen::Vector3d(0.2, -0.1, 9.7);
  sample.gyro = Eigen::Vector3d(0.01, -0.02, 0.03);
  return sample;
}
}  // namespace

int main()
{
  const double ndt_stamp = 1517157219.693280;
  const double deadline = ndt_stamp + 0.15;
  require(std::abs(deadline - 1517157219.843280) < 1e-6, "known index-5 deadline");
  double release_a = 0.0, release_b = 0.0;
  const auto a = releaseOrder({0, 1}, 1517157219.799216, &release_a);  // old 22-IMU endpoint
  const auto b = releaseOrder({1, 0}, 1517157219.804208, &release_b);  // old 23-IMU endpoint
  require(a == std::vector<uint64_t>({1, 2}) && a == b,
          "arrival order cannot change sensor-time release order");
  require(std::abs(release_a - 1517157219.844208) < 1e-6 && release_b == release_a,
          "both old endpoints wait for the same first IMU node after deadline");

  std::vector<double> stamps = {1517157219.689216, 1517157219.694208,
      1517157219.699216, 1517157219.704208, 1517157219.709216,
      1517157219.714208, 1517157219.719216, 1517157219.724208,
      1517157219.729216, 1517157219.734208, 1517157219.739216,
      1517157219.744208, 1517157219.749216, 1517157219.754208,
      1517157219.759216, 1517157219.764208, 1517157219.769216,
      1517157219.774208, 1517157219.779216, 1517157219.784208,
      1517157219.789216, 1517157219.794208, 1517157219.799216,
      1517157219.804208, 1517157219.809216, 1517157219.814208,
      1517157219.819216, 1517157219.824208, 1517157219.829216,
      1517157219.834208, 1517157219.839216, 1517157219.844208};
  StateHistory history;
  std::deque<ImuSample> imu_history;
  for (double stamp : stamps)
  {
    FilterStateSnapshot state = snapshot(stamp);
    history.insertMonotonic(state);
    imu_history.push_back(imu(stamp));
  }
  OosmReplayPlanner planner;
  const auto plan_a = planner.makePlan(history, imu_history, ndt_stamp, release_a, 0.02, 0.02);
  const auto plan_b = planner.makePlan(history, imu_history, ndt_stamp, release_b, 0.02, 0.02);
  require(plan_a.status == OosmPlanStatus::kReady && plan_b.status == OosmPlanStatus::kReady,
          "existing OOSM planner accepts fixed endpoint");
  require(plan_a.replay_samples.size() == plan_b.replay_samples.size(), "same replay count");
  for (std::size_t i = 0; i < plan_a.replay_samples.size(); ++i)
    require(plan_a.replay_samples[i].stamp == plan_b.replay_samples[i].stamp,
            "same replay sample sequence");

  OosmFixedLagScheduler late;
  require(late.configure(0.15, 4), "late config");
  require(late.enqueue(1, ndt_stamp, deadline, 600.0) == FixedLagEnqueueStatus::kLate,
          "at-cutoff callback is reported late");
  OosmFixedLagScheduler bounded;
  require(bounded.configure(0.15, 1), "queue bound config");
  require(bounded.enqueue(1, ndt_stamp, 1517157219.799216, 700.0) ==
              FixedLagEnqueueStatus::kQueued, "first queue entry");
  require(bounded.enqueue(2, ndt_stamp + 0.001, 1517157219.799216, 701.0) ==
              FixedLagEnqueueStatus::kQueueFull, "queue bound");
  std::cout << "fixed-lag scheduler contract PASS; deadline=" << deadline
            << " release=" << release_a << " replay_count="
            << plan_a.replay_samples.size() << std::endl;
  return 0;
}
