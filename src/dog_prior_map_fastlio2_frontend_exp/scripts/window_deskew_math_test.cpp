#include "dog_prior_map_fastlio2_frontend_exp/window_scan_processor.hpp"
#include <iostream>
#include <stdexcept>
using namespace dog_prior_map_fastlio2_frontend_exp;
using namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag;
static void check(bool ok, const std::string& text) { if (!ok) throw std::runtime_error(text); }
int main() {
  double maximum = 0, jacobian_error = 0;
  for (int scenario = 0; scenario < 6; ++scenario) {
    WindowState anchor;
    anchor.stamp_ns = 1000000000;
    anchor.position = Eigen::Vector3d(1,2,3);
    anchor.rotation = Eigen::AngleAxisd(.3, Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
    if (scenario == 1 || scenario >= 3) anchor.velocity = Eigen::Vector3d(.5,-.2,.1);
    const Eigen::Vector3d omega = scenario >= 2 ? Eigen::Vector3d(.2,-.1,.4) : Eigen::Vector3d::Zero();
    if (scenario == 5) { anchor.gyro_bias = Eigen::Vector3d(.01,-.03,.02); anchor.accel_bias = Eigen::Vector3d(.2,-.1,.05); }
    Pose3d extrinsic;
    extrinsic.orientation = Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitX());
    if (scenario >= 4) extrinsic.position = Eigen::Vector3d(.3,-.1,.2);
    ImuNoiseParameters noise;
    std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> imu;
    std::vector<TimedLidarPoint, Eigen::aligned_allocator<TimedLidarPoint>> cloud;
    // World acceleration is zero at each left knot. Inputs average to the
    // left-knot specific force, matching the frozen midpoint/left-R model.
    Eigen::Vector3d last_input = -anchor.rotation.transpose() * noise.gravity;
    for (int i = 0; i <= 20; ++i) {
      const double t = .005 * i;
      const Eigen::Matrix3d rotation = anchor.rotation *
          (omega.norm() ? Eigen::AngleAxisd(omega.norm()*t, omega.normalized()).toRotationMatrix() : Eigen::Matrix3d::Identity());
      ImuSample sample;
      sample.stamp_ns = anchor.stamp_ns + i * 5000000;
      sample.angular_velocity = omega + anchor.gyro_bias;
      if (i > 0) {
        const double previous_t = t - .005;
        const Eigen::Matrix3d previous_r = anchor.rotation *
            (omega.norm() ? Eigen::AngleAxisd(omega.norm()*previous_t, omega.normalized()).toRotationMatrix() : Eigen::Matrix3d::Identity());
        last_input = -2.0 * previous_r.transpose() * noise.gravity - last_input;
      }
      sample.acceleration = last_input + anchor.accel_bias;
      imu.push_back(sample);
      const Eigen::Vector3d world(3+.02*i,-1+.03*i,4+.01*i);
      TimedLidarPoint point;
      point.stamp_ns = sample.stamp_ns;
      point.intensity = i;
      point.position = extrinsic.orientation.conjugate() *
          (rotation.transpose() * (world - anchor.position - anchor.velocity*t) - extrinsic.position);
      cloud.push_back(point);
    }
    // Half-knot timestamps exercise the unchanged p interpolation / SO3
    // slerp geometry rather than testing only exact IMU sample stamps.
    for(int i=0;i<20;++i) {
      const double t=.005*i+.0025;
      const Eigen::Matrix3d r=anchor.rotation * (omega.norm() ?
          Eigen::AngleAxisd(omega.norm()*t,omega.normalized()).toRotationMatrix() : Eigen::Matrix3d::Identity());
      const std::size_t k=cloud.size();
      const Eigen::Vector3d world(3+.02*k,-1+.03*k,4+.01*k);
      TimedLidarPoint p; p.stamp_ns=anchor.stamp_ns+i*5000000+2500000;
      p.position=extrinsic.orientation.conjugate() *
          (r.transpose()*(world-anchor.position-anchor.velocity*t)-extrinsic.position);
      cloud.push_back(p);
    }
    WindowDeskewResult result; std::string reason;
    check(deskewScanWithWindowState(anchor, anchor.stamp_ns, imu.back().stamp_ns,
        imu, noise, extrinsic, cloud, &result, &reason), reason);
    const Eigen::Matrix3d end_r = result.predicted_end_state.rotation;
    // Independent copy of the old ScanEndProcessor point transform, with
    // exactly the same IMU trajectory. Its production code now calls the
    // extracted shared geometry; no legacy estimator enters Window runtime.
    for(std::size_t k=0;k<cloud.size();++k) {
      const auto& point=cloud[k];
      auto upper=std::lower_bound(result.imu_trajectory.begin(),result.imu_trajectory.end(),point.stamp_ns,
          [](const ImuPoseSample& p,std::uint64_t t){return p.stamp_ns<t;});
      Eigen::Vector3d position=upper->position;
      Eigen::Quaterniond rotation(upper->rotation);
      if(upper->stamp_ns!=point.stamp_ns) {
        const auto& lower=*(upper-1);
        const double a=static_cast<double>(point.stamp_ns-lower.stamp_ns)/(upper->stamp_ns-lower.stamp_ns);
        position=(1-a)*lower.position+a*upper->position;
        rotation=Eigen::Quaterniond(lower.rotation).slerp(a,rotation);
      }
      rotation.normalize();
      const Eigen::Quaterniond lidar_r=(rotation*extrinsic.orientation).normalized();
      const Eigen::Vector3d lidar_p=position+rotation*extrinsic.position;
      const Eigen::Quaterniond end_q=(Eigen::Quaterniond(end_r)*extrinsic.orientation).normalized();
      const Eigen::Vector3d end_p=result.predicted_end_state.position+end_r*extrinsic.position;
      const Eigen::Vector3d old=end_q.conjugate()*(lidar_r*point.position+lidar_p-end_p);
      check((old-result.cloud_end_frame[k].position).norm()<1e-12,"old geometry same-trajectory parity");
    }
    check((result.predicted_end_state.position - anchor.position - anchor.velocity*.1).norm() < 1e-12, "model position");
    for (std::size_t i=0; i<cloud.size(); ++i) {
      const Eigen::Vector3d world(3+.02*i,-1+.03*i,4+.01*i);
      const Eigen::Vector3d expected = extrinsic.orientation.conjugate() *
          (end_r.transpose() * (world - anchor.position - anchor.velocity*.1) - extrinsic.position);
      maximum = std::max(maximum, (result.cloud_end_frame[i].position - expected).norm());
    }
    ImuPreintegratedMeasurement reference;
    check(preintegrateImu(imu, anchor.stamp_ns, imu.back().stamp_ns, anchor.gyro_bias,
        anchor.accel_bias, noise, &reference, &reason), reason);
    const WindowState propagated = propagateWindowState(anchor, reference, noise.gravity);
    check(localDifference(propagated, result.predicted_end_state).norm() == 0, "shared integration parity");
    WindowState from = anchor, to = propagated;
    from.gyro_bias += Eigen::Vector3d(.02,-.01,.03);
    from.accel_bias += Eigen::Vector3d(.1,.05,-.04);
    Vector15d perturb = Vector15d::LinSpaced(15, -.2, .3);
    check(applyLocalIncrement(&to, perturb), "perturb");
    Matrix15d a,b,af,bf; Vector15d r,rf;
    check(linearizeImuFactor(from,to,reference,noise,&a,&b,&r,&reason), reason);
    check(linearizeImuFactorFiniteDifferenceReference(from,to,reference,noise,&af,&bf,&rf,&reason),reason);
    jacobian_error = std::max(jacobian_error, std::max((a-af).cwiseAbs().maxCoeff(), (b-bf).cwiseAbs().maxCoeff()));
    check((r-rf).norm() == 0, "residual changed");
    for(double angle:{.02,1.4,3.1413}) {
      WindowState near=propagated;
      Vector15d rotation_delta=Vector15d::Zero();
      rotation_delta.head<3>()=angle*Eigen::Vector3d(2,-1,3).normalized();
      check(applyLocalIncrement(&near,rotation_delta),"near-pi perturbation");
      check(linearizeImuFactor(anchor,near,reference,noise,&a,&b,&r,&reason),reason);
      check(linearizeImuFactorFiniteDifferenceReference(anchor,near,reference,noise,&af,&bf,&rf,&reason),reason);
      jacobian_error=std::max(jacobian_error,std::max((a-af).cwiseAbs().maxCoeff(),(b-bf).cwiseAbs().maxCoeff()));
    }
    WindowState branch_cut=propagated;
    Vector15d at_pi=Vector15d::Zero(); at_pi(0)=std::acos(-1.0);
    check(applyLocalIncrement(&branch_cut,at_pi),"branch cut perturbation");
    check(!linearizeImuFactor(anchor,branch_cut,reference,noise,&a,&b,&r,&reason) &&
        reason=="imu_rotation_log_branch_cut","nonunique Log derivative not rejected");
    cloud.front().stamp_ns = 0;
    check(!deskewScanWithWindowState(anchor, anchor.stamp_ns, imu.back().stamp_ns,
        imu, noise, extrinsic, cloud, &result, &reason) && reason == "RAW_POINT_TIME_UNAVAILABLE", "missing point time must reject");
  }
  check(maximum < 1e-10, "deskew error");
  check(jacobian_error < 2e-7, "analytic IMU Jacobian mismatch");
  std::cout << "PASS six motions max_point_error=" << maximum << " analytic_FD_max=" << jacobian_error << '\n';
}
