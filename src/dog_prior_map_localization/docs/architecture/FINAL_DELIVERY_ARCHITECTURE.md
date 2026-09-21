# Final delivery architecture

The frozen delivery path is a two-node ROS 1 chain:

```text
/livox/lidar + prior PCD
        |
        v
dog_prior_map_ndt_node_cpp
  preprocessing -> NDT map alignment -> step-limit audit
        |
        +--> /dog_livo/ndt_odom
                         |
/livox/imu ----------------+------------------+
                                             v
                              dog_prior_map_ekf_node_cpp
                              IMU propagation + NDT OOSM replay
                                             |
               +-----------------------------+------------------+
               v                                                v
      /dog_livo/odom_high_rate                         /dog_livo/odom_corrected
```

The canonical launch is `launch/dog_prior_map_localization_split.launch` and
the canonical parameters are in `config/dog_prior_map_localization_ndt.yaml`.
The EKF no longer owns a prior map, LiDAR subscriber, deskewer, or matcher;
the independent NDT node owns the map and publishes timestamped absolute
observations.  IMU propagation and OOSM replay remain in the EKF.

The delivery executable contains no active visual update, directional fusion,
information-matrix guard, Schur diagnostic, or local-degeneracy publisher.
Offline research probes remain opt-in and are not linked into the default
runtime build.  A future sensor extension must enter through a typed external
measurement module and must not restore a second map/matcher path inside EKF.

The final code is intentionally conservative: no NDT mathematics, EKF update,
OOSM policy, IMU propagation, topic contract, or coordinate convention was
changed by the cleanup stages.  The cleanup removed unreachable code and
resource ownership only.
