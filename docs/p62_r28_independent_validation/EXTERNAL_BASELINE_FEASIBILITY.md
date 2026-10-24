# Limited external-baseline feasibility

No external method was compiled or run. No clone, package install, system upgrade or dataset download was performed. Only four small upstream files were inspected; exact received bytes/URLs/hashes are recorded in sources/manifest.json. The upstream `main` URL is mutable; no remote commit is claimed because GitHub's API returned rate-limit HTTP403. The retained file SHA fixes what was actually reviewed.

Official project: [HViktorTsoi/FAST_LIO_LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION). The similarly named HKUST URL initially tested returned404; it was not used as evidence about this project's implementation.

|Question|Observed evidence|Assessment|
|---|---|---|
|ROS|Noetic and Python3 rospy available|Environment component available; framework build NOT_TESTED|
|Python/Open3D|README requests Python2.7/Open3D0.9. Python2 cannot import Noetic rospy; Python3 Open3D0.19 imports, but lacks `o3d.registration` used by the source; source also imports Python2 `thread`.|Not runnable unchanged; a scoped compatibility adapter may be possible, not proven uneconomic|
|ros_numpy|Module discoverable, but actual import fails with NumPy1.24.4: deprecated `np.float` removed|Availability is not an import PASS; no installed package patched|
|Ouster input|Upstream preprocess.cpp includes OUST64 processing|Input-type support exists; point field/time/deskew adapter not runtime-certified|
|Map|Localization script subscribes a PointCloud2 `/map`|Same PCD could be published; identical target and crop scheduling not yet demonstrated|
|Initialization|README exposes six-DoF `/initialpose` and advises initial stationarity|Potential anchor/C3 entry, not a matched initial-state test|
|Output|Localizer publishes `/map_to_odom`, consumes FAST-LIO `/Odometry` and `/cloud_registered`|Map-to-odometry architecture exists; complete output timestamp/frame contract not validated|

`logs/environment.json` reports package discovery; `logs/import_checks.json` reports actual imports. First pkg-config probe failed because PCL's .pc path was absent; the actual PCL1.10 GICP header and CMake config are present. This is not a finding that PCL is unavailable.

PCL GICP is therefore a plausible controlled registration baseline in the same EKF, not an implementation of the official FAST-LIO-LOCALIZATION framework. It was not integrated or run while G0 blocks valid reference initialization/comparison. No alternative method was silently substituted.

Decision: FAST-LIO-LOCALIZATION NOT_READY_AS_IS; adaptation cost NOT_ESTABLISHED by this finite check. Formal matched comparison **BLOCKED_BY_G0** for either method. No external results CSV is produced. The environment compatibility issues are secondary; the single upstream prerequisite to any fair accuracy comparison is the exact CSV frame binding.
