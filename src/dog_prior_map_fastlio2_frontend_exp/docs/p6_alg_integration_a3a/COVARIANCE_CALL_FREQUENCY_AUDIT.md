# Pre-measurement timing and request frequency

The V2/V3 producer uses the sequence:

`prepareStateAt(scan_end)` → predicted state + IMU factor → sparse marginal → NDT/reliability → selected NIS → current LiDAR factor admission → optimization.

Thus current LiDAR information is absent from the pre-measurement P15. Previously accepted measurements, including any causally earlier visual event, are allowed in the graph.

Only `LIDAR_SCAN` and `LIDAR_SCAN_END` request covariance. This producer enables U_nonlocal in every P0/P1/P2/P3 policy, so each LiDAR terminal has a legitimate potential use of P. P2/P3 additionally use it for selected NIS. No covariance is requested at `LIDAR_SCAN_START`, `VISUAL_REFERENCE` or `VISUAL_CURRENT`.

For a non-LiDAR diagnostic row: `window_covariance_valid=NOT_REQUESTED_NON_LIDAR_EVENT`; position/rotation sigma=NaN; runtime marginal time=0. These values mean not requested, not failed or silently filled from an earlier event.

Synthetic real-PCL V2 fixture: 3 requests for 7 events. V3 fixture: 3 requests for 10 events. All four policies are tested. Runtime assertions check each event's request counter delta, zero dense-oracle requests, and unchanged LiDAR factor count during covariance extraction. CSV assertions check the non-LiDAR marker and NaNs.

Unavailable P disables U_nonlocal probing. Existing `evaluateSelectedLidarNis()` rejects an invalid P; P2/P3 cannot pass because P is absent. P0/P1 semantics, directional factors, A_exact relinearization, single Window ownership, and post-handoff IKFoM call count zero are retained.
