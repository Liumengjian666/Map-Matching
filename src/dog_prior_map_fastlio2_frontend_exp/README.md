# dog_prior_map_fastlio2_frontend_exp

This is an opt-in GPL-2.0-only research package. It is isolated from the
existing MIT NDT package. With the exact pinned FAST-LIO2 snapshot enabled, it
builds an IKFoM propagation/pose-update facade and a sequential ROS frontend
that exchanges scan transactions with the separate MIT prior-map NDT node.

Default catkin configure/build is protected by
`DOG_PRIOR_ENABLE_FASTLIO2_EXP=OFF`; no FAST-LIO checkout path is required or
inspected. To explicitly build the frontend, enable the option and provide the
pinned external checkout documented in `FASTLIO2_DEPENDENCY_LOCK.md`. See
[`docs/RUNTIME_1B_WIRED_FRONTEND.md`](docs/RUNTIME_1B_WIRED_FRONTEND.md) for
input fields, startup gates, and test coverage. The supplied runtime template
intentionally omits the initial map pose and expected map/config hashes, so it
cannot silently start with invented identity values.
