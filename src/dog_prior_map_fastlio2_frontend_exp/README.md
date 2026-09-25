# dog_prior_map_fastlio2_frontend_exp

This is an opt-in GPL-2.0-only research package. It is isolated from the
existing MIT NDT package and currently contains only the exact-snapshot IKFoM
generic pose-measurement API spike. It is not a runtime localization node.

Default catkin configure/build is protected by
`DOG_PRIOR_ENABLE_FASTLIO2_EXP=OFF`; no FAST-LIO checkout path is required or
inspected. To explicitly build the spike, enable the option and provide the
pinned external checkout documented in `FASTLIO2_DEPENDENCY_LOCK.md`.
