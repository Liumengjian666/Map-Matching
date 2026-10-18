# Bounded implementation review

Skills used: git-workflow-and-versioning (isolated worktree), incremental-implementation (time/input/build/smoke gates), debugging-and-error-recovery (two build-environment faults), doubt-driven-development and code-review-and-quality (causal gate/export review).

Claim under review: the mapper receives only the immutable first45% sensor data, and observation/export instrumentation leaves author estimation, Deskew and insertion mathematics unchanged.

Fresh-context read-only review received only code and the contract. It found one required issue: direct final tree flatten could race background ikd-Tree rebuild and old-node deletion. Accepted as actionable. Add export-only `frozen_snapshot`, called after foreground processing stops, with the same lock order as rebuild (`rebuild_ptr_mutex_lock` then `working_flag_mutex`). Second bounded pass confirmed the lock construction and identified the remaining unlocked `Root_Node` precheck; removed it. No further fresh-context review cycle was run on the unchanged artifact. The exporter handles empty snapshots as a failure, not a valid map.

Environment build fixes: author target lacked a generated-message dependency (`fast_lio/Pose6D.h`); add explicit catkin generated targets. Installed MVS libusb shadowed system libusb needed by PCL; explicitly link and preload the installed system `/lib/x86_64-linux-gnu/libusb-1.0.so.0`. No libraries were installed, replaced or reconfigured globally. These are build/runtime loader fixes before any smoke, not tuning or a failed-data rerun.

Before smoke: eight contract tests pass (fixed windows, exact complete prefix, crossing tail exclusion, inclusive endpoint, pre-window rejection, test-window rejection, Xsens/zero-offset config, SO(3)/inverse check). The full filtered bag is independently re-traversed; LiDAR serialized hashes must match the inherited R2 ledger. Final code diff is restricted to research tools, observation-only author logging/ACK/final snapshot export, and build dependencies. Protected worktrees and production NDT/EKF/Coupled/Recovery are untouched.

Cross-model CLI review was offered; no CLI execution is authorized or implied by historical approvals.
