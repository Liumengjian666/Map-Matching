# P18 resumed run: pre-GT contract

P18 original start: `3b3d53da8fe755c5f3c24dd8c99008579625b7a0`.
Resume start: `e43b07006c52a08eba0aeb7c0e651e1813d5c844`.
The previous missing-device receipt is preserved; this is a later observed
external-state change, not a revision of the old observation.

At 10:04:25 HIKVISION re-enumerated on usb 2-2; mount is /dev/sda2 fuseblk rw.
One bounded copy to a new SSD recovery directory succeeded. Size 4058751;
SHA256 103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f.
Only map.pcd_fallback_path changes in the independent P18 YAML. All other
YAML values must compare equal. Actual ROS map path is checked before play.

Reuse the frozen V2 bag, original visual.csv, compiled P17 NDT/EKF nodes,
scan-start reference, CONTROL mode, camera disabled, fixed lag .150 seconds,
playback .25x. No geometry hook, Coupled feedback, new align or tuning.
Short Control and Visual each play 18.5 seconds from the bag start; evaluation
origin remains 1517157224.188979. This covers evaluation 6-12 seconds.
Compare the first **180** Control NDT transactions and corresponding OOSM
records with historical Control V2, plus high-rate states up to the short
input endpoint. These 180 transactions have complete scan and lag coverage.
Remaining truncated-boundary frames are logged separately, never hidden.

Existing frozen pose tolerance: translation 1e-9 m, rotation 1e-8 deg;
fitness 1e-12*max(1,abs(control_score)); sensor times 1e-12 seconds; cloud
count/hash and discrete status exact. Do not adjust tolerances. Report the
earliest mismatch and stop Visual if Control parity fails.

After disabled-prefix parity and enabled smoke pass, run one full 396-frame
Visual replay. Freeze outputs and configuration before opening GT. Reuse
P10 Control's fixed 0-3-second full-pose transform, not a new fit. Evaluate
3-35 seconds and the predetermined 6-12,26-30,30-35-second windows separately.
Initial/raw/used LiDAR poses must convert to IMU origins using frozen T_il;
high-rate EKF lineage is already IMU. Metrics are prefix-aligned relative
drift, not absolute map-frame ATE. Offline CSV injection is not realtime
camera frontend deployment. Late no-visual windows can only show downstream
causal history effects, not direct visual observations.
