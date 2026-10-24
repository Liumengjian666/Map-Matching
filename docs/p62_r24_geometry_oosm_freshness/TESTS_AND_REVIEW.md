# Necessary R24 checks

Release catkin build completed, original stdout in logs/BUILD.log. Same whitelist and -j2 as R23, no compilation-parameter optimization search. Local installed PCL1.10 ndt.hpp constructor sets outlier_ratio_=.55; no outlier setter change.

New helper test command (exit0, observed stdout EXACT_LATEST_CORRECTED_HISTORY_CONTRACT_PASS):
`g++ -std=c++14 -Wall -Wextra -I src/dog_prior_map_localization/include docs/p62_r24_geometry_oosm_freshness/tools/test_revision.cpp -o /tmp/p62_r24_revision_test && /tmp/p62_r24_revision_test`
Covers exact match, +/-1ns mismatch, later history, invalid source/frame, nonfinite inputs, no history, duplicate corrected sample. Assertions enabled.

Launch XML and Python syntax pass. Actual roslaunch --dump-params in logs/RESOLVED_GF_PARAMS.yaml confirms unchanged1m/3m/.50 pre-gate, native .8/.08/.001/40, existing OOSM and only explicit G/F flags enabled. No ROS localization run in these static checks.

Local source review: default-off flags; strict same-stamp/latest-only replacement; unchanged at-or-before query; corrected provenance/frame/quaternion/pose/twist checks; no propagation/TF/OOSM mathematics edits; C-only enforcement and unchanged frozen policy imports; geometry load failure remains a hard rejection. Logs distinguish original pre-gate decision, actual native attempt, replacement and query consumption. No new optimizer or candidate generation method.

Existing corrected source proof: fusion/ndt_observation.cpp publishes odom_corrected only after OOSM_REPLAY_COMPLETE in the active OOSM branch, followed by ros_output.cpp LiDAR-pose/twist output. Both files remain byte-identical. Runtime records will be cross-checked post-seal; source proof does not imply actual timestamp equality or successful future query use.

Actual post-seal verification: all12 runs60 scans complete; all720 NDT queries at-or-before scan start and age<=20ms; 415 corrected messages match actual ROS corrected output and OOSM replay-complete lineage; 4 exact replacements, zero future-state consumption. All frozen algorithm/binary SHA and 228 raw native artifact SHA match; stable snapshot unchanged. G-only/GF and F/historical baseline pose outcomes checked independently. No additional NDT test calls. Raw ROS/build stdout whitespace retained rather than edited.
