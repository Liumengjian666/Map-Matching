# R1 stop protocol and resume prerequisites

## Frozen decision

Do not launch P2B ROS nodes, run a new NDT alignment, evaluate a new trajectory,
or run Weak-only/Coupled while the historical source/runtime contract is open.
Do not repurpose P9 scan-end inputs or P2C full-SE3 output as P2B v1.

## Minimum artifacts required to resume

1. The P2B rotation-only adapter source snapshot including every transitive
   include, its original launch, CMake/package files, calibration, and a
   reproducible build receipt (or the exact historic binary plus dependency
   receipt).
2. The P2B R1 runtime config/launch override that resolves scan reference
   time, map path, topics, NDT parameters, EKF parameters, OOSM settings, and
   the startup pose/time.
3. Either the original P2B v1 derived bag and its per-message receipts, or
   authorization to reconstruct a bounded input from the verified official
   raw bag once items 1–2 are closed. The P2B scan-start and IMU frame/time
   semantics must be checked before replay.
4. The frozen Control trajectory and per-frame NDT receipt (or a replay
   parity gate against a trusted historical artifact) so the new 35-second
   Control can be checked before algorithm comparison.

## Resume gate

After provenance closure, reproduce the causal P2B Control over the fixed
historical evaluation interval: first five seconds for startup/initialization,
then the complete `+0..+35 s` evaluation window. First freeze the full output;
only then load GT and apply the single Control-prefix evaluation transform.
Proceed to independent causal Nominal, Weak-only, and Coupled feedback only if
Control's first ten evaluation seconds are sub-meter and do not show a
persistent ~180-degree frame conflict, with the same scan-time and map/GT
evaluation contract.

No new algorithm or parameter change is authorized by this receipt.
