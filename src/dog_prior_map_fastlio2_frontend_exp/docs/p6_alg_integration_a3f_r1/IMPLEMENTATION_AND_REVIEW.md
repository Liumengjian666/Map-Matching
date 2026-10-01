# Implementation and review

New covariance production code is isolated in `src/window_square_root_covariance.cpp`.
The old sparse solver is unchanged and exposed through a private same-snapshot
helper. V3 explicitly selects QR; its diagnostic environment flag enables only
legacy shadow and CSV output, not an alternative estimator.

New CTest covers: known inverse, nontrivial QR permutation, correlated whitening,
extreme-scale known covariance, rank-deficient fail-closed, rank-5 LiDAR,
initial/IMU-only/joint directional-visual graphs, repeated Schur, nonzero rotation
prior chart after optimization and before reanchoring, exact diagnostic parity,
NIS/probe parity, one callback including shadow, and read-only state/prior/lifecycle.

The existing A3A source test initially assumed the covariance function remained
in fixed_lag_window.cpp. Its path assumption was updated without deleting a test
or loosening numerical tolerance: legacy sparse/no-dense assertions remain, and
new direct-row/no-normal-solve/no-inverse assertions were added. The frozen FULL
byte-parity audit ignores only the new additive helper include, retaining the
original body comparison.

Fresh-context read-only review approved core math and final guarded replay.
Findings resolved before real replay: validate invalid snapshot rank before
allocation; explicitly test moved rotation chart; include Pmap-only divergence;
buffer independently flushed logs; require complete streams; match exact
request/comparison/terminal identities and reject duplicates. All received
Required findings are addressed with tests. Cross-model review is reserved for
the user's manual final handoff, per their instruction.

Replay guard permits one process launch, pins raw hashes and frozen A3E-R2
comparison file SHA, requires clean committed source and correct ancestry,
checks physical schedule, rejects pre-output state drift, and records first
output/decision/old-event divergence separately. It cannot retry or tune anything.

READY_FOR_FORMAL_EXPERIMENT=NO.
