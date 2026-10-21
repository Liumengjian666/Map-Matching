# R11 source delta

All Git changes limited to docs/p62_r11_cause_aware_recovery/. R6–R10 and production untouched.

- tools/se3.hpp: full right-tangent SE3 Exp/Log, inverse translation Jacobian, principal quaternion rotation log, deterministic physically separate caps and explicit actual-pose checks. Numerical safety depends only on input coordinates/orthogonality and machine precision, never match score or GT.
- tools/shadow.cpp: genuine native PCL FULL100 +SPLIT100 common phase. Verify frozen five natural events before any recovery. Each arm creates its own vector with a value copy of the common nominal. BASELINE retains frozen G/SMALL; PROPOSED uses prediction residual seeds regardless of weak direction. No third generator/forced diagnostic, shared20-extra budget.
- Frozen native_capture.hpp captures returned optimizer pose before auxiliary fitness. New driver flushes raw metrics before validation and enables I/O exceptions. All native attempts journaled; no fabricated recovery output. Existing V2 dedup, eligibility, robust common-weight comparison and R10 final decision reused directly, not approximated.
- Final observations saved separately per arm; no acceptable candidate gives NA accepted matrix and prediction-only Shadow fallback. Invalid SE3 stops. Native nonfinite score/fitness is retained/rejected under unchanged R10 safety semantics and cannot enter G or selection.
- control.py: preservation gate, exact raw/initial identity checks, clean committed-source cold build with test/binary receipts, exclusive one-run token, and full post-run input/source/binary/stable protection checks. It never calls old experiment/audit programs or production binaries.
- Timing: FULL charged explicitly. Common source/map costs shared, arms report incremental phase time including logging. Native align excludes auxiliary fitness; global process high-water RSS is NOT an independent per-arm memory estimate.

Raw sources inherit the frozen loop2 raw/no-deskew, midpoint reference contract. No new IMU propagation, EKF trajectory or independent accuracy claim. The same-record FAST-LIVO2 map is not independent GT. StageA artifacts stay on internal disk; original R10 protection failure is not rewritten.
