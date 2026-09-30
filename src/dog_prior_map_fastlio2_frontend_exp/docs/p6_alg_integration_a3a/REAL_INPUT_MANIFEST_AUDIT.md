# Actual input identity audit

Raw input is newly exported, persisted on HIKVISION, and independently revalidated with `--validate-only`. Both validations reported PASS, 2777 scans / 79,932,911 points / 3,197,316,440 bytes. `RAW_TIMED_INPUT_MANIFEST.txt` is an exact Git copy of the persistent input's manifest; the large binary is not uploaded.

The exporter checked original bag SHA, calibration SHA, clean pinned driver revision, actual helper source hash, and source identity functions in the actual loaded decoder library. Its manifest records source paths, binary/library hashes, calibration/extrinsic identities, conversion formula, counts, bounds, output hashes and all four false GT/estimator/deskew flags.

The Python validator verifies every output file SHA, all 79,932,911 point records, catalog offsets/counts/minimum timestamps/intervals, raw provenance, physical-time schedule, pinned data identity, hash syntax and exact conservative time contract. The V3 startup gate checks actual raw/catalog/filter SHA, catalog byte coverage and manifest counts/bounds, sensor identity, hash syntax and time semantics **before NDT**; the C++ point loader independently rejects zero/outside/nonfinite/truncated records and false scan-start minimum. A manifest is provenance evidence plus integrity metadata, not a signed execution attestation.

Raw scan schedule is generated from original sensor sequence, not old pose output. All 2777 raw terminals disagree with old asset terminals. This is recorded, not adjusted. The V3 CLI uses raw catalog in place of legacy scans.csv. It preserves transaction IDs and physical timestamps when skipping pre-handoff acquisition.

Visual CSV source-chain independence fails the conservative audit. The 482-row sidecar exactly matches each existing ref/cur pair and contains only UNKNOWN; no missing row receives a default RAW promotion. `VISUAL_INPUT_MANIFEST.txt` marks FORMAL_ELIGIBILITY=NO and MAP_POSE_USED=true (generation control flow). Old data remain compatibility-only. The current preparation script is not falsely presented as a recorded generation hash for the historical CSV.

No GT file was read, no estimator pose was applied to raw points, no Window state was used in export, and no real-data NDT, trajectory generation or ATE/RPE evaluation was executed. Raw export is decoding—not a localization experiment. NDT used by CTests sees synthetic fixtures only.

READY_FOR_SHORT_REAL_LINK_TEST=YES, limited to qualified raw LiDAR with existing independently sourced IMU/calibration and GT-free initialization. This is input/build eligibility, not a passed real-link replay. Existing visual products are not qualified for that link. Hardware-clock origin/sync accuracy remains explicitly unproven.

READY_FOR_FORMAL_EXPERIMENT=NO.
