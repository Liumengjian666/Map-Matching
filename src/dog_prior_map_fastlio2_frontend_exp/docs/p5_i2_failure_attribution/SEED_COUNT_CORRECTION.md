# Seed-count correction record

The prompt's enumerated mandatory list contains seven seeds: exact GT, ±0.4 m X, ±0.4 m Y, and ±5° yaw. Its prose also said “nine”. The user explicitly resolved this conflict: **execute exactly the seven listed seeds; do not add seeds**.

Before that clarification arrived, a provisional symmetric 9-seed batch had started. It was interrupted after the terminal had reported complete frames P2F001–P2F005. Because CSV output may also contain a partially processed next frame, all interrupted files are preserved as:

- `oracle_ndt_runs_9seed_interrupted_superseded.csv`
- `oracle_objective_raw_9seed_interrupted_superseded.csv`

These files are superseded, must not be merged into the final analysis, and are retained only as execution provenance. The canonical oracle outputs will be regenerated from the beginning using only the seven seeds in `scripts/p5_i2/seeds.csv`.
