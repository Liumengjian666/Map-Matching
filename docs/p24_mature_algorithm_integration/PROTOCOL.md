# P24 replay and evaluation protocol

## Frozen identity

- START_SHA: `1725d31c7ad81b170eb0a72ad1997c659c743dea`
- Branch: `research/p9-r4-heldout-visual-evidence`
- Map: 4058751-byte Corridor01 normalized PCD, SHA256 `103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f`
- Bag SHA256: `7f72f808a9f2282c65705e7e1e731061bb2cc2950329e46da768608e4a7d552e`
- visual.csv SHA256: `10869519890a5adcb6f3310bba05f379a959018cf123ea8a90c1cf4ea1314c79`
- Dataset config SHA256: `dc0850f91b43a4afed2e692644ac292db96a599e33fdafb414f3a986bc743220`
- Mature full overlay SHA256: `ecb0c2cd1c5ce9194ca3071833afe3d49b7c1708d2a7e4f14981a8dcf63d2d18`
- Frozen EKF executable SHA256: `fbc1750dc605a7fd3b9549b310394fbff3a5781cad2272ef55b9f49df8c3435a`

The P24 overlay fixes: `translation_scale_m=.8`, degeneracy ratio `.03`, weak translation step `.20m`, weak rotation step `2deg`, score improvement margin `.0002747604276`, motion weight `.05`, translation/rotation motion scales `2m/15deg`, maximum four quick candidates and maximum two extra complete alignments. P24's baseline overlay turns all three modules off. No parameter was chosen using GT.

## Run sequence

1. Build in isolated `.p24_build` / `.p24_devel` prefixes; preserve the existing catkin_make build tree. Run the P24 math test and existing targeted package contract tests.
2. Run a fixed 18.5-second smoke using the frozen Visual input, P18 SSD map config, P24 full overlay, `CONTROL`, camera off, scan-reference `start`, fixed-lag OOSM `.15s`, and rate `.25`. The qualifying smoke had 183 NDT, 182 OOSM APPLIED, 3698 high-rate states and 106 visual applications. Compare its physical prefix to P19.
3. Run once with the exact same inputs and flags for the full bag, without `--duration` or `--geometry`; output is the local `full_visual` run. The run receipt captured actual ROS parameters, executable hashes, resource samples and wall time.
4. Freeze output SHA256 values with `pre_gt_freeze.json`; the manifest says `GT_LOADED=false`. Only after this freeze, evaluate GT.

The full replay command was:

```bash
source .p24_devel/setup.bash && python3 src/dog_prior_map_localization/scripts/p17_visual/run.py --repo /home/jian/livox_ws/p10_corridor01_oosm_r1_ws --output /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/.p24_mature_algorithm_integration/full_visual --visual /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/.p17_visual_aided_ndt_r1/visual_input/visual.csv --dataset-config /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/src/dog_prior_map_localization/config/p18_corridor01_ssd_map.yaml --mature-config /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/src/dog_prior_map_localization/config/p24_mature_full.yaml --enable --port 11393
```

## Evaluation

Use the P10/P18 frozen `0–3s` Full-Pose SE(3) transform, the original GT interpolation and IMU-origin convention, and evaluate `3–35s`. This is prefix-aligned relative drift, not absolute map-frame ATE. P19 Visual is the frozen reference. Do not refit alignment, choose samples, alter parameters, or attribute the multi-module closed-loop result to one component.

Compare Initial Guess, Raw NDT, Final Used and EKF HighRate; also report 26–30s, 30–35s and fixed one-second bins. GT is not available to the runtime. P24's adapted full-run statistics and raw/lightweight records are in this directory. The recorded `outputs.bag`, map and input bag remain only in the local experiment cache; their SHA values are retained.

## Implementation classifications

- DCReg: adapted Schur/physical-subspace detector only. This does not reproduce a DCReg optimizer or targeted preconditioner.
- Multi-Hypothesis NDT: local, bounded preview and full refine using the current PCL NDT and existing objective. No global search; nominal is retained unless finite/converged plus objective and motion-merit gates pass. At most four previews, two extra full aligns, three total.
- SKF: status/direction gate only. The current relative displacement is already consumed by the NDT initial-guess path and lacks calibrated covariance, so no fake EKF observation is generated.

The complete P24 replay used NDT SHA `6e16596566da9e561e2b86c4d91dcb5aa2ce77cdb238c982ecb8fd3824e74f21`. A post-replay robustness fallback for general invertible indefinite Schur blocks was rebuilt as SHA `0df9548b09eb75537a0c104384c18d358faaf1cc836804393d20a7b4890d6331`; its rare fallback branch was covered by a unit test, not by the full replay. All 396 replay rows had valid Schur reports. The conservative near-singular rejection remains; no pseudoinverse is used to invent a weak axis.
