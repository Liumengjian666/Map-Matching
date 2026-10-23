# P62-R17 — Event-triggered consensus admission

## Formal result

**`FALSE_REJECTION_RECOVERABLE_BY_RELAXATION`**. On the sole frozen 30-frame Newer College window, EVENT_CONSENSUS rescues the raw7570 reference-relative false rejection, but RELAXED_6DEG makes **the same 30 decisions and the exact same 30 output poses without the extra Split NDT**. No independent benefit of the Full/Split agreement test was observed. This mechanism should not be retained as the core paper innovation from this evidence.

The official `registered_poses.csv` is a **map-anchored benchmark reference**, not independent motion capture. The Base-axis interpretation remains paper-supported but lacks a direct CSV export binding. The 6° ceiling was chosen after observing R16 raw7570; this experiment is a development check on the same window, not an independent confirmation.

## Identity and execution

- Repository: `https://github.com/Liumengjian666/Map-Matching`; branch `research/p62-r17-consensus-admission`; independent worktree `/home/jian/livox_ws/p62_r17_consensus_ws`.
- START `537ea5b954277e5e4ba16633066b9256926e9bd4`; executed source commit **`fffd28d0404f8f4653c51350475f5873e5087325`**. Final archive commit is the local branch HEAD after this report is committed. No push performed.
- Release flags `-O3 -DNDEBUG -std=gnu++14`; PCL 1.10.0; executed binary SHA256 `2b045ac5111bb37995a5aabe21745215f7e095bcff897fe52681d75c1e2f4e9b`. Actual sources and commands: `tools/`, `RELEASE_BUILD_IDENTITY.json`. The C++ admission test printed `R17_POLICY_TEST_PASS`.
- Window: anchor ordinal7552; evaluations7553–7582; raw7570 is local18, timestamp 1583837348070227968 ns. All 31 selected PCD SHA256 identities match R16; each evaluated Source FNV and point count matched. Frozen 239,748-point Leica ROI SHA256 `ecc3b8d11e68ad28633e2aa4fb6effae1a5ccaeb6a8e1eb20d81e6fcc4ae7eca`. No map, raw PCD or stable-machine-dog workspace write operation.
- Source and binary were frozen before the single formal native run; its 90 decision rows and 91 align calls were hashed in `NATIVE_OUTPUT_FREEZE.json` before evaluation-frame reference values were read. `STATIC_RULE_REPLAY.csv` is a separate common-TP no-GT rule replay and is **not** a causal trajectory result.

## Causal admission comparison

All three strategies use the same anchor and independent policy-specific R14B constant-velocity history. Each runs 30 native PCL Full NDT aligns. After raw7570, Baseline and the two accepting arms have distinct predictions: raw7571 is the first divergent prediction; Baseline versus Relaxed differ in TP on 12 frames and in output pose on 13 frames. Relaxed and Consensus TP, Full result and output pose agree **exactly** on all 30 frames.

| Online policy | Full aligns | Extra Split aligns | Accepted / rejected | Longest consecutive fallback | Output translation RMSE / P95 / max (m) | Output rotation RMSE / P95 / max (°) |
|---|---:|---:|---:|---:|---:|---:|
| BASELINE_5DEG | 30 | 0 | 29 / 1 | 1 | 0.253623 / 0.319795 / 0.320876 | 1.998076 / 3.285604 / 4.874079 |
| RELAXED_6DEG | 30 | 0 | 30 / 0 | 0 | 0.245813 / 0.305165 / 0.320875 | 1.839348 / 2.924246 / 3.399598 |
| EVENT_CONSENSUS | 30 | 1 | 30 / 0 | 0 | 0.245813 / 0.305165 / 0.320875 | 1.839348 / 2.924246 / 3.399598 |

The online output is accepted Full when admitted and the causal prediction fallback when rejected. A rejected map observation is not counted as zero error. All statistics use 30 exact-time reference pairs; no trajectory-specific alignment was fitted.

### raw7570 and raw7571

At raw7570, Full was finite and converged, with Coverage valid, Overlap@0.5m **0.876984**, translation from TP **0.097251m**, rotation from TP **5.627697°**. Baseline therefore rejected it; 6° accepted it. Consensus ran one same-TP Training Split NDT; Split was finite and converged with Overlap **0.874339**. Full–Split separation was **0.042594m / 0.475783°**, so Consensus conditionally accepted the same Full pose. The baseline fallback error was **0.292366m / 4.874079°**; the accepted Full error was **0.238731m / 1.833496°** relative to the conditional official reference. This is one reference-relative false rejection, not proof that consistency certifies correctness.

At raw7571, Baseline TP had **0.503635m / 2.556580°** reference error, while Relaxed/Consensus TP had **0.254497m / 4.107537°**. Their histories really differ; the latter prediction has better translation but worse rotation at that instant. Baseline Full output was **0.320876m / 1.389061°**, and Relaxed/Consensus Full output **0.243982m / 1.844773°**. All three accepted their own Full. The variation demonstrates why static common-TP replay cannot substitute for policy-specific causal feedback.

## Runtime and budget

The one Release process performed **90 Full + 1 event Split**, within the extra-Split budget≤3. Event eligibility occurred once among30 frames (3.33%); only Consensus executed the extra call. Map load and Gaussian grid initialization took **34.416ms** once; process wall time was **1450.33ms**; peak RSS **70,816KiB**. Shared PCD load averaged **6.204ms/frame**, Source preprocessing **3.784ms/frame**. Per-arm wall mean / P95 / max: Baseline **12.970 / 17.960 / 41.011ms**; Relaxed **11.515 / 16.019 / 16.995ms**; Consensus **11.608 / 16.117 / 23.319ms**. Event Split align took **5.001ms**, with **1.398ms** auxiliary fitness and additional validation. These are sequential, warm-cache research measurements; the first Baseline Full call costs 37.673ms. They do not establish a 10Hz production system or isolate hardware scheduling effects.

## Literature, limits and next

`LITERATURE_COMPARISON.md` reviews Park & Chung (2024), Jeong & Kang (2026) and SuperLoc (2025). Reliable map-observation weighting, runtime disagreement signals and risk-aware localization are existing ideas. Our one-window consistency exception has no observed value over 6° relaxation. Full/Split are correlated, and neither is an independent correctness oracle. No ROS visualization was added because the condition for a valuable, method-specific effect was not met. Native optimizer parameters, Production EKF and R9–R16 archives were not changed.

**Unique NEXT for research-lead decision:** stop this agreement-based exception as a core innovation candidate; preregister an independent long-corridor or ambiguous-registration case with a traceable reference and test whether any event detector separates harmful from helpful >5° NDT corrections before designing another admission method. Do not tune this known raw7570 window again.
