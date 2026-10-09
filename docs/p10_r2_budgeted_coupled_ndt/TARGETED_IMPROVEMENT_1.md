# Targeted development improvement1: mixed-scale first-stage coverage

Attempt0 is fully preserved with source/binary hashes, candidate/pose/time ledgers and blind-output freeze. Post-hoc GT had already been inspected. The change below is motivated by objective preview and cost evidence only, not by GT-selected parameters or frames.

Observed: all200 continuous C frames extended to16; mean/P95 total138.284/198.174ms. First8 global farthest-point proposals had zero admissible near-quality previews on all200 frames. The second stage supplied96 extra refinements and33 recommendations. Therefore zero first-stage admission does NOT establish unproductive expansion.

An initially proposed stop-on-zero quality gate was rejected BEFORE any real-data run. Its exact own-source diff is retained in `unrun_opportunity_gate.diff`; no experiment results belong to that proposal. The original adaptive expansion condition is kept.

Freeze the actual targeted change before new execution: choose4 admissible nonzero nodes closest to zero by normalized weak-coordinate distance, ties lowest original grid ID; then choose4 global farthest-point nodes from remaining original admissible grid, distances to zero and already selected points. This makes the first8 contain near-center resolution and outer coverage. Select the remaining8 by the same farthest-point rule. Visit each stage with the unchanged nearest-previous rule. No new weak ranges/grid nodes, motion weights, quality thresholds, correction steps, NDT calls or GT-dependent choices.

All A/B/C methods use this one common schedule. R2-A still forces16; integration remains adaptive8→16 with the unchanged fewer-than2 diverse-quality expansion rule. Re-run the entire two-frame,32-single,200-control/A/B/C matrix, not a favorable subset. This is development, and no GT improvement or runtime achievement is assumed in advance. Reduced query counts may also reduce recovery; report both outcomes.
