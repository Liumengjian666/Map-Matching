# Figure captions and placement

## Graphical abstract — separate upload

Diagnostic geometry, unchanged quality admission, and bounded retries form distinct stages of a causal inertial localization path. The depicted 60-scan development case uses eight additional native alignments and publishes 57 map observations. This is a mechanism illustration, not a claim of universal recovery.

Files: `figures/gagraphic.png` (660 × 295 pixels, 300 dpi metadata), `.pdf` and editable `.svg`. The PNG is below the IEEE recommended45KB. The graphic is original vector artwork generated for this package. It summarizes frozen measurements and contains no invented trajectory.

## Figure 1 — measurement path

Causal observation path. G changes only the authority of the pre-alignment geometry predicate. All candidate outputs face unchanged post-registration quality, publication, and historical filter-update checks. A rejected scan leaves the real inertial estimator propagating. The dashed route illustrates the original early-return failure.

Files: `figures/measurement_path.pdf/.svg/.png`. Double-column vector figure; no experimental data are recomputed.

## Figure 2 — complete causal error curves

All60 reference-relative fused-state errors in W1/C3. Original and G-only trajectories coincide. The vertical marker identifies the first C+G map update at raw2603. The initial large rotation errors remain visible. The horizontal axis uses scan-start elapsed time; errors are evaluated at each output's actual fused-state time.

File: `figures/ablation_errors.pdf`, byte-identical to the frozen R25 figure. The underlying180-row figure dataset remains in R25; no new reference evaluation occurred.

## Figure 3 — memory scope

Historical matched-segment NDT-process high-water memory for global and restricted targets. The82.2% reduction does not include all localization/display processes and does not imply a universal target-independent budget. Timing and maximum-error trade-offs are reported in the text.

Files: `figures/local_map_memory.pdf/.svg/.png`. Replotted from frozen775308/138200KiB; internal stage labels replaced with global/local target. The conversion toMiB is division by1024; axis starts at zero.

## Supplementary Figure S1 — quality admission

Frozen W1/C3 quality decisions. A missing selected value denotes no admitted map observation, not zero loss. The constant0.30threshold was not tuned using the G-only ablation. This plot is a consistency/admission diagnostic rather than independent correctness evidence.

File: `figures/coverage_admission.pdf`, byte-identical to R25. Full NDT used these source points; do not label it independent holdout validation.
