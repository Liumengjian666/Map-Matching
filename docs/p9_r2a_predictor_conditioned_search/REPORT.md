# P9-R2A Predictor-conditioned weak-subspace search

FINAL_RESULT = `PREDICTOR_CONDITIONED_WEAK_SEARCH_SUPPORTED`

Frozen nine-frame/22-ID discovery gate, with 23 no-major safety controls. Oracle labels are evaluation only; GT is post-hoc only.

## Predictor/proposal contract

Seed122 predictor parity: max 0m / 1.70874e-06deg across32 frames.
`eta = eta_pred + Q Q^T (eta_j-eta_pred)`; first seed122, then deterministic farthest-point selection. The complete 16-call order is frozen. Three R2 Haar bases/frame are reused (PCG64 seed20261007+1000*frame_index+rep).
All methods use the unchanged frozen source/map/PCL1.10 NDT parameters. New calls2560; FULL6D new calls0.
The pre-run THEORY.md contains an incorrect raw-score sign sentence. Its original hashed bytes are preserved; OBJECTIVE_SIGN_CORRECTION.md is the authoritative correction: E=-S, so lowest energy means highest raw PCL score. Corrected offline selection is completed before GT scoring; no NDT output or discovery metric is changed.

## Recall (frame macro / ID micro)

|B|Method/rep|Macro|Micro|Random macro range|
|---:|---|---:|---:|---|
|4|COND_WEAK2/-1|0.318519|0.227273|—|
|8|COND_WEAK2/-1|0.725926|0.590909|—|
|12|COND_WEAK2/-1|0.803704|0.727273|—|
|16|COND_WEAK2/-1|0.803704|0.727273|—|
|4|COND_STRONG2/-1|0.277778|0.136364|—|
|8|COND_STRONG2/-1|0.362963|0.318182|—|
|12|COND_STRONG2/-1|0.403704|0.409091|—|
|16|COND_STRONG2/-1|0.403704|0.409091|—|
|4|COND_RANDOM2/0|0.022222|0.045455|—|
|8|COND_RANDOM2/0|0.081481|0.136364|—|
|12|COND_RANDOM2/0|0.233333|0.272727|—|
|16|COND_RANDOM2/0|0.362963|0.363636|—|
|4|COND_RANDOM2/1|0.018519|0.045455|—|
|8|COND_RANDOM2/1|0.185185|0.181818|—|
|12|COND_RANDOM2/1|0.248148|0.318182|—|
|16|COND_RANDOM2/1|0.248148|0.318182|—|
|4|COND_RANDOM2/2|0.000000|0.000000|—|
|8|COND_RANDOM2/2|0.022222|0.045455|—|
|12|COND_RANDOM2/2|0.151852|0.136364|—|
|16|COND_RANDOM2/2|0.192593|0.227273|—|
|4|COND_RANDOM2_MEDIAN/-2|0.018519|0.045455|[0.000000, 0.022222]|
|8|COND_RANDOM2_MEDIAN/-2|0.081481|0.136364|[0.022222, 0.185185]|
|12|COND_RANDOM2_MEDIAN/-2|0.233333|0.272727|[0.151852, 0.248148]|
|16|COND_RANDOM2_MEDIAN/-2|0.248148|0.318182|[0.192593, 0.362963]|

## AUC and gates

AUC uses the complete common1..16 nested prefix against log2(B). Historical H2 uses its frozen100 permutations; these historical orders differ from the new deterministic FPS order. Same-index controls are retained separately.
RANDOM frame-median macro AUC and median of three replicate macro AUCs are distinct aggregates; both and all replicate values are reported. The gate uses the former; WEAK exceeds both.
Important matched-index control: conditioned WEAK AUC0.399270 is lower than old zero-complement WEAK AUC0.479157 when both use the new WEAK seed-index order. Thus the prespecified discovery gate passes, but an across-cohort causal advantage from conditioning alone is not established. The two newly recovered2350 IDs remain genuine differences because old WEAK missed them over all263 seeds.

```json
{
  "auc": {
    "COND_WEAK2": 0.39927010911614236,
    "COND_STRONG2": 0.22568032777405073,
    "COND_RANDOM2_FRAME_MEDIAN": 0.05496486260041847,
    "random_replicate_macro_auc": [
      0.07198947229577518,
      0.09426914191306546,
      0.0384210345361438
    ],
    "COND_RANDOM2_REPLICATE_MEDIAN": 0.07198947229577518,
    "COND_RANDOM2_REPLICATE_MIN": 0.0384210345361438,
    "COND_RANDOM2_REPLICATE_MAX": 0.09426914191306546
  },
  "historical": {
    "FULL6D": {
      "auc_common_1_16": 0.26838242786794764,
      "macro_b16_median": 0.6592592592592593
    },
    "WEAK2": {
      "auc_common_1_16": 0.34211658544712964,
      "macro_b16_median": 0.5444444444444445
    },
    "auc_original_1_263": {
      "FULL6D": 0.5923765550795373,
      "WEAK2": 0.5163516831694155,
      "STRONG2": 0.43110966810022505,
      "RANDOM2": 0.47352220663068895,
      "random_replicate_macro_auc": [
        0.4394288074511306,
        0.4556437179169763,
        0.486422411725657,
        0.46661138247267037,
        0.4893292643882965
      ]
    },
    "matched_recall_cost70_FULL6D": {
      "reached_permutations": 100,
      "total_permutations": 100,
      "median": 32.0,
      "mean": 27.52,
      "P05": 16.0,
      "P95": 64.0
    },
    "full70_exact_prefix_secondary": {
      "median": 18.0,
      "min": 8,
      "max": 52,
      "p05": 10.0,
      "p95": 34.05,
      "warning": "32 is earliest tested ladder checkpoint median, not exact earliest-prefix cost"
    }
  },
  "same_index_reference": {
    "OLD_WEAK2": {
      "auc_common_1_16": 0.4791568627995109,
      "macro_by_budget": {
        "4": 0.48888888888888893,
        "8": 0.7111111111111111,
        "12": 0.751851851851852,
        "16": 0.7703703703703704
      }
    },
    "FULL6D": {
      "auc_common_1_16": 0.233025888151627,
      "macro_by_budget": {
        "4": 0.1111111111111111,
        "8": 0.44814814814814813,
        "12": 0.5037037037037037,
        "16": 0.5592592592592592
      }
    }
  },
  "decisions": [
    {
      "budget": 4,
      "gates": {
        "A": false,
        "B": true,
        "C": true,
        "D": false,
        "E": false,
        "F": false
      },
      "pass_gate": false,
      "primary_recovered": 0,
      "frames_at_least_old_weak": 4
    },
    {
      "budget": 8,
      "gates": {
        "A": true,
        "B": true,
        "C": true,
        "D": true,
        "E": false,
        "F": true
      },
      "pass_gate": false,
      "primary_recovered": 1,
      "frames_at_least_old_weak": 8
    },
    {
      "budget": 12,
      "gates": {
        "A": true,
        "B": true,
        "C": true,
        "D": true,
        "E": true,
        "F": true
      },
      "pass_gate": true,
      "primary_recovered": 2,
      "frames_at_least_old_weak": 9
    },
    {
      "budget": 16,
      "gates": {
        "A": true,
        "B": true,
        "C": true,
        "D": true,
        "E": true,
        "F": true
      },
      "pass_gate": true,
      "primary_recovered": 2,
      "frames_at_least_old_weak": 8
    }
  ]
}
```

## H2 final misses

Supported budget: 12; PRIMARY has three IDs but only two independent frames. This is a prespecified ID gate, not three independent samples.

|TX/ID|Old WEAK all263|COND WEAK B12|COND WEAK B16|
|---|---:|---:|---:|
|2226/P05|0|0|0|
|2350/P01|0|1|1|
|2350/P05|0|1|1|
|2722/P05|0|0|0|
|3341/P02|0|0|0|
|3341/P03|0|0|0|

## Predictor complement audit

|TX/ID|Successful FULL seeds|norm v_pred|Distance to median v|Distance to nearest v|
|---|---:|---:|---:|---:|
|2226/P05|13|0.0216035|0.254075|0.174016|
|2350/P01|10|0.15295|0.455956|0.131569|
|2350/P05|7|0.15295|0.655463|0.285856|
|2722/P05|11|0.0051448|1.92118|1.77892|
|3341/P02|10|0.23186|0.233654|0.169691|
|3341/P03|8|0.23186|0.943831|0.254542|

## Per-frame discovery

|TX|IDs|H1rho|WEAK AUC|STRONG AUC|RANDOM median [range] AUC|WEAK B16|OLD WEAK B16 median|
|---:|---:|---:|---:|---:|---|---:|---:|
|368|1|0.130827|0.459759|0.000000|0.228759 [0.188518, 0.459759]|1.000000|1.000000|
|616|6|0.882682|0.361135|0.092453|0.053014 [0.037373, 0.176328]|0.833333|0.666667|
|2226|2|0.907247|0.275940|0.275940|0.000000 [0.000000, 0.000000]|0.500000|0.000000|
|2350|3|0.885889|0.344972|0.000000|0.108653 [0.000000, 0.128880]|1.000000|0.333333|
|2722|2|0.190744|0.193319|0.000000|0.000000 [0.000000, 0.000000]|0.500000|0.000000|
|2846|1|0.545116|0.325960|0.875000|0.000000 [0.000000, 0.000000]|1.000000|1.000000|
|3341|5|0.543078|0.205466|0.235850|0.104257 [0.083456, 0.211594]|0.400000|0.600000|
|3796|1|0.999311|0.875000|0.000000|0.000000 [0.000000, 0.000000]|1.000000|0.000000|
|3962|1|0.567916|0.551880|0.551880|0.000000 [0.000000, 0.061526]|1.000000|1.000000|

## No-major controls and objective selection

New candidate clustering is descriptive and does not recluster the oracle. No-major frames can contain legitimate non-major alternatives; cluster proliferation is not a proven false-positive count.
Minimum energy E=-S is selected among nominal and all finite returns before GT is loaded (equivalently highest raw score). Nominal-reference errors reproduce the existing fixed-anchor GT contract.
The wrong-sign preliminary safety result is withdrawn. Corrected post-hoc scoring below is secondary; candidate discovery support alone does not establish that objective-only pose switching is safe.
Nominal-score carrier sensitivity is retained separately. Nominal U_obs quaternion reconstruction uses float, while archived terminal scores use a normalized double quaternion before Matrix4f. Tiny score gaps/exact tie identities are not robust; the substitution check is limited to byte-identical archived nominal pose texts and does not change material selected-pose geometry.

```json
{
  "exact_pose_comparison_frames": 23,
  "choices": 460,
  "max_nominal_score_gap": 0.00013738021380049759,
  "changed_exact_tie_choices": 112,
  "max_selected_translation_change_m": 0.0,
  "max_selected_rotation_change_deg": 5.8470537207253195e-06,
  "materially_changed_selected_pose_count": 0,
  "limitation": "float U_obs nominal quaternion carrier differs from archived double Pose3d; tiny score differences and exact ties are not robust"
}
```

|B|Method/rep|Healthy mean/max clusters|Objective switches|Left nominal|Iteration-limit rate|Major improved/same/worse (GT)|Healthy left and GT worse|
|---:|---|---:|---:|---:|---:|---|---:|
|12|COND_WEAK2/-1|4.391/9|21|2|0.029|3/0/6|2|
|16|COND_WEAK2/-1|4.609/10|22|2|0.024|3/0/6|2|
|12|COND_STRONG2/-1|5.043/9|22|1|0.022|6/0/3|0|
|16|COND_STRONG2/-1|5.348/10|22|1|0.016|6/0/3|0|
|12|COND_RANDOM2/0|10.348/12|16|3|0.178|3/3/3|3|
|16|COND_RANDOM2/0|13.478/16|16|3|0.185|3/1/5|3|
|12|COND_RANDOM2/1|10.261/12|17|2|0.149|2/1/6|2|
|16|COND_RANDOM2/1|13.174/16|18|4|0.158|2/1/6|4|
|12|COND_RANDOM2/2|11.000/12|15|2|0.214|3/4/2|2|
|16|COND_RANDOM2/2|14.304/16|15|2|0.215|3/4/2|2|

## Cost

Proposal preparation/selection: 1.993s total. Costs below are incremental discovery calls; total online calls would add the common nominal call. RANDOM2 has3 independent subspaces, so all-replicate cost is3B/frame.

|B|Method/rep|Calls/frame|Mean ms/probe|Mean ms/frame|Mean iterations/probe|
|---:|---|---:|---:|---:|---:|
|4|COND_WEAK2/-1|4|61.428|245.7|28.65|
|8|COND_WEAK2/-1|8|61.309|490.5|27.78|
|12|COND_WEAK2/-1|12|61.505|738.1|27.75|
|16|COND_WEAK2/-1|16|59.537|952.6|26.49|
|4|COND_STRONG2/-1|4|49.949|199.8|23.94|
|8|COND_STRONG2/-1|8|45.920|367.4|21.36|
|12|COND_STRONG2/-1|12|44.768|537.2|20.70|
|16|COND_STRONG2/-1|16|41.820|669.1|19.25|
|4|COND_RANDOM2/0|4|74.456|297.8|40.19|
|8|COND_RANDOM2/0|8|83.985|671.9|44.71|
|12|COND_RANDOM2/0|12|84.459|1013.5|44.86|
|16|COND_RANDOM2/0|16|85.485|1367.8|45.09|
|4|COND_RANDOM2/1|4|69.863|279.5|38.23|
|8|COND_RANDOM2/1|8|79.780|638.2|42.47|
|12|COND_RANDOM2/1|12|82.460|989.5|42.85|
|16|COND_RANDOM2/1|16|83.239|1331.8|43.01|
|4|COND_RANDOM2/2|4|71.784|287.1|39.95|
|8|COND_RANDOM2/2|8|82.356|658.8|45.14|
|12|COND_RANDOM2/2|12|85.926|1031.1|46.43|
|16|COND_RANDOM2/2|16|87.661|1402.6|46.92|

Nonconverged returns: 0; iteration-limit returns: 276. Multiply eligible major-frame terminals: 18.
NOT YET ONLINE-COMPETITIVE: WEAK B12 costs approximately738ms/frame for additional alignment calls alone, before the nominal call and runtime system overhead. The historical32/current12 call ratio2.67 is a checkpoint-schedule ratio, not an exact matched-recall or wall-time speedup. Historical FULL70 exact-prefix median is18 (secondary audit); the predeclared gate retains the original32 checkpoint reference.
Independent post-run audit reconstructs every conditioned proposal and all160 FPS orders; it verifies pool/selected/actual-start identity without calling NDT.
The frozen overlapping-ID admission limitation persists; ID recovery does not certify distinct dynamic minima. Wall time is descriptive across different historical/current runs.

BEST_BUDGET = `12`
NEXT = `PREDICTOR_CONDITIONED_EVIDENCE_COST_REDUCTION`
