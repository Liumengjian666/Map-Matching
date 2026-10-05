# H2 matched-budget basin discovery

FINAL_RESULT = WEAK_GEOMETRIC_PRIOR_SUPPORTED_BUT_DISCOVERY_GAIN_NOT_ESTABLISHED
H1 geometric concentration remains supported on its fixed cohort. H2 is evaluated independently below. No posterior, GT, trajectory accuracy, continuation, EKF or revised oracle definition is used.

## AUC and frame-level paired tests

```json
{
  "auc": {
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
  "exact_tests": {
    "FULL6D": 0.8046875,
    "STRONG2": 0.14453125,
    "RANDOM2": 0.29296875
  },
  "gates": {
    "A": false,
    "B": true,
    "C": true,
    "D": false,
    "E": false,
    "F": true,
    "G": false,
    "H": false
  }
}
```

## Per-frame results

|TX|major|H1 rho|FULL AUC|WEAK|STRONG|RANDOM median [min,max]|W-F|W-R|final FULL/WEAK|
|---|---:|---:|---:|---:|---:|---|---:|---:|---|
|368|1|0.130827|0.751210|0.813408|0.440223|0.689013 [0.689013,0.813408]|0.062197|0.124395|1.000/1.000|
|616|6|0.882682|0.543886|0.600019|0.543079|0.491248 [0.440223,0.511980]|0.056134|0.108772|1.000/1.000|
|2226|2|0.907247|0.502421|0.220112|0.344506|0.220112 [0.046426,0.344506]|-0.282309|0.000000|1.000/0.500|
|2350|3|0.885889|0.564618|0.312601|0.000000|0.229671 [0.063811,0.397145]|-0.252017|0.082930|1.000/0.333|
|2722|2|0.190744|0.580167|0.046426|0.000000|0.502421 [0.378026,0.626815]|-0.533741|-0.455994|1.000/0.500|
|2846|1|0.545116|0.813408|0.937803|1.000000|0.813408 [0.689013,0.937803]|0.124395|0.124395|1.000/1.000|
|3341|5|0.543078|0.570838|0.463166|0.614376|0.626815 [0.564618,0.651694]|-0.107672|-0.163650|1.000/0.600|
|3796|1|0.999311|0.440223|0.440223|0.000000|0.000000 [0.000000,0.000000]|0.000000|0.440223|1.000/1.000|
|3962|1|0.567916|0.564618|0.813408|0.937803|0.689013 [0.440223,0.813408]|0.248790|0.124395|1.000/1.000|

## Recall curves (median [P05,P95], 100 matched permutations)

|Method/rep|B|macro|micro median|
|---|---:|---|---:|
|FULL6D/-1|1|0.038889 [0.000000,0.209259]|0.045455|
|FULL6D/-1|2|0.148148 [0.000000,0.300926]|0.136364|
|FULL6D/-1|4|0.287037 [0.129074,0.467963]|0.227273|
|FULL6D/-1|8|0.470370 [0.288704,0.623704]|0.409091|
|FULL6D/-1|16|0.659259 [0.507407,0.848148]|0.590909|
|FULL6D/-1|32|0.864815 [0.685185,0.960185]|0.818182|
|FULL6D/-1|64|0.979630 [0.851852,1.000000]|0.954545|
|FULL6D/-1|128|1.000000 [0.999074,1.000000]|1.000000|
|FULL6D/-1|192|1.000000 [1.000000,1.000000]|1.000000|
|FULL6D/-1|263|1.000000 [1.000000,1.000000]|1.000000|
|WEAK2/-1|1|0.151852 [0.018519,0.277778]|0.136364|
|WEAK2/-1|2|0.244444 [0.040741,0.411111]|0.227273|
|WEAK2/-1|4|0.392593 [0.192593,0.522222]|0.318182|
|WEAK2/-1|8|0.470370 [0.359259,0.603704]|0.409091|
|WEAK2/-1|16|0.544444 [0.466111,0.677778]|0.500000|
|WEAK2/-1|32|0.640741 [0.528704,0.697222]|0.590909|
|WEAK2/-1|64|0.696296 [0.585185,0.770370]|0.636364|
|WEAK2/-1|128|0.696296 [0.696296,0.770370]|0.636364|
|WEAK2/-1|192|0.770370 [0.696296,0.770370]|0.727273|
|WEAK2/-1|263|0.770370 [0.770370,0.770370]|0.727273|
|STRONG2/-1|1|0.133333 [0.022222,0.244444]|0.090909|
|STRONG2/-1|2|0.240741 [0.133333,0.337037]|0.181818|
|STRONG2/-1|4|0.305556 [0.207222,0.411296]|0.272727|
|STRONG2/-1|8|0.383333 [0.300000,0.507593]|0.363636|
|STRONG2/-1|16|0.440741 [0.377778,0.552778]|0.454545|
|STRONG2/-1|32|0.551852 [0.422037,0.592593]|0.545455|
|STRONG2/-1|64|0.574074 [0.462778,0.592593]|0.590909|
|STRONG2/-1|128|0.592593 [0.570370,0.592593]|0.636364|
|STRONG2/-1|192|0.592593 [0.592593,0.592593]|0.636364|
|STRONG2/-1|263|0.592593 [0.592593,0.592593]|0.636364|
|RANDOM2/0|1|0.031481 [0.000000,0.186296]|0.045455|
|RANDOM2/0|2|0.131481 [0.000000,0.262963]|0.090909|
|RANDOM2/0|4|0.225926 [0.080556,0.429815]|0.181818|
|RANDOM2/0|8|0.357407 [0.174074,0.525926]|0.318182|
|RANDOM2/0|16|0.511111 [0.344444,0.607407]|0.454545|
|RANDOM2/0|32|0.607407 [0.510370,0.686111]|0.636364|
|RANDOM2/0|64|0.662963 [0.624259,0.740741]|0.681818|
|RANDOM2/0|128|0.740741 [0.662222,0.740741]|0.772727|
|RANDOM2/0|192|0.740741 [0.685185,0.740741]|0.772727|
|RANDOM2/0|263|0.740741 [0.740741,0.740741]|0.772727|
|RANDOM2/1|1|0.022222 [0.000000,0.244444]|0.045455|
|RANDOM2/1|2|0.111111 [0.000000,0.296481]|0.090909|
|RANDOM2/1|4|0.233333 [0.055000,0.400741]|0.181818|
|RANDOM2/1|8|0.364815 [0.172963,0.514815]|0.318182|
|RANDOM2/1|16|0.514815 [0.347963,0.629630]|0.454545|
|RANDOM2/1|32|0.625926 [0.518519,0.722222]|0.636364|
|RANDOM2/1|64|0.722222 [0.643704,0.759259]|0.727273|
|RANDOM2/1|128|0.740741 [0.722222,0.759259]|0.772727|
|RANDOM2/1|192|0.759259 [0.740741,0.759259]|0.818182|
|RANDOM2/1|263|0.759259 [0.759259,0.759259]|0.818182|
|RANDOM2/2|1|0.059259 [0.000000,0.222222]|0.045455|
|RANDOM2/2|2|0.170370 [0.022037,0.282222]|0.136364|
|RANDOM2/2|4|0.261111 [0.155370,0.396481]|0.227273|
|RANDOM2/2|8|0.405556 [0.284259,0.527037]|0.363636|
|RANDOM2/2|16|0.527778 [0.436667,0.648704]|0.545455|
|RANDOM2/2|32|0.661111 [0.551852,0.740741]|0.681818|
|RANDOM2/2|64|0.740741 [0.629630,0.740741]|0.772727|
|RANDOM2/2|128|0.740741 [0.740741,0.740741]|0.772727|
|RANDOM2/2|192|0.740741 [0.740741,0.740741]|0.772727|
|RANDOM2/2|263|0.740741 [0.740741,0.740741]|0.772727|
|RANDOM2/3|1|0.037037 [0.000000,0.262963]|0.045455|
|RANDOM2/3|2|0.133333 [0.017593,0.304444]|0.136364|
|RANDOM2/3|4|0.237037 [0.044259,0.430741]|0.227273|
|RANDOM2/3|8|0.394444 [0.229630,0.525926]|0.363636|
|RANDOM2/3|16|0.529630 [0.418333,0.626667]|0.500000|
|RANDOM2/3|32|0.640741 [0.551667,0.696296]|0.636364|
|RANDOM2/3|64|0.685185 [0.640741,0.740741]|0.727273|
|RANDOM2/3|128|0.740741 [0.685185,0.740741]|0.772727|
|RANDOM2/3|192|0.740741 [0.739630,0.740741]|0.772727|
|RANDOM2/3|263|0.740741 [0.740741,0.740741]|0.772727|
|RANDOM2/4|1|0.111111 [0.000000,0.244444]|0.090909|
|RANDOM2/4|2|0.188889 [0.018519,0.358333]|0.136364|
|RANDOM2/4|4|0.337037 [0.118148,0.474630]|0.272727|
|RANDOM2/4|8|0.444444 [0.318519,0.570370]|0.409091|
|RANDOM2/4|16|0.559259 [0.477593,0.630185]|0.545455|
|RANDOM2/4|32|0.629630 [0.570370,0.722222]|0.659091|
|RANDOM2/4|64|0.703704 [0.648148,0.759259]|0.772727|
|RANDOM2/4|128|0.740741 [0.703704,0.796296]|0.818182|
|RANDOM2/4|192|0.796296 [0.740741,0.796296]|0.863636|
|RANDOM2/4|263|0.796296 [0.796296,0.796296]|0.863636|

## Matched recall costs and accounting

```json
{
  "costs": {
    "FULL6D": {
      "0.25": {
        "reached_permutations": 100,
        "total_permutations": 100,
        "median": 4.0,
        "mean": 5.4,
        "P05": 2.0,
        "P95": 8.0
      },
      "0.5": {
        "reached_permutations": 100,
        "total_permutations": 100,
        "median": 16.0,
        "mean": 13.52,
        "P05": 8.0,
        "P95": 16.0
      },
      "0.7": {
        "reached_permutations": 100,
        "total_permutations": 100,
        "median": 32.0,
        "mean": 27.52,
        "P05": 16.0,
        "P95": 64.0
      }
    },
    "WEAK2": {
      "0.25": {
        "reached_permutations": 100,
        "total_permutations": 100,
        "median": 4.0,
        "mean": 3.89,
        "P05": 1.0,
        "P95": 8.0
      },
      "0.5": {
        "reached_permutations": 100,
        "total_permutations": 100,
        "median": 16.0,
        "mean": 17.48,
        "P05": 4.0,
        "P95": 32.0
      },
      "0.7": {
        "reached_permutations": 100,
        "total_permutations": 100,
        "median": 192.0,
        "mean": 182.61,
        "P05": 62.400000000000006,
        "P95": 263.0
      }
    },
    "STRONG2": {
      "0.25": {
        "reached_permutations": 100,
        "total_permutations": 100,
        "median": 4.0,
        "mean": 3.7,
        "P05": 2.0,
        "P95": 8.0
      },
      "0.5": {
        "reached_permutations": 100,
        "total_permutations": 100,
        "median": 32.0,
        "mean": 43.84,
        "P05": 8.0,
        "P95": 128.0
      },
      "0.7": {
        "reached_permutations": 0,
        "total_permutations": 100,
        "median": "NOT_REACHED",
        "mean": "NOT_REACHED",
        "P05": "NOT_REACHED",
        "P95": "NOT_REACHED"
      }
    },
    "RANDOM2_0": {
      "0.25": {
        "reached_permutations": 100,
        "total_permutations": 100,
        "median": 8.0,
        "mean": 7.24,
        "P05": 2.0,
        "P95": 16.0
      },
      "0.5": {
        "reached_permutations": 100,
        "total_permutations": 100,
        "median": 16.0,
        "mean": 23.92,
        "P05": 8.0,
        "P95": 33.59999999999991
      },
      "0.7": {
        "reached_permutations": 100,
        "total_permutations": 100,
        "median": 128.0,
        "mean": 142.32,
        "P05": 62.400000000000006,
        "P95": 263.0
      }
    },
    "RANDOM2_1": {
      "0.25": {
        "reached_permutations": 100,
        "total_permutations": 100,
        "median": 8.0,
        "mean": 7.25,
        "P05": 2.0,
        "P95": 16.0
      },
      "0.5": {
        "reached_permutations": 100,
        "total_permutations": 100,
        "median": 16.0,
        "mean": 22.96,
        "P05": 8.0,
        "P95": 32.0
      },
      "0.7": {
        "reached_permutations": 100,
        "total_permutations": 100,
        "median": 64.0,
        "mean": 84.48,
        "P05": 32.0,
        "P95": 128.0
      }
    },
    "RANDOM2_2": {
      "0.25": {
        "reached_permutations": 100,
        "total_permutations": 100,
        "median": 4.0,
        "mean": 5.89,
        "P05": 2.0,
        "P95": 8.0
      },
      "0.5": {
        "reached_permutations": 100,
        "total_permutations": 100,
        "median": 16.0,
        "mean": 19.88,
        "P05": 8.0,
        "P95": 32.0
      },
      "0.7": {
        "reached_permutations": 100,
        "total_permutations": 100,
        "median": 64.0,
        "mean": 64.64,
        "P05": 32.0,
        "P95": 128.0
      }
    },
    "RANDOM2_3": {
      "0.25": {
        "reached_permutations": 100,
        "total_permutations": 100,
        "median": 8.0,
        "mean": 6.58,
        "P05": 1.0,
        "P95": 16.0
      },
      "0.5": {
        "reached_permutations": 100,
        "total_permutations": 100,
        "median": 16.0,
        "mean": 19.52,
        "P05": 8.0,
        "P95": 32.0
      },
      "0.7": {
        "reached_permutations": 100,
        "total_permutations": 100,
        "median": 128.0,
        "mean": 112.55,
        "P05": 64.0,
        "P95": 192.0
      }
    },
    "RANDOM2_4": {
      "0.25": {
        "reached_permutations": 100,
        "total_permutations": 100,
        "median": 4.0,
        "mean": 4.65,
        "P05": 2.0,
        "P95": 8.0
      },
      "0.5": {
        "reached_permutations": 100,
        "total_permutations": 100,
        "median": 16.0,
        "mean": 15.04,
        "P05": 8.0,
        "P95": 32.0
      },
      "0.7": {
        "reached_permutations": 100,
        "total_permutations": 100,
        "median": 64.0,
        "mean": 86.88,
        "P05": 32.0,
        "P95": 128.0
      }
    },
    "RANDOM2_POOLED_DESCRIPTIVE": {
      "0.25": {
        "reached_permutation_replicates": 500,
        "total_permutation_replicates": 500,
        "median": 8.0,
        "mean": 6.322,
        "P05": 2.0,
        "P95": 16.0
      },
      "0.5": {
        "reached_permutation_replicates": 500,
        "total_permutation_replicates": 500,
        "median": 16.0,
        "mean": 20.264,
        "P05": 8.0,
        "P95": 32.0
      },
      "0.7": {
        "reached_permutation_replicates": 500,
        "total_permutation_replicates": 500,
        "median": 64.0,
        "mean": 98.174,
        "P05": 32.0,
        "P95": 192.0
      }
    }
  },
  "efficiency": {
    "0.5": 1.0,
    "0.7": 0.16666666666666666
  },
  "run_counts": {
    "FULL_reused": 2367,
    "WEAK_new": 2367,
    "STRONG_new": 2367,
    "RANDOM_new": 11835,
    "exceptions": 0,
    "new_nonconverged": 0,
    "new_iteration_limit": 1077
  },
  "runtime": [
    {
      "method": "FULL6D",
      "random_rep": -1,
      "ndt_calls": 2367,
      "iterations": 82094,
      "summed_align_seconds": 124.78398315999999,
      "mean_align_ms": 52.718201588508656,
      "median_align_ms": 42.175899,
      "provenance": "HISTORICAL",
      "nonconverged": 0,
      "iteration_limit": 147
    },
    {
      "method": "WEAK2",
      "random_rep": -1,
      "ndt_calls": 2367,
      "iterations": 58880,
      "summed_align_seconds": 108.56362664699999,
      "mean_align_ms": 45.86549499239543,
      "median_align_ms": 34.490646,
      "provenance": "CURRENT",
      "nonconverged": 0,
      "iteration_limit": 55
    },
    {
      "method": "STRONG2",
      "random_rep": -1,
      "ndt_calls": 2367,
      "iterations": 35911,
      "summed_align_seconds": 60.615006763,
      "mean_align_ms": 25.608367876214615,
      "median_align_ms": 19.875254,
      "provenance": "CURRENT",
      "nonconverged": 0,
      "iteration_limit": 8
    },
    {
      "method": "RANDOM2",
      "random_rep": 0,
      "ndt_calls": 2367,
      "iterations": 79637,
      "summed_align_seconds": 146.58315000800002,
      "mean_align_ms": 61.92782002872836,
      "median_align_ms": 45.20034,
      "provenance": "CURRENT",
      "nonconverged": 0,
      "iteration_limit": 188
    },
    {
      "method": "RANDOM2",
      "random_rep": 1,
      "ndt_calls": 2367,
      "iterations": 90346,
      "summed_align_seconds": 157.189807157,
      "mean_align_ms": 66.40887501351922,
      "median_align_ms": 50.97347,
      "provenance": "CURRENT",
      "nonconverged": 0,
      "iteration_limit": 252
    },
    {
      "method": "RANDOM2",
      "random_rep": 2,
      "ndt_calls": 2367,
      "iterations": 80539,
      "summed_align_seconds": 138.47594506,
      "mean_align_ms": 58.50272288128433,
      "median_align_ms": 42.414626,
      "provenance": "CURRENT",
      "nonconverged": 0,
      "iteration_limit": 227
    },
    {
      "method": "RANDOM2",
      "random_rep": 3,
      "ndt_calls": 2367,
      "iterations": 80271,
      "summed_align_seconds": 144.943835634,
      "mean_align_ms": 61.235249528517116,
      "median_align_ms": 42.476437,
      "provenance": "CURRENT",
      "nonconverged": 0,
      "iteration_limit": 211
    },
    {
      "method": "RANDOM2",
      "random_rep": 4,
      "ndt_calls": 2367,
      "iterations": 77258,
      "summed_align_seconds": 135.400641422,
      "mean_align_ms": 57.203481800591476,
      "median_align_ms": 43.279689,
      "provenance": "CURRENT",
      "nonconverged": 0,
      "iteration_limit": 136
    }
  ],
  "secondary": {
    "spearman": 0.27122540144832413,
    "exact_two_sided_p": 0.48273809523809524,
    "permutations": 362880
  }
}
```

## Interpretation limits

Nine fixed, historically selected single-sequence frames are the inference units; 22 basins and 100 permutations are not iid trials. No universal efficiency law is inferred. FULL6D timing is historical/descriptive; primary fair costs are call counts and iterations. Source/grid preparation, proposal computation, U_obs, nominal NDT, exact-score evaluation are outside align-runtime measurements. The 1 nominal call is common and excluded from incremental budgets. All projected calls, including near duplicates and converged iteration-limit returns, are counted. Admission-ball overlap uses the predeclared single-assignment rule, with pair separations archived.

Observed admission ambiguity: 833 of 18,936 terminals (88 in the reused FULL6D pool) satisfy the admission conditions for more than one frozen canonical basin. The predeclared minimum normalized squared-distance rule assigns each terminal to exactly one basin, with lexical cluster-ID tie breaking; no major-basin definition or clustering was changed after observing results. Recall therefore measures recovery of frozen archived basin IDs, not certification of independently separated stationary modes.

Four canonical-center pairs themselves are within 0.2 m AND 2 deg: TX616/P02-P03 (0.051382 m / 0.939975 deg), TX616/P13-P17 (0.019507 m / 1.307886 deg), TX2722/P04-P05 (0.013488 m / 1.697209 deg), and TX3341/P01-P02 (0.045578 m / 1.928764 deg). Four is the count of close canonical-center pairs, not the total count of overlapping admission regions: two admission regions may overlap even when their centers are farther apart, up to 0.4 m and 4 deg. The complete pair-separation table and per-terminal eligible IDs are archived in `oracle_pair_separations.csv` and `terminal_admission.csv`.

High H1 projection does not ensure capture under projected initial proposals. TX2226 has weak rho 0.907247 but WEAK2 AUC 0.220112 versus FULL6D 0.502421 and final recall 1/2; TX2350 has rho 0.885889 but WEAK2 AUC 0.312601 versus FULL6D 0.564618 and final recall 1/3. TX3796 has rho 0.999311 yet WEAK2 only ties FULL6D AUC at 0.440223. Conversely TX368 has rho 0.130827 and WEAK2 improves AUC from 0.751210 to 0.813408. These are failures of the simple projected-proposal discovery mechanism to turn geometric concentration into a reliable compute advantage, not failures of H1 statistical parity.

NEXT = EVALUATE_SECOND_GENERATION_SUPPORT_AWARE_BRANCH_SEARCH
