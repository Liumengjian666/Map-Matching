# P9-R1B fixed-u attraction/support/refine report

FINAL_RESULT = STRONG_ATTRACTION_MECHANISM_MIXED

NEXT = SUPPORT_AWARE_LOCAL_BRANCH_CONTINUATION_R1C

No GT, weak grid, transported W, posterior weights, or production EKF. All chart/bounds/solver parameters are frozen.

Main endpoint groups are proximity-based, not certified stationary attractors. Bisection intervals are sampled group sections, not 4D capture volumes.

## Summary

```json
{
  "counts": {
    "STRONG_INTRINSIC_MULTIATTRACTOR_certified": 0,
    "SUPPORT_SWITCH_ASSOCIATED_YES": 0,
    "SUPPORT_SWITCH_ASSOCIATED_PARTIAL": 3,
    "GLOBAL_MIN_ERASES_LOCAL_REPRESENTATIVE": 1,
    "LOWER_SEPARATED_REPRESENTATIVE_ALL_PART_A": 4,
    "GLOBAL_MIN_ERASES_LOCAL_BRANCH_certified": "INDETERMINATE",
    "FULL_REFINE_ESCAPE": 2,
    "UNRESOLVED_strict_dynamic_attractor_certification": 7,
    "matched_dynamic_multigroup_frozen_both_single": 5
  },
  "execution": {
    "targets": 7,
    "initial_samples": 350,
    "main_samples": 77,
    "local_samples": 168,
    "matched_controls": 105,
    "bisection_samples": 32,
    "support_path_rows": 707,
    "accepted_event_rows": 3353,
    "accepted_moves": 2971,
    "formal_full_ndt_calls": 38,
    "full_ndt_status": {
      "SUCCESS": 38
    },
    "inner_energy_evaluations": 50756,
    "dynamic_endpoint_trace_diagnostics": 4499,
    "newton_runtime_ms": {
      "mean": 99.98879202166066,
      "median": 100.879007,
      "p95": 147.6938768,
      "max": 168.239771
    },
    "pattern_runtime_ms": {
      "mean": 21.731001533333334,
      "median": 4.427417,
      "p95": 53.905652,
      "max": 82.254199
    },
    "same_settings_development_repeats": {
      "initial_samples": 350,
      "bisection_samples": 32,
      "support_path_rows": 707,
      "extra_full_ndt_calls": 0
    }
  }
}
```

## 616/P02

u_b=[0.061283432956718537, -0.029500840336129486]; v_b=[-0.008035332254346935, -0.03227716246119209, -0.030699602597467457, 0.002310598777840663]

|beta|strong group|canonical radius capture|termination|seed→endpoint strong norm|
|---:|---|---:|---|---:|
|0.0|OTHER_01|0|BRANCH_GRADIENT_SMALL|0.00141292|
|0.1|OTHER_01|0|TRUST_RADIUS_EXHAUSTED|0.000165202|
|0.2|OTHER_01|0|TRUST_RADIUS_EXHAUSTED|0.000214258|
|0.3|CANONICAL|1|ENERGY_CHANGE_SMALL|0.00434891|
|0.4|CANONICAL|1|ENERGY_CHANGE_SMALL|0.000158607|
|0.5|CANONICAL|1|MAX_ITERATIONS|0.000184637|
|0.6|CANONICAL|1|ENERGY_CHANGE_SMALL|0.000272649|
|0.7|CANONICAL|1|ENERGY_CHANGE_SMALL|0.000544723|
|0.8|CANONICAL|1|TRUST_RADIUS_EXHAUSTED|0.000618048|
|0.9|CANONICAL|1|TRUST_RADIUS_EXHAUSTED|0.000857526|
|1.0|CANONICAL|1|STEP_SMALL|1.25172e-06|

Local cross capture (.02/.05/.10): {'0.02': {'captured': 8, 'total': 8}, '0.05': {'captured': 7, 'total': 8}, '0.1': {'captured': 5, 'total': 8}}

Bisection: [{'transaction_id': '616', 'cluster_id': 'P02', 'beta_low': '0.23125', 'beta_high': '0.2375', 'width': '0.006249999999999978', 'low_branch': 'OTHER_01', 'high_branch': 'CANONICAL', 'depth': '4', 'boundary_kind': 'BUDGETED_ENDPOINT_GROUP_SECTION'}]

```json
{
  "support": {
    "largest_support_switch": 0.01675977653631285,
    "largest_support_switch_beta": 0.3,
    "largest_dynamic_cusp_beta": 0.52,
    "dynamic_abs_second_difference_max": 0.008796378737042865,
    "frozen_t0_abs_second_difference_max": 5.546442243709038e-06,
    "frozen_canonical_abs_second_difference_max": 5.594387401330891e-06,
    "top_support_beta": [
      0.3,
      1.0,
      0.11,
      0.45,
      0.46
    ],
    "top_cusp_beta": [
      0.52,
      0.6,
      0.61,
      0.53,
      0.51
    ],
    "per_boundary_association": [
      {
        "beta_low": 0.23125,
        "beta_high": 0.2375,
        "support_hit": false,
        "cusp_hit": false
      }
    ],
    "associated": "NO"
  },
  "matched_pattern": {
    "DYNAMIC": {
      "endpoint_groups": 2,
      "canonical_capture": 4,
      "statuses": {
        "EVALUATION_BUDGET": 3,
        "POLL_STEP_SMALL": 2
      },
      "dynamic_energies": [
        -3.085142636898073,
        -3.077422668642501,
        -3.0688447836201713,
        -3.0925242116758476,
        -3.093566321821138
      ],
      "objective_energies": [
        -3.085142636898073,
        -3.077422668642501,
        -3.0688447836201713,
        -3.0925242116758476,
        -3.093566321821138
      ]
    },
    "FROZEN_T0": {
      "endpoint_groups": 1,
      "canonical_capture": 0,
      "statuses": {
        "POLL_STEP_SMALL": 1,
        "EVALUATION_BUDGET": 4
      },
      "dynamic_energies": [
        -3.0488974235236985,
        -3.0480361942925653,
        -3.0460876649456075,
        -3.0488831920015045,
        -3.052725929975931
      ],
      "objective_energies": [
        -3.0419594501086564,
        -3.0419326214532116,
        -3.0418503260585634,
        -3.041949009200628,
        -3.041723060614421
      ]
    },
    "FROZEN_CANONICAL": {
      "endpoint_groups": 1,
      "canonical_capture": 5,
      "statuses": {
        "EVALUATION_BUDGET": 4,
        "POLL_STEP_SMALL": 1
      },
      "dynamic_energies": [
        -3.074824285760765,
        -3.069660114492546,
        -3.0802939868516095,
        -3.077139297834212,
        -3.072560115607611
      ],
      "objective_energies": [
        -3.0723331560645377,
        -3.0725461051488137,
        -3.072451073328599,
        -3.0725329079522017,
        -3.072560115607611
      ]
    }
  },
  "global_profile_main": {
    "canonical_group_energy": -3.0725601717097923,
    "closed_canonical_energy": -3.0725601476991624,
    "lowest_discovered_other_main_group_energy": -3.054928064058792,
    "lowest_other_group_energy_without_pair_separation": -3.0610032855894227,
    "other_branch": "OTHER_01",
    "other_to_canonical_representative_distance_m_deg": [
      0.004833483892322784,
      2.316183013555042
    ],
    "global_min_would_erase_canonical_representative": false,
    "negative_result_means": "no witnessed lower separated main representative; not proof of global uniqueness",
    "strict_local_minimum_certified": false
  },
  "global_profile_all_part_a": {
    "lowest_separated_other_energy": -3.0610032855894227,
    "other_group": "NEW_02",
    "distance_from_canonical_representative_m_deg": [
      0.012743979812855423,
      5.050545855268367
    ],
    "lower_separated_representative_witness": false
  }
}
```

|branch|pre distance m/deg|post distance m/deg|iterations/status|escape|
|---|---|---|---|---|
|CANONICAL|0.000001/0.000049|0.000004/0.000187|1/SUCCESS|0|
|OTHER_01|0.004834/2.316195|0.020898/2.656320|3/SUCCESS|0|
|NEW_01|0.002106/0.317320|0.002980/0.159537|1/SUCCESS|0|
|NEW_02|0.012744/5.050585|0.032296/2.615960|3/SUCCESS|0|
|NEW_03|0.011185/5.693347|0.072959/7.429298|8/SUCCESS|0|
|NEW_04|0.019652/5.068517|0.078079/3.033207|7/SUCCESS|0|

## 616/P03

u_b=[0.12062756467407006, -0.010410313613903256]; v_b=[0.004814025239058845, -0.014925688505535602, -0.03275812522409694, 0.0014079691571288432]

|beta|strong group|canonical radius capture|termination|seed→endpoint strong norm|
|---:|---|---:|---|---:|
|0.0|OTHER_01|0|BRANCH_GRADIENT_SMALL|0.00187156|
|0.1|CANONICAL|1|BRANCH_GRADIENT_SMALL|0.00119184|
|0.2|CANONICAL|1|BRANCH_GRADIENT_SMALL|0.00134894|
|0.3|CANONICAL|1|TRUST_RADIUS_EXHAUSTED|0.0027717|
|0.4|CANONICAL|1|TRUST_RADIUS_EXHAUSTED|8.20379e-06|
|0.5|CANONICAL|1|MAX_ITERATIONS|0.00277905|
|0.6|CANONICAL|1|TRUST_RADIUS_EXHAUSTED|1.55194e-05|
|0.7|CANONICAL|1|TRUST_RADIUS_EXHAUSTED|0.00200337|
|0.8|CANONICAL|1|MAX_ITERATIONS|0.00283291|
|0.9|CANONICAL|1|TRUST_RADIUS_EXHAUSTED|0.00166248|
|1.0|CANONICAL|1|MAX_ITERATIONS|0.000534836|

Local cross capture (.02/.05/.10): {'0.02': {'captured': 8, 'total': 8}, '0.05': {'captured': 4, 'total': 8}, '0.1': {'captured': 5, 'total': 8}}

Bisection: [{'transaction_id': '616', 'cluster_id': 'P03', 'beta_low': '0.0', 'beta_high': '0.00625', 'width': '0.00625', 'low_branch': 'OTHER_01', 'high_branch': 'CANONICAL', 'depth': '4', 'boundary_kind': 'BUDGETED_ENDPOINT_GROUP_SECTION'}]

```json
{
  "support": {
    "largest_support_switch": 0.01675977653631285,
    "largest_support_switch_beta": 0.96,
    "largest_dynamic_cusp_beta": 0.47,
    "dynamic_abs_second_difference_max": 0.005815155414013784,
    "frozen_t0_abs_second_difference_max": 5.408524203165399e-06,
    "frozen_canonical_abs_second_difference_max": 5.420096017338949e-06,
    "top_support_beta": [
      0.96,
      0.03,
      0.59,
      0.65,
      0.92
    ],
    "top_cusp_beta": [
      0.47,
      0.42,
      0.54,
      0.02,
      0.41
    ],
    "per_boundary_association": [
      {
        "beta_low": 0.0,
        "beta_high": 0.00625,
        "support_hit": false,
        "cusp_hit": true
      }
    ],
    "associated": "PARTIAL"
  },
  "matched_pattern": {
    "DYNAMIC": {
      "endpoint_groups": 2,
      "canonical_capture": 5,
      "statuses": {
        "EVALUATION_BUDGET": 5
      },
      "dynamic_energies": [
        -3.084761398220761,
        -3.0922343413367606,
        -3.0840991477353374,
        -3.088237964363474,
        -3.0859188072549935
      ],
      "objective_energies": [
        -3.084761398220761,
        -3.0922343413367606,
        -3.0840991477353374,
        -3.088237964363474,
        -3.0859188072549935
      ]
    },
    "FROZEN_T0": {
      "endpoint_groups": 2,
      "canonical_capture": 1,
      "statuses": {
        "POLL_STEP_SMALL": 1,
        "EVALUATION_BUDGET": 4
      },
      "dynamic_energies": [
        -3.0575867935800143,
        -3.059533349292246,
        -3.058540591028148,
        -3.060253359446844,
        -3.0577922956799015
      ],
      "objective_energies": [
        -3.03685300535298,
        -3.0368367247552848,
        -3.0367973131115544,
        -3.0367795576761543,
        -3.036698174552947
      ]
    },
    "FROZEN_CANONICAL": {
      "endpoint_groups": 1,
      "canonical_capture": 5,
      "statuses": {
        "EVALUATION_BUDGET": 4,
        "POLL_STEP_SMALL": 1
      },
      "dynamic_energies": [
        -3.0680819676143005,
        -3.0594720712983534,
        -3.070166119026523,
        -3.063436081849023,
        -3.0654247312615053
      ],
      "objective_energies": [
        -3.066884730363991,
        -3.067008586544134,
        -3.0669773249269277,
        -3.0670093248122203,
        -3.0670155481937966
      ]
    }
  },
  "global_profile_main": {
    "canonical_group_energy": -3.0721797697573234,
    "closed_canonical_energy": -3.0670127193452497,
    "lowest_discovered_other_main_group_energy": null,
    "lowest_other_group_energy_without_pair_separation": -3.07325802336872,
    "other_branch": null,
    "other_to_canonical_representative_distance_m_deg": null,
    "global_min_would_erase_canonical_representative": false,
    "negative_result_means": "no witnessed lower separated main representative; not proof of global uniqueness",
    "strict_local_minimum_certified": false
  },
  "global_profile_all_part_a": {
    "lowest_separated_other_energy": -3.07325802336872,
    "other_group": "NEW_02",
    "distance_from_canonical_representative_m_deg": [
      0.007994702479196019,
      2.466765148281112
    ],
    "lower_separated_representative_witness": true
  }
}
```

|branch|pre distance m/deg|post distance m/deg|iterations/status|escape|
|---|---|---|---|---|
|CANONICAL|0.002506/0.705569|0.002861/0.772656|1/SUCCESS|0|
|OTHER_01|0.006995/2.003482|0.026974/1.886892|3/SUCCESS|0|
|NEW_01|0.001305/0.515496|0.003262/0.362285|1/SUCCESS|0|
|NEW_02|0.007115/2.657559|0.005039/1.004791|3/SUCCESS|0|
|NEW_03|0.005681/2.340423|0.040880/0.734322|6/SUCCESS|0|
|NEW_04|0.011356/5.428476|0.083321/6.459258|8/SUCCESS|0|
|NEW_05|0.008046/4.311421|0.050286/2.574000|7/SUCCESS|0|

## 616/P10

u_b=[-0.6531200792563073, -0.03376183950488629]; v_b=[-0.10158229468086878, -0.03763208952569348, 0.007720242737744301, -0.0024453794291365172]

|beta|strong group|canonical radius capture|termination|seed→endpoint strong norm|
|---:|---|---:|---|---:|
|0.0|OTHER_01|0|MAX_ITERATIONS|0.0712464|
|0.1|OTHER_01|0|TRUST_RADIUS_EXHAUSTED|0.0370161|
|0.2|CANONICAL|1|MAX_ITERATIONS|0.0570361|
|0.3|CANONICAL|1|MAX_ITERATIONS|0.0235321|
|0.4|CANONICAL|1|TRUST_RADIUS_EXHAUSTED|0.014523|
|0.5|CANONICAL|1|MAX_ITERATIONS|0.0300111|
|0.6|CANONICAL|1|MAX_ITERATIONS|0.0149014|
|0.7|CANONICAL|1|MAX_ITERATIONS|0.000864909|
|0.8|CANONICAL|1|TRUST_RADIUS_EXHAUSTED|0.000641619|
|0.9|CANONICAL|1|MAX_ITERATIONS|0.0122089|
|1.0|CANONICAL|1|TRUST_RADIUS_EXHAUSTED|0.000216666|

Local cross capture (.02/.05/.10): {'0.02': {'captured': 8, 'total': 8}, '0.05': {'captured': 5, 'total': 8}, '0.1': {'captured': 5, 'total': 8}}

Bisection: [{'transaction_id': '616', 'cluster_id': 'P10', 'beta_low': '0.14375000000000002', 'beta_high': '0.15000000000000002', 'width': '0.0062500000000000056', 'low_branch': 'OTHER_01', 'high_branch': 'CANONICAL', 'depth': '4', 'boundary_kind': 'BUDGETED_ENDPOINT_GROUP_SECTION'}]

```json
{
  "support": {
    "largest_support_switch": 0.01675977653631285,
    "largest_support_switch_beta": 0.51,
    "largest_dynamic_cusp_beta": 0.82,
    "dynamic_abs_second_difference_max": 0.007206219976753747,
    "frozen_t0_abs_second_difference_max": 7.474384897232511e-06,
    "frozen_canonical_abs_second_difference_max": 7.814093913260933e-06,
    "top_support_beta": [
      0.51,
      0.19,
      0.29,
      0.53,
      0.69
    ],
    "top_cusp_beta": [
      0.82,
      0.08,
      0.18,
      0.51,
      0.53
    ],
    "per_boundary_association": [
      {
        "beta_low": 0.14375000000000002,
        "beta_high": 0.15000000000000002,
        "support_hit": false,
        "cusp_hit": false
      }
    ],
    "associated": "NO"
  },
  "matched_pattern": {
    "DYNAMIC": {
      "endpoint_groups": 4,
      "canonical_capture": 1,
      "statuses": {
        "EVALUATION_BUDGET": 5
      },
      "dynamic_energies": [
        -3.0737468482858947,
        -3.072607990699917,
        -3.0737191932124293,
        -3.0819452209075475,
        -3.08774866953453
      ],
      "objective_energies": [
        -3.0737468482858947,
        -3.072607990699917,
        -3.0737191932124293,
        -3.0819452209075475,
        -3.08774866953453
      ]
    },
    "FROZEN_T0": {
      "endpoint_groups": 1,
      "canonical_capture": 0,
      "statuses": {
        "EVALUATION_BUDGET": 5
      },
      "dynamic_energies": [
        -3.044376216551408,
        -3.0425418126590733,
        -3.043423337346978,
        -3.046944533020153,
        -3.0426861232113915
      ],
      "objective_energies": [
        -2.840585301857347,
        -2.8405245675225084,
        -2.8402756025570604,
        -2.840580408939381,
        -2.8402858809165865
      ]
    },
    "FROZEN_CANONICAL": {
      "endpoint_groups": 1,
      "canonical_capture": 5,
      "statuses": {
        "EVALUATION_BUDGET": 4,
        "POLL_STEP_SMALL": 1
      },
      "dynamic_energies": [
        -3.0461470651493068,
        -3.0539348791274623,
        -3.051098924122663,
        -3.0544460448192248,
        -3.048753001181435
      ],
      "objective_energies": [
        -3.0488299909284904,
        -3.04924240721175,
        -3.049138983883489,
        -3.0492879220270517,
        -3.0493272560301965
      ]
    }
  },
  "global_profile_main": {
    "canonical_group_energy": -3.076321463394974,
    "closed_canonical_energy": -3.0493000375525288,
    "lowest_discovered_other_main_group_energy": null,
    "lowest_other_group_energy_without_pair_separation": -3.078922058043584,
    "other_branch": null,
    "other_to_canonical_representative_distance_m_deg": null,
    "global_min_would_erase_canonical_representative": false,
    "negative_result_means": "no witnessed lower separated main representative; not proof of global uniqueness",
    "strict_local_minimum_certified": false
  },
  "global_profile_all_part_a": {
    "lowest_separated_other_energy": -3.078922058043584,
    "other_group": "NEW_02",
    "distance_from_canonical_representative_m_deg": [
      0.02173408889808513,
      3.1009362648332437
    ],
    "lower_separated_representative_witness": true
  }
}
```

|branch|pre distance m/deg|post distance m/deg|iterations/status|escape|
|---|---|---|---|---|
|CANONICAL|0.020459/1.346745|0.038659/1.702713|5/SUCCESS|0|
|OTHER_01|0.023071/2.094361|0.039772/1.837200|6/SUCCESS|0|
|NEW_01|0.003216/0.833811|0.024557/0.326251|4/SUCCESS|0|
|NEW_02|0.010139/2.764147|0.037765/1.146944|3/SUCCESS|0|

## 616/P12

u_b=[0.10718894733932126, 0.042801256028973035]; v_b=[0.010944362535181142, 0.02541425647714722, 0.018790343870562352, -0.0035912972867095864]

|beta|strong group|canonical radius capture|termination|seed→endpoint strong norm|
|---:|---|---:|---|---:|
|0.0|CANONICAL|1|BRANCH_GRADIENT_SMALL|0.012487|
|0.1|CANONICAL|1|BRANCH_GRADIENT_SMALL|0.00980627|
|0.2|CANONICAL|1|BRANCH_GRADIENT_SMALL|0.00752141|
|0.3|CANONICAL|1|STEP_SMALL|0.00651466|
|0.4|CANONICAL|1|TRUST_RADIUS_EXHAUSTED|1.67847e-05|
|0.5|CANONICAL|1|MAX_ITERATIONS|0.00283848|
|0.6|CANONICAL|1|TRUST_RADIUS_EXHAUSTED|0.00258664|
|0.7|CANONICAL|1|TRUST_RADIUS_EXHAUSTED|6.67572e-07|
|0.8|CANONICAL|1|MAX_ITERATIONS|0.00426697|
|0.9|CANONICAL|1|TRUST_RADIUS_EXHAUSTED|0.000461219|
|1.0|CANONICAL|1|STEP_SMALL|0|

Local cross capture (.02/.05/.10): {'0.02': {'captured': 8, 'total': 8}, '0.05': {'captured': 5, 'total': 8}, '0.1': {'captured': 4, 'total': 8}}

Bisection: []

```json
{
  "support": {
    "largest_support_switch": 0.0111731843575419,
    "largest_support_switch_beta": 0.32,
    "largest_dynamic_cusp_beta": 0.93,
    "dynamic_abs_second_difference_max": 0.005359279756309565,
    "frozen_t0_abs_second_difference_max": 2.6625868350116377e-06,
    "frozen_canonical_abs_second_difference_max": 2.6133661217286885e-06,
    "top_support_beta": [
      0.32,
      0.49,
      0.93,
      0.3,
      0.46
    ],
    "top_cusp_beta": [
      0.93,
      0.48,
      0.52,
      0.31,
      0.32
    ],
    "per_boundary_association": [],
    "associated": "NO"
  },
  "matched_pattern": {
    "DYNAMIC": {
      "endpoint_groups": 1,
      "canonical_capture": 0,
      "statuses": {
        "POLL_STEP_SMALL": 1,
        "EVALUATION_BUDGET": 4
      },
      "dynamic_energies": [
        -3.085492804623295,
        -3.0890966168276166,
        -3.091893429746301,
        -3.0935547335739737,
        -3.0915252389907546
      ],
      "objective_energies": [
        -3.085492804623295,
        -3.0890966168276166,
        -3.091893429746301,
        -3.0935547335739737,
        -3.0915252389907546
      ]
    },
    "FROZEN_T0": {
      "endpoint_groups": 1,
      "canonical_capture": 5,
      "statuses": {
        "POLL_STEP_SMALL": 1,
        "EVALUATION_BUDGET": 4
      },
      "dynamic_energies": [
        -3.0634461023863175,
        -3.0644980004902975,
        -3.061229569655992,
        -3.0614980841718515,
        -3.0597376327177335
      ],
      "objective_energies": [
        -3.0367461686762525,
        -3.036723231567703,
        -3.0367037315709755,
        -3.0366827898758904,
        -3.036340755631699
      ]
    },
    "FROZEN_CANONICAL": {
      "endpoint_groups": 1,
      "canonical_capture": 5,
      "statuses": {
        "EVALUATION_BUDGET": 4,
        "POLL_STEP_SMALL": 1
      },
      "dynamic_energies": [
        -3.0400106850102273,
        -3.042027802147035,
        -3.0454502711296847,
        -3.043832647848364,
        -3.0433733192506436
      ],
      "objective_energies": [
        -3.0430202727433002,
        -3.043317102687356,
        -3.0432590288348096,
        -3.043333118498615,
        -3.0433733192506436
      ]
    }
  },
  "global_profile_main": {
    "canonical_group_energy": -3.071320715500295,
    "closed_canonical_energy": -3.0433733264728438,
    "lowest_discovered_other_main_group_energy": null,
    "lowest_other_group_energy_without_pair_separation": -3.0790779256303944,
    "other_branch": null,
    "other_to_canonical_representative_distance_m_deg": null,
    "global_min_would_erase_canonical_representative": false,
    "negative_result_means": "no witnessed lower separated main representative; not proof of global uniqueness",
    "strict_local_minimum_certified": false
  },
  "global_profile_all_part_a": {
    "lowest_separated_other_energy": -3.0553583667807027,
    "other_group": "NEW_03",
    "distance_from_canonical_representative_m_deg": [
      0.006730635795740849,
      2.0268937821651876
    ],
    "lower_separated_representative_witness": false
  }
}
```

|branch|pre distance m/deg|post distance m/deg|iterations/status|escape|
|---|---|---|---|---|
|CANONICAL|0.004144/1.343639|0.016271/1.552301|3/SUCCESS|0|
|NEW_01|0.004012/1.107183|0.037434/1.960396|9/SUCCESS|0|
|NEW_02|0.004637/1.944781|0.016013/0.684510|4/SUCCESS|0|
|NEW_03|0.006582/2.817951|0.025754/1.807702|4/SUCCESS|0|
|NEW_04|0.009140/2.497079|0.024676/2.416104|4/SUCCESS|0|
|NEW_05|0.016518/4.436887|0.048287/2.886706|7/SUCCESS|0|

## 616/P13

u_b=[0.2254520738454328, 0.14435329541458952]; v_b=[-0.004735437528155997, 0.0686220936433447, 0.03353284081797035, -0.008454016086814224]

|beta|strong group|canonical radius capture|termination|seed→endpoint strong norm|
|---:|---|---:|---|---:|
|0.0|OTHER_01|0|MAX_ITERATIONS|0.0386349|
|0.1|OTHER_01|0|MAX_ITERATIONS|0.0305254|
|0.2|OTHER_01|0|MAX_ITERATIONS|0.00137477|
|0.3|OTHER_01|0|MAX_ITERATIONS|0.0204781|
|0.4|OTHER_01|0|MAX_ITERATIONS|0.00348498|
|0.5|CANONICAL|1|MAX_ITERATIONS|0.00592932|
|0.6|CANONICAL|1|MAX_ITERATIONS|0.00767361|
|0.7|CANONICAL|1|TRUST_RADIUS_EXHAUSTED|0.000170514|
|0.8|CANONICAL|1|MAX_ITERATIONS|0.00446718|
|0.9|CANONICAL|1|ENERGY_CHANGE_SMALL|0.00530096|
|1.0|CANONICAL|1|TRUST_RADIUS_EXHAUSTED|0.000624495|

Local cross capture (.02/.05/.10): {'0.02': {'captured': 8, 'total': 8}, '0.05': {'captured': 7, 'total': 8}, '0.1': {'captured': 5, 'total': 8}}

Bisection: [{'transaction_id': '616', 'cluster_id': 'P13', 'beta_low': '0.49375', 'beta_high': '0.5', 'width': '0.006249999999999978', 'low_branch': 'OTHER_01', 'high_branch': 'CANONICAL', 'depth': '4', 'boundary_kind': 'BUDGETED_ENDPOINT_GROUP_SECTION'}]

```json
{
  "support": {
    "largest_support_switch": 0.019553072625698324,
    "largest_support_switch_beta": 0.46,
    "largest_dynamic_cusp_beta": 0.21,
    "dynamic_abs_second_difference_max": 0.007294368622580816,
    "frozen_t0_abs_second_difference_max": 1.034050079695703e-05,
    "frozen_canonical_abs_second_difference_max": 1.0654205089721813e-05,
    "top_support_beta": [
      0.46,
      0.64,
      0.22,
      0.28,
      0.74
    ],
    "top_cusp_beta": [
      0.21,
      0.65,
      0.22,
      0.33,
      0.73
    ],
    "per_boundary_association": [
      {
        "beta_low": 0.49375,
        "beta_high": 0.5,
        "support_hit": false,
        "cusp_hit": false
      }
    ],
    "associated": "NO"
  },
  "matched_pattern": {
    "DYNAMIC": {
      "endpoint_groups": 3,
      "canonical_capture": 0,
      "statuses": {
        "EVALUATION_BUDGET": 5
      },
      "dynamic_energies": [
        -3.0765008707373824,
        -3.085143865623763,
        -3.080829461325956,
        -3.076121624790282,
        -3.081924459154223
      ],
      "objective_energies": [
        -3.0765008707373824,
        -3.085143865623763,
        -3.080829461325956,
        -3.076121624790282,
        -3.081924459154223
      ]
    },
    "FROZEN_T0": {
      "endpoint_groups": 1,
      "canonical_capture": 0,
      "statuses": {
        "EVALUATION_BUDGET": 5
      },
      "dynamic_energies": [
        -3.0518959547327538,
        -3.0522059541615625,
        -3.0496704399311483,
        -3.0491194151729313,
        -3.0484676729361175
      ],
      "objective_energies": [
        -2.9960543408699674,
        -2.9959587236419067,
        -2.995596468258308,
        -2.995854433648432,
        -2.9960277789275285
      ]
    },
    "FROZEN_CANONICAL": {
      "endpoint_groups": 1,
      "canonical_capture": 5,
      "statuses": {
        "EVALUATION_BUDGET": 4,
        "POLL_STEP_SMALL": 1
      },
      "dynamic_energies": [
        -3.030289923645355,
        -3.0295826546773714,
        -3.0257335062331654,
        -3.030282060931403,
        -3.033021512201037
      ],
      "objective_energies": [
        -3.0315470976648413,
        -3.031469938534396,
        -3.031600872918619,
        -3.0317249773913697,
        -3.0317521694182363
      ]
    }
  },
  "global_profile_main": {
    "canonical_group_energy": -3.0467537001131713,
    "closed_canonical_energy": -3.0317503362237197,
    "lowest_discovered_other_main_group_energy": null,
    "lowest_other_group_energy_without_pair_separation": -3.0668745907035726,
    "other_branch": null,
    "other_to_canonical_representative_distance_m_deg": null,
    "global_min_would_erase_canonical_representative": false,
    "negative_result_means": "no witnessed lower separated main representative; not proof of global uniqueness",
    "strict_local_minimum_certified": false
  },
  "global_profile_all_part_a": {
    "lowest_separated_other_energy": -3.0521567245938535,
    "other_group": "NEW_03",
    "distance_from_canonical_representative_m_deg": [
      0.020854233489290403,
      4.784441203614916
    ],
    "lower_separated_representative_witness": true
  }
}
```

|branch|pre distance m/deg|post distance m/deg|iterations/status|escape|
|---|---|---|---|---|
|CANONICAL|0.005005/1.906323|0.015703/1.599331|3/SUCCESS|0|
|OTHER_01|0.004262/2.578630|0.099606/4.263999|12/SUCCESS|0|
|NEW_01|0.052022/0.659877|0.022734/0.506033|8/SUCCESS|0|
|NEW_02|0.007759/3.085416|0.042393/1.725968|7/SUCCESS|0|
|NEW_03|0.020185/5.526320|0.100164/5.698092|9/SUCCESS|0|

## 616/P17

u_b=[0.16730263552927827, 0.14107179780210732]; v_b=[0.0038607443449266733, 0.08390619963356893, 0.053406835396749126, -0.005881960323256362]

|beta|strong group|canonical radius capture|termination|seed→endpoint strong norm|
|---:|---|---:|---|---:|
|0.0|OTHER_01|0|MAX_ITERATIONS|0.017129|
|0.1|OTHER_01|0|MAX_ITERATIONS|0.0168144|
|0.2|OTHER_01|0|MAX_ITERATIONS|0.0132782|
|0.3|OTHER_01|0|ENERGY_CHANGE_SMALL|0.00394477|
|0.4|OTHER_01|0|MAX_ITERATIONS|0.00911717|
|0.5|OTHER_02|0|MAX_ITERATIONS|0.00496769|
|0.6|CANONICAL|1|MAX_ITERATIONS|0.00882101|
|0.7|CANONICAL|1|MAX_ITERATIONS|0.00237736|
|0.8|CANONICAL|1|MAX_ITERATIONS|0.000195873|
|0.9|CANONICAL|1|TRUST_RADIUS_EXHAUSTED|0.000675051|
|1.0|CANONICAL|1|BRANCH_GRADIENT_SMALL|7.91384e-05|

Local cross capture (.02/.05/.10): {'0.02': {'captured': 8, 'total': 8}, '0.05': {'captured': 5, 'total': 8}, '0.1': {'captured': 6, 'total': 8}}

Bisection: [{'transaction_id': '616', 'cluster_id': 'P17', 'beta_low': '0.41875000000000007', 'beta_high': '0.42500000000000004', 'width': '0.006249999999999978', 'low_branch': 'OTHER_01', 'high_branch': 'OTHER_02', 'depth': '4', 'boundary_kind': 'BUDGETED_ENDPOINT_GROUP_SECTION'}, {'transaction_id': '616', 'cluster_id': 'P17', 'beta_low': '0.5625', 'beta_high': '0.56875', 'width': '0.006249999999999978', 'low_branch': 'OTHER_02', 'high_branch': 'CANONICAL', 'depth': '4', 'boundary_kind': 'BUDGETED_ENDPOINT_GROUP_SECTION'}]

```json
{
  "support": {
    "largest_support_switch": 0.019553072625698324,
    "largest_support_switch_beta": 0.42,
    "largest_dynamic_cusp_beta": 0.97,
    "dynamic_abs_second_difference_max": 0.007666985756712652,
    "frozen_t0_abs_second_difference_max": 1.998123479163283e-05,
    "frozen_canonical_abs_second_difference_max": 2.1150032702976773e-05,
    "top_support_beta": [
      0.42,
      0.49,
      0.2,
      0.21,
      0.24
    ],
    "top_cusp_beta": [
      0.97,
      0.09,
      0.11,
      0.18,
      0.46
    ],
    "per_boundary_association": [
      {
        "beta_low": 0.41875000000000007,
        "beta_high": 0.42500000000000004,
        "support_hit": true,
        "cusp_hit": false
      },
      {
        "beta_low": 0.5625,
        "beta_high": 0.56875,
        "support_hit": false,
        "cusp_hit": false
      }
    ],
    "associated": "PARTIAL"
  },
  "matched_pattern": {
    "DYNAMIC": {
      "endpoint_groups": 2,
      "canonical_capture": 0,
      "statuses": {
        "EVALUATION_BUDGET": 5
      },
      "dynamic_energies": [
        -3.0843415892955104,
        -3.0850622706040802,
        -3.079306440352081,
        -3.0806049903884976,
        -3.076593977596492
      ],
      "objective_energies": [
        -3.0843415892955104,
        -3.0850622706040802,
        -3.079306440352081,
        -3.0806049903884976,
        -3.076593977596492
      ]
    },
    "FROZEN_T0": {
      "endpoint_groups": 1,
      "canonical_capture": 0,
      "statuses": {
        "POLL_STEP_SMALL": 1,
        "EVALUATION_BUDGET": 4
      },
      "dynamic_energies": [
        -3.051112787805941,
        -3.0518885348405442,
        -3.0569251655429177,
        -3.04974972430127,
        -3.0496446351998894
      ],
      "objective_energies": [
        -3.009424432736749,
        -3.0093417790519803,
        -3.0091293786612034,
        -3.0088003047450225,
        -3.0077205864410685
      ]
    },
    "FROZEN_CANONICAL": {
      "endpoint_groups": 1,
      "canonical_capture": 5,
      "statuses": {
        "EVALUATION_BUDGET": 4,
        "POLL_STEP_SMALL": 1
      },
      "dynamic_energies": [
        -3.0276451303098155,
        -3.0238449896871167,
        -3.030334763759784,
        -3.0287280990849856,
        -3.025408368265518
      ],
      "objective_energies": [
        -3.0251737150033504,
        -3.0248731280509182,
        -3.0253508010041945,
        -3.0253925230096166,
        -3.025408368265518
      ]
    }
  },
  "global_profile_main": {
    "canonical_group_energy": -3.03585249630662,
    "closed_canonical_energy": -3.0254084282004485,
    "lowest_discovered_other_main_group_energy": -3.0700866331693395,
    "lowest_other_group_energy_without_pair_separation": -3.0764387174337284,
    "other_branch": "OTHER_01",
    "other_to_canonical_representative_distance_m_deg": [
      0.003620053649108596,
      2.6392934278937767
    ],
    "global_min_would_erase_canonical_representative": true,
    "negative_result_means": "no witnessed lower separated main representative; not proof of global uniqueness",
    "strict_local_minimum_certified": false
  },
  "global_profile_all_part_a": {
    "lowest_separated_other_energy": -3.0764387174337284,
    "other_group": "NEW_03",
    "distance_from_canonical_representative_m_deg": [
      0.01365252733621447,
      3.036587587136986
    ],
    "lower_separated_representative_witness": true
  }
}
```

|branch|pre distance m/deg|post distance m/deg|iterations/status|escape|
|---|---|---|---|---|
|CANONICAL|0.001592/1.850512|0.013528/1.423167|4/SUCCESS|0|
|OTHER_01|0.004556/4.479736|0.070192/5.361610|8/SUCCESS|0|
|OTHER_02|0.002663/2.614396|0.016847/2.076289|3/SUCCESS|0|
|NEW_01|0.001259/0.459230|0.001840/0.407984|1/SUCCESS|0|
|NEW_02|0.009991/2.736515|0.013343/1.081140|5/SUCCESS|0|
|NEW_03|0.013662/4.704676|0.070193/5.361596|7/SUCCESS|0|

## 2226/P09

u_b=[0.21613437119718437, 1.1159776608082246]; v_b=[0.12862060504342168, -0.15956404399459712, 0.013215510209065195, -0.07937295264003402]

|beta|strong group|canonical radius capture|termination|seed→endpoint strong norm|
|---:|---|---:|---|---:|
|0.0|OTHER_01|0|MAX_ITERATIONS|0.104457|
|0.1|OTHER_02|0|MAX_ITERATIONS|0.149454|
|0.2|OTHER_02|0|MAX_ITERATIONS|0.128023|
|0.3|OTHER_02|0|MAX_ITERATIONS|0.10576|
|0.4|OTHER_02|0|MAX_ITERATIONS|0.0853765|
|0.5|OTHER_02|0|MAX_ITERATIONS|0.0639807|
|0.6|OTHER_02|0|MAX_ITERATIONS|0.0381907|
|0.7|OTHER_02|0|MAX_ITERATIONS|0.0211889|
|0.8|OTHER_02|0|MAX_ITERATIONS|0.0172648|
|0.9|CANONICAL|1|MAX_ITERATIONS|0.00631703|
|1.0|CANONICAL|1|TRUST_RADIUS_EXHAUSTED|0.000110484|

Local cross capture (.02/.05/.10): {'0.02': {'captured': 8, 'total': 8}, '0.05': {'captured': 6, 'total': 8}, '0.1': {'captured': 5, 'total': 8}}

Bisection: [{'transaction_id': '2226', 'cluster_id': 'P09', 'beta_low': '0.00625', 'beta_high': '0.0125', 'width': '0.00625', 'low_branch': 'OTHER_01', 'high_branch': 'OTHER_02', 'depth': '4', 'boundary_kind': 'BUDGETED_ENDPOINT_GROUP_SECTION'}, {'transaction_id': '2226', 'cluster_id': 'P09', 'beta_low': '0.8187500000000001', 'beta_high': '0.8250000000000001', 'width': '0.006249999999999978', 'low_branch': 'OTHER_02', 'high_branch': 'CANONICAL', 'depth': '4', 'boundary_kind': 'BUDGETED_ENDPOINT_GROUP_SECTION'}]

```json
{
  "support": {
    "largest_support_switch": 0.030685920577617327,
    "largest_support_switch_beta": 0.46,
    "largest_dynamic_cusp_beta": 0.96,
    "dynamic_abs_second_difference_max": 0.009240786430487091,
    "frozen_t0_abs_second_difference_max": 4.284696999401483e-05,
    "frozen_canonical_abs_second_difference_max": 0.00011496434964763935,
    "top_support_beta": [
      0.46,
      0.93,
      0.94,
      0.01,
      0.31
    ],
    "top_cusp_beta": [
      0.96,
      0.83,
      0.35,
      0.78,
      0.08
    ],
    "per_boundary_association": [
      {
        "beta_low": 0.00625,
        "beta_high": 0.0125,
        "support_hit": true,
        "cusp_hit": false
      },
      {
        "beta_low": 0.8187500000000001,
        "beta_high": 0.8250000000000001,
        "support_hit": false,
        "cusp_hit": true
      }
    ],
    "associated": "PARTIAL"
  },
  "matched_pattern": {
    "DYNAMIC": {
      "endpoint_groups": 3,
      "canonical_capture": 3,
      "statuses": {
        "EVALUATION_BUDGET": 5
      },
      "dynamic_energies": [
        -2.8787294641401298,
        -2.8956742727573666,
        -2.8929222721087564,
        -2.892526722743271,
        -2.8956258257913094
      ],
      "objective_energies": [
        -2.8787294641401298,
        -2.8956742727573666,
        -2.8929222721087564,
        -2.892526722743271,
        -2.8956258257913094
      ]
    },
    "FROZEN_T0": {
      "endpoint_groups": 1,
      "canonical_capture": 0,
      "statuses": {
        "EVALUATION_BUDGET": 5
      },
      "dynamic_energies": [
        -2.6658076790178153,
        -2.6531311481284465,
        -2.535938147015921,
        -2.6873537534317857,
        -2.650681856442321
      ],
      "objective_energies": [
        -1.09511935172406,
        -1.0936611487506775,
        -1.0905465143253197,
        -1.0904429601436205,
        -1.0943252005612307
      ]
    },
    "FROZEN_CANONICAL": {
      "endpoint_groups": 1,
      "canonical_capture": 5,
      "statuses": {
        "EVALUATION_BUDGET": 4,
        "POLL_STEP_SMALL": 1
      },
      "dynamic_energies": [
        -2.872986355889546,
        -2.872529929794151,
        -2.879453815983239,
        -2.883231430357388,
        -2.881692463647063
      ],
      "objective_energies": [
        -2.879276478309442,
        -2.8799628485179896,
        -2.8814171142163403,
        -2.8811484484910883,
        -2.881692463647063
      ]
    }
  },
  "global_profile_main": {
    "canonical_group_energy": -2.884399216513999,
    "closed_canonical_energy": -2.8816924971624447,
    "lowest_discovered_other_main_group_energy": -2.868442984950582,
    "lowest_other_group_energy_without_pair_separation": -2.8869656607235075,
    "other_branch": "OTHER_01",
    "other_to_canonical_representative_distance_m_deg": [
      0.053871166308178016,
      4.503010534221522
    ],
    "global_min_would_erase_canonical_representative": false,
    "negative_result_means": "no witnessed lower separated main representative; not proof of global uniqueness",
    "strict_local_minimum_certified": false
  },
  "global_profile_all_part_a": {
    "lowest_separated_other_energy": -2.868442984950582,
    "other_group": "OTHER_01",
    "distance_from_canonical_representative_m_deg": [
      0.053871166308178016,
      4.503010534221522
    ],
    "lower_separated_representative_witness": false
  }
}
```

|branch|pre distance m/deg|post distance m/deg|iterations/status|escape|
|---|---|---|---|---|
|CANONICAL|0.014777/0.899530|0.806450/15.272398|68/SUCCESS|1|
|OTHER_01|0.068499/5.395384|0.805734/15.247442|61/SUCCESS|0|
|OTHER_02|0.026508/2.367926|0.805734/15.247438|67/SUCCESS|0|
|NEW_01|0.030123/1.394224|0.805967/15.258929|73/SUCCESS|1|

## Interpretation and limits

All seven canonical neighborhoods recover8/8 at the .02 local cross. This is a finite neighborhood under Newton20, not merely an exact oracle seed.

Matched frozen objectives and dynamic pattern have different endpoint behavior. Freezing support changes the actual objective, and its choice selects a different neighborhood. This intervention demonstrates support-dependent attraction, not support as the sole cause. Finite budgets and broad grouping thresholds remain limitations.

Several main endpoint groups are unfinished iterates. A group switch can be a .2m/2deg threshold crossing rather than an abrupt stable-attractor switch. No intrinsic strong multimodality certificate is inferred.

Global-profile comparisons require both CLOSED-anchor and representative-to-representative separation. Different complete-link labels alone do not qualify. A positive comparison proves a lower-energy separated feasible endpoint exists relative to the chosen canonical representative, not global or strict-local minima. Negative comparisons mean no witness in the tested main representatives, not global uniqueness. The paper formula is not changed by this experiment.

The full-refine map is isolated: one actual MAIN representative per frozen MAIN group plus one local/bisection representative per supplemental NEW group. Every PART A group is covered; PART B controls are not refined. This is not an escape-rate estimate for every near-canonical endpoint.

Validation independently reconstructs poses from archived W/S and u/v, checks input SHA256, request coverage, accepted traces, matrix distances, selected representatives and escape flags. The tests are the five P9 tests, not the production localization suite.
