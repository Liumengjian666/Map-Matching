# PAPER-P6-I1-BRANCHED-RECOVERY-PROTOTYPE

Result: `LOCAL_ONLY_IMPROVEMENT`.

## Baseline gate

```json
{
  "predictor_t_difference_m": 4.984983334952808e-14,
  "predictor_r_difference_deg": 5.036540923925453e-05,
  "raw_t_difference_m": 0,
  "raw_r_difference_deg": 0.03886516117896221,
  "used_t_difference_m": 0,
  "used_r_difference_deg": 0.03182115320856514,
  "corrected_t_difference_m": 3.9203647687275676e-14,
  "corrected_r_difference_deg": 5.036540923925453e-05
}
```
All five trajectories were generated before the official GT was opened. The P4 shared evaluator alignment is one fixed left anchor from the first BASELINE corrected IMU pose; there is no per-mode fitting or GT-informed branch selection.

## Modes

```json
{
  "BASELINE": {
    "global_t_rmse_improvement": 0.0,
    "late_t_rmse_improvement": 0.0,
    "global_r_rmse_ratio": 1.0,
    "crossing_2m_s": 151.48321318626404,
    "crossing_5m_s": 157.43361401557922
  },
  "DCREG_ONLY": {
    "global_t_rmse_improvement": -2.1088265626303038,
    "late_t_rmse_improvement": -2.109830620512976,
    "global_r_rmse_ratio": 1.2323098193670916,
    "crossing_2m_s": 151.78579545021057,
    "crossing_5m_s": 157.43361401557922
  },
  "MULTISTART_OBJECTIVE": {
    "global_t_rmse_improvement": 0.9593575135190489,
    "late_t_rmse_improvement": 0.9685515291721591,
    "global_r_rmse_ratio": 0.27391195464155293,
    "crossing_2m_s": null,
    "crossing_5m_s": null
  },
  "MULTISTART_VISUAL": {
    "global_t_rmse_improvement": 0.9020987364388816,
    "late_t_rmse_improvement": 0.905793695323079,
    "global_r_rmse_ratio": 0.4840225856428974,
    "crossing_2m_s": 151.28149318695068,
    "crossing_5m_s": null
  },
  "FULL_ROUTER": {
    "global_t_rmse_improvement": -0.25937812712324404,
    "late_t_rmse_improvement": -0.2595592855130222,
    "global_r_rmse_ratio": 0.9995012979700679,
    "crossing_2m_s": 150.4746642112732,
    "crossing_5m_s": 165.6028254032135
  }
}
```

## Ablation answers and verdict evidence

- Q1 DCREG_ONLY: it did not improve global or 150s-end translation RMSE; it reached 447 degenerate branches, and among converged degenerate branch frames it was closer to anchored GT in 254/447 cases.
- Q2 MULTISTART_OBJECTIVE: objective-only multi-start improved global translation RMSE by 95.9% and 150s-end RMSE by 96.9%; its selected corrected pose had lower post-hoc translation error than baseline on 3607/4126 evaluated frames.
- Q3 MULTISTART_VISUAL: visual arbitration did not improve the aggregate result over objective-only (global translation RMSE 2.148 m vs 0.892 m); it was closer than objective-only on 852/4126 frames, so the sequence-level evidence does not show a net visual-arbitration gain.
- Q4 FULL_ROUTER: not better than baseline or objective-only globally; translation RMSE improvement vs baseline was -25.9%, rotation RMSE ratio was 1.000, 5 m crossing delay was 8.17 s (required at least 20 s), and sustained recovery events were 0.
- Aggregate 150s-end local-predictor translation RMSE decreased from 0.284 m to 0.171 m (39.8%); therefore the task-defined verdict is `LOCAL_ONLY_IMPROVEMENT`. Segment-level results are mixed (see `segment_metrics.csv`; 300–350 s worsens), so this is not a uniform per-segment claim.

## Full trajectory metrics

```json
[
  {
    "mode": "BASELINE",
    "segment": "all",
    "count": 4126,
    "t_mean": 14.660182042205129,
    "t_rmse": 21.936132729375835,
    "t_median": 6.034344271830071,
    "t_p95": 46.757084095160685,
    "t_max": 54.298934504711355,
    "r_mean": 7.602125629646099,
    "r_rmse": 9.906078073266107,
    "r_median": 5.256541785546897,
    "r_p95": 19.08964428500651,
    "r_max": 22.682863869442333
  },
  {
    "mode": "BASELINE",
    "segment": "150-end",
    "count": 2644,
    "t_mean": 22.443868635217253,
    "t_rmse": 27.392906834641643,
    "t_median": 20.19096570197663,
    "t_p95": 50.42933903691806,
    "t_max": 54.298934504711355,
    "r_mean": 10.643158040173066,
    "r_rmse": 12.175511106160773,
    "r_median": 11.478670562399573,
    "r_p95": 19.747661909530912,
    "r_max": 22.682863869442333
  },
  {
    "mode": "DCREG_ONLY",
    "segment": "all",
    "count": 4126,
    "t_mean": 50.628755777491456,
    "t_rmse": 68.19563211046759,
    "t_median": 68.79567361015933,
    "t_p95": 113.30308603207465,
    "t_max": 128.15400872681317,
    "r_mean": 9.311387159495387,
    "r_rmse": 12.207357281102864,
    "r_median": 7.777084726501004,
    "r_p95": 24.510126865490285,
    "r_max": 29.451091331032064
  },
  {
    "mode": "DCREG_ONLY",
    "segment": "150-end",
    "count": 2644,
    "t_mean": 78.58814755325304,
    "t_rmse": 85.18730045922777,
    "t_median": 81.95933396646242,
    "t_p95": 119.14762206281337,
    "t_max": 128.15400872681317,
    "r_mean": 13.314039744190556,
    "r_rmse": 15.08483100769658,
    "r_median": 13.87563710574204,
    "r_p95": 26.028524281299774,
    "r_max": 29.451091331032064
  },
  {
    "mode": "MULTISTART_OBJECTIVE",
    "segment": "all",
    "count": 4126,
    "t_mean": 0.7352673629029501,
    "t_rmse": 0.8915389778980062,
    "t_median": 0.5770612868741174,
    "t_p95": 1.583456986094081,
    "t_max": 1.8067101061629338,
    "r_mean": 2.2191112233919847,
    "r_rmse": 2.7133932078801477,
    "r_median": 1.7915032330855112,
    "r_p95": 5.590391301240942,
    "r_max": 12.469733679113734
  },
  {
    "mode": "MULTISTART_OBJECTIVE",
    "segment": "150-end",
    "count": 2644,
    "t_mean": 0.7328163532115683,
    "t_rmse": 0.8614650314789907,
    "t_median": 0.5987323798613755,
    "t_p95": 1.4807211722246785,
    "t_max": 1.7423286219050182,
    "r_mean": 2.4329606465767415,
    "r_rmse": 2.920277826537706,
    "r_median": 1.9037177219742971,
    "r_p95": 5.876702921413054,
    "r_max": 9.324683502560164
  },
  {
    "mode": "MULTISTART_VISUAL",
    "segment": "all",
    "count": 4126,
    "t_mean": 1.7152977270107554,
    "t_rmse": 2.1475751118502995,
    "t_median": 1.66580795394482,
    "t_p95": 3.7649814096724374,
    "t_max": 4.669088876858145,
    "r_mean": 3.698086888004055,
    "r_rmse": 4.794765522602672,
    "r_median": 2.4409495719441177,
    "r_p95": 9.613171074729097,
    "r_max": 12.750769660862312
  },
  {
    "mode": "MULTISTART_VISUAL",
    "segment": "150-end",
    "count": 2644,
    "t_mean": 2.245733961262868,
    "t_rmse": 2.5805845272507635,
    "t_median": 2.4601737887305584,
    "t_p95": 4.062241948103097,
    "t_max": 4.669088876858145,
    "r_mean": 4.562077646807981,
    "r_rmse": 5.592164154039125,
    "r_median": 3.410999780141661,
    "r_p95": 10.262645152849705,
    "r_max": 12.750769660862312
  },
  {
    "mode": "FULL_ROUTER",
    "segment": "all",
    "count": 4126,
    "t_mean": 19.021215984671297,
    "t_rmse": 27.625885753048234,
    "t_median": 6.667488314971325,
    "t_p95": 53.08571027608878,
    "t_max": 54.39509832973801,
    "r_mean": 7.578498927639119,
    "r_rmse": 9.901137892022303,
    "r_median": 6.457643743781823,
    "r_p95": 20.029293229316732,
    "r_max": 23.620317635038703
  },
  {
    "mode": "FULL_ROUTER",
    "segment": "150-end",
    "count": 2644,
    "t_mean": 29.266570794452804,
    "t_rmse": 34.50299016076601,
    "t_median": 33.770766146634216,
    "t_p95": 53.34036907319293,
    "t_max": 54.39509832973801,
    "r_mean": 10.658368422737462,
    "r_rmse": 12.191355768848915,
    "r_median": 10.341590475940658,
    "r_p95": 20.65264158766675,
    "r_max": 23.620317635038703
  }
]
```

## Segment metrics

```json
[
  {
    "mode": "BASELINE",
    "segment": "all",
    "count": 4126,
    "local_predictor_t_rmse_m": 0.22830059885130438,
    "local_predictor_r_rmse_deg": 0.7909327846957829,
    "t_mean": 14.660182042205129,
    "t_rmse": 21.936132729375835,
    "t_median": 6.034344271830071,
    "t_p95": 46.757084095160685,
    "t_max": 54.298934504711355,
    "r_mean": 7.602125629646099,
    "r_rmse": 9.906078073266107,
    "r_median": 5.256541785546897,
    "r_p95": 19.08964428500651,
    "r_max": 22.682863869442333
  },
  {
    "mode": "BASELINE",
    "segment": "150-end",
    "count": 2644,
    "local_predictor_t_rmse_m": 0.2843900486720956,
    "local_predictor_r_rmse_deg": 0.7773312636037508,
    "t_mean": 22.443868635217253,
    "t_rmse": 27.392906834641643,
    "t_median": 20.19096570197663,
    "t_p95": 50.42933903691806,
    "t_max": 54.298934504711355,
    "r_mean": 10.643158040173066,
    "r_rmse": 12.175511106160773,
    "r_median": 11.478670562399573,
    "r_p95": 19.747661909530912,
    "r_max": 22.682863869442333
  },
  {
    "mode": "BASELINE",
    "segment": "0-50",
    "count": 491,
    "local_predictor_t_rmse_m": 0.016378843524973808,
    "local_predictor_r_rmse_deg": 0.5899310928887558,
    "t_mean": 0.23346344869978328,
    "t_rmse": 0.2524884207820451,
    "t_median": 0.22441695061519226,
    "t_p95": 0.4056383773969965,
    "t_max": 0.438683041572821,
    "r_mean": 1.508234817002313,
    "r_rmse": 1.9980265244019766,
    "r_median": 0.9858584625007832,
    "r_p95": 4.210234936798447,
    "r_max": 5.314630985665611
  },
  {
    "mode": "BASELINE",
    "segment": "50-100",
    "count": 496,
    "local_predictor_t_rmse_m": 0.025365382596180798,
    "local_predictor_r_rmse_deg": 0.910382237210887,
    "t_mean": 0.544744565594205,
    "t_rmse": 0.6271268632277923,
    "t_median": 0.4636913598937501,
    "t_p95": 1.1780074304895762,
    "t_max": 1.2022258194156579,
    "r_mean": 1.87780505801661,
    "r_rmse": 2.249941179016896,
    "r_median": 1.640633479169567,
    "r_p95": 3.89510851910942,
    "r_max": 12.46829772290136
  },
  {
    "mode": "BASELINE",
    "segment": "100-150",
    "count": 495,
    "local_predictor_t_rmse_m": 0.037834905740875606,
    "local_predictor_r_rmse_deg": 0.9003509679323554,
    "t_mean": 1.5383809631871102,
    "t_rmse": 1.5556363203801027,
    "t_median": 1.5469048909208856,
    "t_p95": 1.8128152209535013,
    "t_max": 2.242602082665403,
    "r_mean": 3.139244213692627,
    "r_rmse": 4.134185094604907,
    "r_median": 2.040835683980531,
    "r_p95": 9.504708449604836,
    "r_max": 10.106850765150043
  },
  {
    "mode": "BASELINE",
    "segment": "150-200",
    "count": 496,
    "local_predictor_t_rmse_m": 0.2804334998374871,
    "local_predictor_r_rmse_deg": 0.8776125655099739,
    "t_mean": 18.645812473096576,
    "t_rmse": 22.968057571835548,
    "t_median": 13.153068870734145,
    "t_p95": 44.58225427235899,
    "t_max": 45.59630877017078,
    "r_mean": 12.48242811314886,
    "r_rmse": 12.605871557383873,
    "r_median": 12.359547485216993,
    "r_p95": 14.545247257821696,
    "r_max": 21.097128116840874
  },
  {
    "mode": "BASELINE",
    "segment": "200-250",
    "count": 495,
    "local_predictor_t_rmse_m": 0.26262430714154844,
    "local_predictor_r_rmse_deg": 0.802817447597735,
    "t_mean": 26.281376568108396,
    "t_rmse": 28.16081877384434,
    "t_median": 28.06050774702622,
    "t_p95": 40.61795246054182,
    "t_max": 41.24335607853048,
    "r_mean": 17.980836958230608,
    "r_rmse": 18.10624418336962,
    "r_median": 17.503002993658015,
    "r_p95": 21.652940376861217,
    "r_max": 22.682863869442333
  },
  {
    "mode": "BASELINE",
    "segment": "250-300",
    "count": 496,
    "local_predictor_t_rmse_m": 0.18337968254148748,
    "local_predictor_r_rmse_deg": 0.6700536921005744,
    "t_mean": 13.437909577888163,
    "t_rmse": 17.154813677052317,
    "t_median": 7.1394321189556536,
    "t_p95": 33.59610963058723,
    "t_max": 33.73334927281516,
    "r_mean": 15.410936326854534,
    "r_rmse": 15.549935270124106,
    "r_median": 15.37286086951904,
    "r_p95": 19.474022504284196,
    "r_max": 20.080059957106123
  },
  {
    "mode": "BASELINE",
    "segment": "300-350",
    "count": 496,
    "local_predictor_t_rmse_m": 0.1517819839581037,
    "local_predictor_r_rmse_deg": 0.9468193429280304,
    "t_mean": 6.999279125677945,
    "t_rmse": 8.061316427252054,
    "t_median": 5.80883559770633,
    "t_p95": 17.32255016103639,
    "t_max": 27.435135694972207,
    "r_mean": 4.946075816956185,
    "r_rmse": 5.469157044939282,
    "r_median": 4.083753278722884,
    "r_p95": 10.205564725047601,
    "r_max": 12.806758457387787
  },
  {
    "mode": "BASELINE",
    "segment": "350-end",
    "count": 661,
    "local_predictor_t_rmse_m": 0.41270766484954013,
    "local_predictor_r_rmse_deg": 0.5888869472454994,
    "t_mean": 40.767229480599205,
    "t_rmse": 41.73934484260686,
    "t_median": 40.674295819182404,
    "t_p95": 53.36940041349662,
    "t_max": 54.298934504711355,
    "r_mean": 4.465405743481813,
    "r_rmse": 4.933478678746598,
    "r_median": 4.742536695487551,
    "r_p95": 7.59850708346158,
    "r_max": 9.493194791454831
  },
  {
    "mode": "DCREG_ONLY",
    "segment": "all",
    "count": 4126,
    "local_predictor_t_rmse_m": 0.2705490857902477,
    "local_predictor_r_rmse_deg": 0.7909563264957616,
    "t_mean": 50.628755777491456,
    "t_rmse": 68.19563211046759,
    "t_median": 68.79567361015933,
    "t_p95": 113.30308603207465,
    "t_max": 128.15400872681317,
    "r_mean": 9.311387159495387,
    "r_rmse": 12.207357281102864,
    "r_median": 7.777084726501004,
    "r_p95": 24.510126865490285,
    "r_max": 29.451091331032064
  },
  {
    "mode": "DCREG_ONLY",
    "segment": "150-end",
    "count": 2644,
    "local_predictor_t_rmse_m": 0.33729373277696284,
    "local_predictor_r_rmse_deg": 0.7773709023732398,
    "t_mean": 78.58814755325304,
    "t_rmse": 85.18730045922777,
    "t_median": 81.95933396646242,
    "t_p95": 119.14762206281337,
    "t_max": 128.15400872681317,
    "r_mean": 13.314039744190556,
    "r_rmse": 15.08483100769658,
    "r_median": 13.87563710574204,
    "r_p95": 26.028524281299774,
    "r_max": 29.451091331032064
  },
  {
    "mode": "DCREG_ONLY",
    "segment": "0-50",
    "count": 491,
    "local_predictor_t_rmse_m": 0.015953634964642697,
    "local_predictor_r_rmse_deg": 0.589917163653011,
    "t_mean": 0.19021488420625565,
    "t_rmse": 0.22960814771870391,
    "t_median": 0.18895331028453977,
    "t_p95": 0.4101779404528272,
    "t_max": 0.44882228927643913,
    "r_mean": 1.4282595662721413,
    "r_rmse": 1.9865796866115562,
    "r_median": 0.9894883812509321,
    "r_p95": 4.341466785814724,
    "r_max": 5.295486047926746
  },
  {
    "mode": "DCREG_ONLY",
    "segment": "50-100",
    "count": 496,
    "local_predictor_t_rmse_m": 0.02383809029843783,
    "local_predictor_r_rmse_deg": 0.9103779938503621,
    "t_mean": 0.5286668343296718,
    "t_rmse": 0.6072700283442494,
    "t_median": 0.43545456218166834,
    "t_p95": 1.1626220107119463,
    "t_max": 1.2024513940565298,
    "r_mean": 1.8910875498931854,
    "r_rmse": 2.2506164382114267,
    "r_median": 1.7253526181037553,
    "r_p95": 3.822466523808269,
    "r_max": 12.405710822632429
  },
  {
    "mode": "DCREG_ONLY",
    "segment": "100-150",
    "count": 495,
    "local_predictor_t_rmse_m": 0.03840642781947697,
    "local_predictor_r_rmse_deg": 0.9003538419276683,
    "t_mean": 1.5183231296078976,
    "t_rmse": 1.53660079075855,
    "t_median": 1.4953825604241553,
    "t_p95": 1.8359911888795788,
    "t_max": 2.2710513340343845,
    "r_mean": 3.1862777063666567,
    "r_rmse": 4.206013885253191,
    "r_median": 1.9682770962890677,
    "r_p95": 9.693609525859648,
    "r_max": 10.27089463170814
  },
  {
    "mode": "DCREG_ONLY",
    "segment": "150-200",
    "count": 496,
    "local_predictor_t_rmse_m": 0.2594861873646641,
    "local_predictor_r_rmse_deg": 0.8776311612836537,
    "t_mean": 22.70416825119063,
    "t_rmse": 32.12260620581742,
    "t_median": 11.35991975195023,
    "t_p95": 74.62850834535189,
    "t_max": 79.55365973735074,
    "r_mean": 13.253187106535693,
    "r_rmse": 13.40462600126808,
    "r_median": 12.751885617516466,
    "r_p95": 15.494863330248192,
    "r_max": 20.945811868256374
  },
  {
    "mode": "DCREG_ONLY",
    "segment": "200-250",
    "count": 495,
    "local_predictor_t_rmse_m": 0.389900353369037,
    "local_predictor_r_rmse_deg": 0.8027565736978932,
    "t_mean": 85.60156482005367,
    "t_rmse": 86.33523798435351,
    "t_median": 82.99050899244061,
    "t_p95": 107.59401273901702,
    "t_max": 108.4341990762926,
    "r_mean": 15.234896598435597,
    "r_rmse": 15.366371137767592,
    "r_median": 14.421815230072033,
    "r_p95": 18.269926155040945,
    "r_max": 21.697038417398723
  },
  {
    "mode": "DCREG_ONLY",
    "segment": "250-300",
    "count": 496,
    "local_predictor_t_rmse_m": 0.2825648509498769,
    "local_predictor_r_rmse_deg": 0.6699648201722814,
    "t_mean": 78.32733869240194,
    "t_rmse": 79.14140799064262,
    "t_median": 77.48297956231058,
    "t_p95": 102.80441627040078,
    "t_max": 107.71517196037746,
    "r_mean": 19.39601826222336,
    "r_rmse": 19.613102555375647,
    "r_median": 18.538658808069187,
    "r_p95": 25.358960121286884,
    "r_max": 26.888856325065827
  },
  {
    "mode": "DCREG_ONLY",
    "segment": "300-350",
    "count": 496,
    "local_predictor_t_rmse_m": 0.47395603386848056,
    "local_predictor_r_rmse_deg": 0.9469986378360676,
    "t_mean": 105.20139059009178,
    "t_rmse": 106.33211903617594,
    "t_median": 105.0044404439463,
    "t_p95": 127.57359303893934,
    "t_max": 128.15400872681317,
    "r_mean": 17.166187166820063,
    "r_rmse": 19.389144895394384,
    "r_median": 17.849920863187805,
    "r_p95": 28.524305157662326,
    "r_max": 29.451091331032064
  },
  {
    "mode": "DCREG_ONLY",
    "segment": "350-end",
    "count": 661,
    "local_predictor_t_rmse_m": 0.24945075214300325,
    "local_predictor_r_rmse_deg": 0.588997124436162,
    "t_mean": 95.49587045108474,
    "t_rmse": 97.40271406384545,
    "t_median": 111.8242183175884,
    "t_p95": 113.27048715744904,
    "t_max": 113.67318402762005,
    "r_mean": 4.466887397529452,
    "r_rmse": 5.272970836938001,
    "r_median": 3.691194388318197,
    "r_p95": 9.468004865539479,
    "r_max": 10.466607736420723
  },
  {
    "mode": "MULTISTART_OBJECTIVE",
    "segment": "all",
    "count": 4126,
    "local_predictor_t_rmse_m": 0.023915378474088873,
    "local_predictor_r_rmse_deg": 0.7909097182013073,
    "t_mean": 0.7352673629029501,
    "t_rmse": 0.8915389778980062,
    "t_median": 0.5770612868741174,
    "t_p95": 1.583456986094081,
    "t_max": 1.8067101061629338,
    "r_mean": 2.2191112233919847,
    "r_rmse": 2.7133932078801477,
    "r_median": 1.7915032330855112,
    "r_p95": 5.590391301240942,
    "r_max": 12.469733679113734
  },
  {
    "mode": "MULTISTART_OBJECTIVE",
    "segment": "150-end",
    "count": 2644,
    "local_predictor_t_rmse_m": 0.021874942689101454,
    "local_predictor_r_rmse_deg": 0.7772967810524112,
    "t_mean": 0.7328163532115683,
    "t_rmse": 0.8614650314789907,
    "t_median": 0.5987323798613755,
    "t_p95": 1.4807211722246785,
    "t_max": 1.7423286219050182,
    "r_mean": 2.4329606465767415,
    "r_rmse": 2.920277826537706,
    "r_median": 1.9037177219742971,
    "r_p95": 5.876702921413054,
    "r_max": 9.324683502560164
  },
  {
    "mode": "MULTISTART_OBJECTIVE",
    "segment": "0-50",
    "count": 491,
    "local_predictor_t_rmse_m": 0.01697482679817883,
    "local_predictor_r_rmse_deg": 0.5899375939571724,
    "t_mean": 0.1905650417831844,
    "t_rmse": 0.22515203334441602,
    "t_median": 0.1913015800087402,
    "t_p95": 0.4016713049655223,
    "t_max": 0.43802381053523526,
    "r_mean": 1.4102019717704897,
    "r_rmse": 1.934863752287039,
    "r_median": 0.9594156094521509,
    "r_p95": 4.034140767514731,
    "r_max": 5.253001971354918
  },
  {
    "mode": "MULTISTART_OBJECTIVE",
    "segment": "50-100",
    "count": 496,
    "local_predictor_t_rmse_m": 0.025294423540867278,
    "local_predictor_r_rmse_deg": 0.9103731914587258,
    "t_mean": 0.5495744029410604,
    "t_rmse": 0.6306691567453925,
    "t_median": 0.4635183740757266,
    "t_p95": 1.176852874222742,
    "t_max": 1.2007829286587763,
    "r_mean": 1.8297453839633755,
    "r_rmse": 2.2129695858911846,
    "r_median": 1.578384374856718,
    "r_p95": 3.8345040665197034,
    "r_max": 12.469733679113734
  },
  {
    "mode": "MULTISTART_OBJECTIVE",
    "segment": "100-150",
    "count": 495,
    "local_predictor_t_rmse_m": 0.03583210479813348,
    "local_predictor_r_rmse_deg": 0.9003460740026916,
    "t_mean": 1.4747280041856072,
    "t_rmse": 1.487439119340975,
    "t_median": 1.4696177654919018,
    "t_p95": 1.7798159045786524,
    "t_max": 1.8067101061629338,
    "r_mean": 2.26937793854804,
    "r_rmse": 2.6826831823801003,
    "r_median": 1.7539332794359037,
    "r_p95": 5.751029493422632,
    "r_max": 7.778045645062131
  },
  {
    "mode": "MULTISTART_OBJECTIVE",
    "segment": "150-200",
    "count": 496,
    "local_predictor_t_rmse_m": 0.027721369316736882,
    "local_predictor_r_rmse_deg": 0.8775906074706002,
    "t_mean": 1.4125834711577858,
    "t_rmse": 1.4184172812657152,
    "t_median": 1.4279563076180541,
    "t_p95": 1.653326972455406,
    "t_max": 1.7423286219050182,
    "r_mean": 1.4101814027439554,
    "r_rmse": 1.5625600563871362,
    "r_median": 1.2925875013450274,
    "r_p95": 2.47696004118733,
    "r_max": 4.812141242459784
  },
  {
    "mode": "MULTISTART_OBJECTIVE",
    "segment": "200-250",
    "count": 495,
    "local_predictor_t_rmse_m": 0.01861558353477766,
    "local_predictor_r_rmse_deg": 0.8027628682437614,
    "t_mean": 1.0679954249550212,
    "t_rmse": 1.0773866998394057,
    "t_median": 1.1104364985754787,
    "t_p95": 1.2281679516399404,
    "t_max": 1.338429646208563,
    "r_mean": 2.244517438158556,
    "r_rmse": 2.6173721823512457,
    "r_median": 2.0400478571610665,
    "r_p95": 4.650782536482895,
    "r_max": 9.324683502560164
  },
  {
    "mode": "MULTISTART_OBJECTIVE",
    "segment": "250-300",
    "count": 496,
    "local_predictor_t_rmse_m": 0.019071189587216538,
    "local_predictor_r_rmse_deg": 0.6700941798163678,
    "t_mean": 0.6621316676133296,
    "t_rmse": 0.690009140454581,
    "t_median": 0.6045101254539209,
    "t_p95": 1.0299497181121458,
    "t_max": 1.1508599662391106,
    "r_mean": 1.9381436158053649,
    "r_rmse": 2.064400675904723,
    "r_median": 1.7998491759076867,
    "r_p95": 3.2629742030519604,
    "r_max": 6.317762651043255
  },
  {
    "mode": "MULTISTART_OBJECTIVE",
    "segment": "300-350",
    "count": 496,
    "local_predictor_t_rmse_m": 0.02618476483418467,
    "local_predictor_r_rmse_deg": 0.9467465841907525,
    "t_mean": 0.40402039048690735,
    "t_rmse": 0.4387436309633415,
    "t_median": 0.3957190612615186,
    "t_p95": 0.7480248214645979,
    "t_max": 0.8377051710201231,
    "r_mean": 4.644565248600464,
    "r_rmse": 4.924737375851721,
    "r_median": 4.625002328817409,
    "r_p95": 7.495028822778092,
    "r_max": 8.646555651541252
  },
  {
    "mode": "MULTISTART_OBJECTIVE",
    "segment": "350-end",
    "count": 661,
    "local_predictor_t_rmse_m": 0.017043570772959375,
    "local_predictor_r_rmse_deg": 0.5888383631568735,
    "t_mean": 0.27149149777106213,
    "t_rmse": 0.29636554943318405,
    "t_median": 0.26680752399219154,
    "t_p95": 0.5083416050541543,
    "t_max": 0.6094390979861349,
    "r_mean": 2.053310507041039,
    "r_rmse": 2.398518423864156,
    "r_median": 1.6935485042581282,
    "r_p95": 4.386024447792914,
    "r_max": 6.394159569377165
  },
  {
    "mode": "MULTISTART_VISUAL",
    "segment": "all",
    "count": 4126,
    "local_predictor_t_rmse_m": 0.034742081188034096,
    "local_predictor_r_rmse_deg": 0.7909107178953423,
    "t_mean": 1.7152977270107554,
    "t_rmse": 2.1475751118502995,
    "t_median": 1.66580795394482,
    "t_p95": 3.7649814096724374,
    "t_max": 4.669088876858145,
    "r_mean": 3.698086888004055,
    "r_rmse": 4.794765522602672,
    "r_median": 2.4409495719441177,
    "r_p95": 9.613171074729097,
    "r_max": 12.750769660862312
  },
  {
    "mode": "MULTISTART_VISUAL",
    "segment": "150-end",
    "count": 2644,
    "local_predictor_t_rmse_m": 0.03821743579366436,
    "local_predictor_r_rmse_deg": 0.7772946256330394,
    "t_mean": 2.245733961262868,
    "t_rmse": 2.5805845272507635,
    "t_median": 2.4601737887305584,
    "t_p95": 4.062241948103097,
    "t_max": 4.669088876858145,
    "r_mean": 4.562077646807981,
    "r_rmse": 5.592164154039125,
    "r_median": 3.410999780141661,
    "r_p95": 10.262645152849705,
    "r_max": 12.750769660862312
  },
  {
    "mode": "MULTISTART_VISUAL",
    "segment": "0-50",
    "count": 491,
    "local_predictor_t_rmse_m": 0.016621881568910674,
    "local_predictor_r_rmse_deg": 0.589940009970438,
    "t_mean": 0.21950473189579653,
    "t_rmse": 0.24062411890724153,
    "t_median": 0.19695713854772656,
    "t_p95": 0.4042379113255372,
    "t_max": 0.442330018616067,
    "r_mean": 1.5501048424107917,
    "r_rmse": 1.976311293922416,
    "r_median": 1.1334583994289553,
    "r_p95": 3.881391009414134,
    "r_max": 5.268717487070718
  },
  {
    "mode": "MULTISTART_VISUAL",
    "segment": "50-100",
    "count": 496,
    "local_predictor_t_rmse_m": 0.02522071831092731,
    "local_predictor_r_rmse_deg": 0.9103864877684755,
    "t_mean": 0.5456354873269782,
    "t_rmse": 0.6288156403477002,
    "t_median": 0.46269708748379296,
    "t_p95": 1.1796122471403943,
    "t_max": 1.2049410594580752,
    "r_mean": 1.8340650493344632,
    "r_rmse": 2.1997384414981664,
    "r_median": 1.518864960568814,
    "r_p95": 3.8402468423112595,
    "r_max": 12.32307277589384
  },
  {
    "mode": "MULTISTART_VISUAL",
    "segment": "100-150",
    "count": 495,
    "local_predictor_t_rmse_m": 0.03668851534293256,
    "local_predictor_r_rmse_deg": 0.900348292978208,
    "t_mean": 1.5377490969542154,
    "t_rmse": 1.5553026576529092,
    "t_median": 1.5334046870383407,
    "t_p95": 1.8815717526760558,
    "t_max": 2.173283974108686,
    "r_mean": 3.0815665851532077,
    "r_rmse": 3.9833076897705353,
    "r_median": 2.021848797832522,
    "r_p95": 8.86634454807349,
    "r_max": 10.071886314189143
  },
  {
    "mode": "MULTISTART_VISUAL",
    "segment": "150-200",
    "count": 496,
    "local_predictor_t_rmse_m": 0.045623848752592955,
    "local_predictor_r_rmse_deg": 0.8776014744858018,
    "t_mean": 2.9788600188031706,
    "t_rmse": 3.061041259073383,
    "t_median": 2.628123310974023,
    "t_p95": 4.156085599964861,
    "t_max": 4.2824122257998045,
    "r_mean": 8.780892461859484,
    "r_rmse": 8.864042407511034,
    "r_median": 8.6250226416212,
    "r_p95": 10.7847136034794,
    "r_max": 12.091274948247857
  },
  {
    "mode": "MULTISTART_VISUAL",
    "segment": "200-250",
    "count": 495,
    "local_predictor_t_rmse_m": 0.04547251473573389,
    "local_predictor_r_rmse_deg": 0.8027496229805786,
    "t_mean": 3.4721366513326553,
    "t_rmse": 3.516189444251884,
    "t_median": 3.372913500640937,
    "t_p95": 4.5664674965881344,
    "t_max": 4.669088876858145,
    "r_mean": 5.033521052259173,
    "r_rmse": 5.824880581720615,
    "r_median": 3.867708872073016,
    "r_p95": 10.085444136185089,
    "r_max": 12.619730844213732
  },
  {
    "mode": "MULTISTART_VISUAL",
    "segment": "250-300",
    "count": 496,
    "local_predictor_t_rmse_m": 0.027315756405474224,
    "local_predictor_r_rmse_deg": 0.6700447223889409,
    "t_mean": 2.2245166891129022,
    "t_rmse": 2.2728543397331276,
    "t_median": 2.2524157921814476,
    "t_p95": 2.927322398723251,
    "t_max": 3.0031286243916577,
    "r_mean": 2.088499905722384,
    "r_rmse": 2.4232872051472323,
    "r_median": 1.9054434183775872,
    "r_p95": 4.114619169827718,
    "r_max": 8.844277628594579
  },
  {
    "mode": "MULTISTART_VISUAL",
    "segment": "300-350",
    "count": 496,
    "local_predictor_t_rmse_m": 0.045922219799036754,
    "local_predictor_r_rmse_deg": 0.946767478681995,
    "t_mean": 2.810960665013683,
    "t_rmse": 2.876056108700808,
    "t_median": 3.0427251055551077,
    "t_p95": 3.6163645104456297,
    "t_max": 3.728833041646953,
    "r_mean": 5.683309774827695,
    "r_rmse": 6.3351709899046345,
    "r_median": 5.413018965755763,
    "r_p95": 10.524586502062654,
    "r_max": 12.750769660862312
  },
  {
    "mode": "MULTISTART_VISUAL",
    "segment": "350-end",
    "count": 661,
    "local_predictor_t_rmse_m": 0.024280737035364867,
    "local_predictor_r_rmse_deg": 0.5888453737278107,
    "t_mean": 0.3689888263180026,
    "t_rmse": 0.5145119979434659,
    "t_median": 0.27961060516013275,
    "t_p95": 0.8215791268557233,
    "t_max": 2.158530915766023,
    "r_mean": 2.058093970736561,
    "r_rmse": 2.4900169942305888,
    "r_median": 1.4998567720187788,
    "r_p95": 4.7939667645488475,
    "r_max": 7.163659683980591
  },
  {
    "mode": "FULL_ROUTER",
    "segment": "all",
    "count": 4126,
    "local_predictor_t_rmse_m": 0.13811882777356632,
    "local_predictor_r_rmse_deg": 0.7909495324368834,
    "t_mean": 19.021215984671297,
    "t_rmse": 27.625885753048234,
    "t_median": 6.667488314971325,
    "t_p95": 53.08571027608878,
    "t_max": 54.39509832973801,
    "r_mean": 7.578498927639119,
    "r_rmse": 9.901137892022303,
    "r_median": 6.457643743781823,
    "r_p95": 20.029293229316732,
    "r_max": 23.620317635038703
  },
  {
    "mode": "FULL_ROUTER",
    "segment": "150-end",
    "count": 2644,
    "local_predictor_t_rmse_m": 0.1713104599911319,
    "local_predictor_r_rmse_deg": 0.777362124620098,
    "t_mean": 29.266570794452804,
    "t_rmse": 34.50299016076601,
    "t_median": 33.770766146634216,
    "t_p95": 53.34036907319293,
    "t_max": 54.39509832973801,
    "r_mean": 10.658368422737462,
    "r_rmse": 12.191355768848915,
    "r_median": 10.341590475940658,
    "r_p95": 20.65264158766675,
    "r_max": 23.620317635038703
  },
  {
    "mode": "FULL_ROUTER",
    "segment": "0-50",
    "count": 491,
    "local_predictor_t_rmse_m": 0.016090648700345916,
    "local_predictor_r_rmse_deg": 0.5899164988199449,
    "t_mean": 0.18989559089594787,
    "t_rmse": 0.22881237826915074,
    "t_median": 0.1797806602315778,
    "t_p95": 0.4101625867781135,
    "t_max": 0.4453885668523171,
    "r_mean": 1.4157254034914646,
    "r_rmse": 1.9171895330958075,
    "r_median": 1.0333429466063169,
    "r_p95": 3.914520820203612,
    "r_max": 5.312490033542709
  },
  {
    "mode": "FULL_ROUTER",
    "segment": "50-100",
    "count": 496,
    "local_predictor_t_rmse_m": 0.023816169048202187,
    "local_predictor_r_rmse_deg": 0.9103753244212422,
    "t_mean": 0.5307748368782838,
    "t_rmse": 0.60885265947899,
    "t_median": 0.4350466892375511,
    "t_p95": 1.1620834356364949,
    "t_max": 1.200327421557083,
    "r_mean": 1.8340321600859824,
    "r_rmse": 2.1970790419622968,
    "r_median": 1.543411661797578,
    "r_p95": 3.744655584403748,
    "r_max": 12.393851513874655
  },
  {
    "mode": "FULL_ROUTER",
    "segment": "100-150",
    "count": 495,
    "local_predictor_t_rmse_m": 0.03732062781141403,
    "local_predictor_r_rmse_deg": 0.9003477211249726,
    "t_mean": 1.5034766020182502,
    "t_rmse": 1.5202529297808092,
    "t_median": 1.4655861157660854,
    "t_p95": 1.7979686716337935,
    "t_max": 2.1865740428862273,
    "r_mean": 2.99668553778626,
    "r_rmse": 3.8422797480441306,
    "r_median": 1.936996594604806,
    "r_p95": 8.034986204861335,
    "r_max": 10.076497512299316
  },
  {
    "mode": "FULL_ROUTER",
    "segment": "150-200",
    "count": 496,
    "local_predictor_t_rmse_m": 0.04677420829564303,
    "local_predictor_r_rmse_deg": 0.8776125109553129,
    "t_mean": 4.878159581932093,
    "t_rmse": 4.982177657411693,
    "t_median": 5.090374686785175,
    "t_p95": 6.089466249753075,
    "t_max": 6.15338485623728,
    "r_mean": 9.328032718664515,
    "r_rmse": 9.449492469512238,
    "r_median": 9.376156540771856,
    "r_p95": 11.464652820942248,
    "r_max": 15.751978787842786
  },
  {
    "mode": "FULL_ROUTER",
    "segment": "200-250",
    "count": 495,
    "local_predictor_t_rmse_m": 0.186645904851548,
    "local_predictor_r_rmse_deg": 0.8027943865419495,
    "t_mean": 11.453935258246068,
    "t_rmse": 13.816967589287135,
    "t_median": 7.658541694717553,
    "t_p95": 29.943099812103696,
    "t_max": 35.17264167907499,
    "r_mean": 12.76447648409063,
    "r_rmse": 12.915938953225563,
    "r_median": 12.538001708796614,
    "r_p95": 15.541310548949943,
    "r_max": 17.05967856645218
  },
  {
    "mode": "FULL_ROUTER",
    "segment": "250-300",
    "count": 496,
    "local_predictor_t_rmse_m": 0.15525614102507956,
    "local_predictor_r_rmse_deg": 0.6702554978921138,
    "t_mean": 46.15478076652035,
    "t_rmse": 46.62370829712767,
    "t_median": 50.05778303572157,
    "t_p95": 52.83675340736062,
    "t_max": 54.028418157619164,
    "r_mean": 18.216614582479725,
    "r_rmse": 18.402145755063824,
    "r_median": 18.27438259113356,
    "r_p95": 22.014121929291452,
    "r_max": 22.921459751678604
  },
  {
    "mode": "FULL_ROUTER",
    "segment": "300-350",
    "count": 496,
    "local_predictor_t_rmse_m": 0.19506814820227564,
    "local_predictor_r_rmse_deg": 0.9467608110444339,
    "t_mean": 49.50340125136057,
    "t_rmse": 49.90753320775504,
    "t_median": 52.56124279805232,
    "t_p95": 54.26720988741739,
    "t_max": 54.39509832973801,
    "r_mean": 11.253839474220273,
    "r_rmse": 12.931057660011161,
    "r_median": 12.322054227011968,
    "r_p95": 22.385754582309715,
    "r_max": 23.620317635038703
  },
  {
    "mode": "FULL_ROUTER",
    "segment": "350-end",
    "count": 661,
    "local_predictor_t_rmse_m": 0.20740987721549056,
    "local_predictor_r_rmse_deg": 0.5889717808078612,
    "t_mean": 33.0485473437128,
    "t_rmse": 33.16732277230654,
    "t_median": 33.85976385517141,
    "t_p95": 36.87427562696632,
    "t_max": 37.12809598857871,
    "r_mean": 3.961060226190907,
    "r_rmse": 4.796520300029939,
    "r_median": 2.1697552154890847,
    "r_p95": 8.939714798772819,
    "r_max": 9.886962889230922
  }
]
```

## Branch counts

```json
[
  {
    "mode": "DCREG_ONLY",
    "total_scans": 4127,
    "baseline_branch_count": 3680,
    "dcreg_branch_count": 447,
    "multistart_attempted": 0,
    "multistart_multicluster": 0,
    "visual_arbitration_used": 0,
    "visual_changed_baseline_choice": 0,
    "objective_only_changed_baseline": "",
    "selected_nonbaseline_seed_count": 0
  },
  {
    "mode": "MULTISTART_OBJECTIVE",
    "total_scans": 4127,
    "baseline_branch_count": 2325,
    "dcreg_branch_count": 0,
    "multistart_attempted": 1802,
    "multistart_multicluster": 623,
    "visual_arbitration_used": 0,
    "visual_changed_baseline_choice": 0,
    "objective_only_changed_baseline": 1715,
    "selected_nonbaseline_seed_count": 1715
  },
  {
    "mode": "MULTISTART_VISUAL",
    "total_scans": 4127,
    "baseline_branch_count": 2839,
    "dcreg_branch_count": 0,
    "multistart_attempted": 1802,
    "multistart_multicluster": 1288,
    "visual_arbitration_used": 1288,
    "visual_changed_baseline_choice": 911,
    "objective_only_changed_baseline": "",
    "selected_nonbaseline_seed_count": 911
  },
  {
    "mode": "FULL_ROUTER",
    "total_scans": 4127,
    "baseline_branch_count": 2532,
    "dcreg_branch_count": 455,
    "multistart_attempted": 1607,
    "multistart_multicluster": 1140,
    "visual_arbitration_used": 1140,
    "visual_changed_baseline_choice": 706,
    "objective_only_changed_baseline": "",
    "selected_nonbaseline_seed_count": 706
  }
]
```

## DCReg and multi-start post-hoc

DCReg-only branch scans: 447; full-router DCReg branches: 455; among 447 converged degenerate DCReg-only frames, DCReg was closer to the anchored GT in 254 and farther in 193. The route activates only for converged registrations reported degenerate by the native DCReg API.
Multi-start objective attempts/multi-cluster: 1802/623; visual mode: 1802/1288; full router: 1607/1140.
Post-hoc corrected-frame translation comparisons (strictly lower error): objective-only vs baseline: 3607/4126; visual vs objective-only: 852/4126.

## Recovery events

```json
[]
```

## Compute (ms per scan)

```json
[
  {
    "mode": "BASELINE",
    "scan_count": 4127,
    "baseline_ndt_mean_ms": 7.068156587593894,
    "baseline_ndt_p95_ms": 19.710707,
    "baseline_ndt_max_ms": 115.094037,
    "dcreg_mean_ms": 0.0,
    "dcreg_p95_ms": 0.0,
    "dcreg_max_ms": 0.0,
    "multistart_mean_ms": 0.0,
    "multistart_p95_ms": 0.0,
    "multistart_max_ms": 0.0,
    "visual_arbitration_mean_ms": 0.0,
    "visual_arbitration_p95_ms": 0.0,
    "visual_arbitration_max_ms": 0.0,
    "ikfom_update_mean_ms": NaN,
    "ikfom_update_p95_ms": NaN,
    "ikfom_update_max_ms": NaN,
    "total_mean_ms": 8.908674332929488,
    "total_p95_ms": 22.049486999999996,
    "total_max_ms": 117.52628,
    "router_overhead_mean_ms": 0.0,
    "router_overhead_p95_ms": 0.0,
    "router_overhead_max_ms": 0.0
  },
  {
    "mode": "DCREG_ONLY",
    "scan_count": 4127,
    "baseline_ndt_mean_ms": 7.237204763508602,
    "baseline_ndt_p95_ms": 18.452709999999996,
    "baseline_ndt_max_ms": 102.281,
    "dcreg_mean_ms": 108.0029521686455,
    "dcreg_p95_ms": 127.85,
    "dcreg_max_ms": 396.32,
    "multistart_mean_ms": 0.0,
    "multistart_p95_ms": 0.0,
    "multistart_max_ms": 0.0,
    "visual_arbitration_mean_ms": 0.0,
    "visual_arbitration_p95_ms": 0.0,
    "visual_arbitration_max_ms": 0.0,
    "ikfom_update_mean_ms": 0.04422437460625151,
    "ikfom_update_p95_ms": 0.051382299999999985,
    "ikfom_update_max_ms": 0.147715,
    "total_mean_ms": 120.85382306760359,
    "total_p95_ms": 146.90189999999998,
    "total_max_ms": 409.014,
    "router_overhead_mean_ms": 5.310425132057184,
    "router_overhead_p95_ms": 7.330626199999992,
    "router_overhead_max_ms": 41.12121000000002
  },
  {
    "mode": "MULTISTART_OBJECTIVE",
    "scan_count": 4127,
    "baseline_ndt_mean_ms": 4.909429531136419,
    "baseline_ndt_p95_ms": 8.882887999999998,
    "baseline_ndt_max_ms": 101.19,
    "dcreg_mean_ms": 0.0,
    "dcreg_p95_ms": 0.0,
    "dcreg_max_ms": 0.0,
    "multistart_mean_ms": 53.82527421856069,
    "multistart_p95_ms": 152.1425,
    "multistart_max_ms": 194.624,
    "visual_arbitration_mean_ms": 0.0,
    "visual_arbitration_p95_ms": 0.0,
    "visual_arbitration_max_ms": 0.0,
    "ikfom_update_mean_ms": 0.02999344075599709,
    "ikfom_update_p95_ms": 0.049755299999999975,
    "ikfom_update_max_ms": 0.095473,
    "total_mean_ms": 74.89834349164042,
    "total_p95_ms": 193.7609,
    "total_max_ms": 237.532,
    "router_overhead_mean_ms": 15.934003896050399,
    "router_overhead_p95_ms": 34.917342599999984,
    "router_overhead_max_ms": 68.000822
  },
  {
    "mode": "MULTISTART_VISUAL",
    "scan_count": 4127,
    "baseline_ndt_mean_ms": 5.477816549309425,
    "baseline_ndt_p95_ms": 11.78062,
    "baseline_ndt_max_ms": 105.171,
    "dcreg_mean_ms": 0.0,
    "dcreg_p95_ms": 0.0,
    "dcreg_max_ms": 0.0,
    "multistart_mean_ms": 45.51365846862128,
    "multistart_p95_ms": 146.14319999999998,
    "multistart_max_ms": 242.131,
    "visual_arbitration_mean_ms": 0.0007507048703658831,
    "visual_arbitration_p95_ms": 0.002863,
    "visual_arbitration_max_ms": 0.008172,
    "ikfom_update_mean_ms": 0.03085287666585898,
    "ikfom_update_p95_ms": 0.048959,
    "ikfom_update_max_ms": 0.110071,
    "total_mean_ms": 67.71401026653743,
    "total_p95_ms": 187.665,
    "total_max_ms": 282.123,
    "router_overhead_mean_ms": 16.488608266779742,
    "router_overhead_p95_ms": 39.058115199999996,
    "router_overhead_max_ms": 65.68860499999997
  },
  {
    "mode": "FULL_ROUTER",
    "scan_count": 4127,
    "baseline_ndt_mean_ms": 7.416112252241338,
    "baseline_ndt_p95_ms": 19.70477999999999,
    "baseline_ndt_max_ms": 105.204,
    "dcreg_mean_ms": 110.74265255633631,
    "dcreg_p95_ms": 137.3018,
    "dcreg_max_ms": 468.908,
    "multistart_mean_ms": 35.22026008965349,
    "multistart_p95_ms": 146.8474,
    "multistart_max_ms": 247.855,
    "visual_arbitration_mean_ms": 0.0007251398110007268,
    "visual_arbitration_p95_ms": 0.002864,
    "visual_arbitration_max_ms": 0.009708,
    "ikfom_update_mean_ms": 0.04211516937242549,
    "ikfom_update_p95_ms": 0.05592229999999999,
    "ikfom_update_max_ms": 0.103366,
    "total_mean_ms": 169.98389772231647,
    "total_p95_ms": 300.15409999999997,
    "total_max_ms": 550.599,
    "router_overhead_mean_ms": 16.319838911558037,
    "router_overhead_p95_ms": 41.287097999999986,
    "router_overhead_max_ms": 77.59937000000002
  }
]
```
P4-I3's visual frontend cost is about 23.6 ms as measured by its offline Python/OpenCV path; this is not a C++ runtime WCET. The runtime table measures the offline replay stages and records router overhead as the total replay step minus instrumented components.

## Reliability candidate data (descriptive only)

```json
{
  "U_intra": {
    "degenerate_frame_count": 447,
    "dominant_weak_axis_mask_counts": {
      "rx": 404,
      "ry": 33,
      "rz": 0,
      "tx": 0,
      "ty": 0,
      "tz": 43
    },
    "rotation_schur_eigenvalue_stats": {
      "mean": 20741.284934312214,
      "rmse": 45016.26560747512,
      "median": 11127.4,
      "p95": 48564.7,
      "max": 350452.0
    },
    "translation_schur_eigenvalue_stats": {
      "mean": 259.0884093199382,
      "rmse": 299.9618053970058,
      "median": 219.635,
      "p95": 524.951,
      "max": 661.331
    }
  },
  "U_inter": {
    "U_inter_multicluster_mode_events": 5097,
    "U_inter_multicluster_mode_events_by_mode": {
      "MULTISTART_OBJECTIVE": 1721,
      "MULTISTART_VISUAL": 1786,
      "FULL_ROUTER": 1590
    },
    "U_inter_raw_cluster_count": {
      "mean": 4.910143221502845,
      "rmse": 5.215824610591506,
      "median": 5.0,
      "p95": 7.0,
      "max": 8.0
    },
    "U_inter_max_translation_separation_m": {
      "mean": 0.8370460390896606,
      "rmse": 1.0470809880416572,
      "median": 0.817506,
      "p95": 2.090322,
      "max": 3.63762
    },
    "U_inter_max_rotation_separation_deg": {
      "mean": 6.786599210712183,
      "rmse": 9.562064547477409,
      "median": 5.04222,
      "p95": 14.994379999999996,
      "max": 139.779
    },
    "U_inter_objective_gap": {
      "mean": 262.2343910920345,
      "rmse": 431.10578147679547,
      "median": 36.9061,
      "p95": 910.2823999999998,
      "max": 1303.65
    },
    "U_inter_visual_residual_spread_m": {
      "mean": 0.5338000531721075,
      "rmse": 0.6891504763201691,
      "median": 0.481924,
      "p95": 1.3351899999999999,
      "max": 2.83288
    }
  },
  "visual_mode_residual_stats": {
    "MULTISTART_VISUAL": {
      "gated_cluster_representative_count": 4505,
      "all_gated_cluster_residual_m": {
        "mean": 0.35108779724306327,
        "rmse": 0.5411401687245542,
        "median": 0.14537,
        "p95": 1.2158700000000002,
        "max": 2.84971
      },
      "selected_cluster_residual_m": {
        "mean": 0.07331442296851574,
        "rmse": 0.10510021589660702,
        "median": 0.0521458,
        "p95": 0.21047099999999996,
        "max": 0.733163
      },
      "baseline_cluster_residual_m": {
        "mean": 0.08141593546614873,
        "rmse": 0.11107128895361618,
        "median": 0.06030165,
        "p95": 0.21419594999999997,
        "max": 0.733163
      }
    },
    "FULL_ROUTER": {
      "gated_cluster_representative_count": 4601,
      "all_gated_cluster_residual_m": {
        "mean": 0.4426395931993045,
        "rmse": 0.5939380397678529,
        "median": 0.354062,
        "p95": 1.10565,
        "max": 3.22435
      },
      "selected_cluster_residual_m": {
        "mean": 0.133884338394458,
        "rmse": 0.2380674098709467,
        "median": 0.0675424,
        "p95": 0.4918255,
        "max": 2.40048
      },
      "baseline_cluster_residual_m": {
        "mean": 0.13607935967641568,
        "rmse": 0.2414921953873427,
        "median": 0.070932,
        "p95": 0.5084966999999958,
        "max": 2.40048
      }
    }
  }
}
```
No novelty claim. DCReg, PCL NDT multi-start, and KLT/PnP are mature modules. The allowed novelty candidate remains an unverified reliability decomposition that conditionally invokes established recovery mechanisms.

## Provenance and limitations

- Frozen workspace baseline HEAD: `cf8d7be2cac53120e68fb74b31033f5174f232d8`; report generated from the P6-I1 worktree.
- Frozen map SHA: `2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570` (`EXACT`); runtime-topic bag SHA: `860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db`; config SHA: `4e9584a4c1d5c2ada963700892880cdf2a7f4e75e43f0ff258b5fd4272af7d77`; official calibration SHA: `fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414`; official GT SHA (post-hoc only): `b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f`.
- DCReg commit: `ce7db8220f549a4a4391729e3bf4de4d4ab74635`; FAST-LIO2/IKFoM source commit: `7cc4175de6f8ba2edf34bab02a42195b141027e9`.
- Offline closed-loop replay; each mode independently carries its corrected IKFoM state to the next scan.
- Frozen scan-end deskew clouds are reused; this does not recompute upstream deskew or visual frontend output.
- Frozen P4 visual valid-pair translations are used only to form S7 and/or discriminate gated NDT mode clusters; they are never a continuous state measurement.
- GT is evaluation-only after all five trajectories are complete.
- No runtime ROS integration was performed.
- No parameter sweeps or added modes were run.

## Git

Prototype outputs are local/untracked at report time. Commit only P6 prototype code, analysis CSVs, plots and summary; never include raw bags/clouds, PCD maps or build artifacts. Push at most once after selective staging.
