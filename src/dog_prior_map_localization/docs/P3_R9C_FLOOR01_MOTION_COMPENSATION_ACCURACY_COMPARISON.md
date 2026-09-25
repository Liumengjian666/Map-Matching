# PAPER-P3-R9C — Floor01 motion-compensation pipeline accuracy comparison

## Scope and provenance

This compares two closed-loop motion-compensation infrastructures; it is not an isolated deskew-only ablation. No runtime algorithm, configuration, launch, map, or NDT source was changed, and no localization run was performed. GT was accessed only after the NDT-affecting parameter-equivalence gate passed and was used post-hoc only.

- Paper start commit: `2804be14cb78449d3dd8afefb915719082cb7e03`; source branch `paper`.
- Frozen baseline: `feature/visual-factor-window` at `41999ea700c66c4cadf0eca9e0c5d73caa2783fd`.
- Legacy formal R7H Run A result: `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r7_floor01_full_baseline/runA/ndt_determinism.csv` (SHA-256 `7b6aa71d161480fbd0f03fe395317688869dd40d3d36363f03a1194a8826e307`); effective config SHA-256 `a8047125cbf57224351d8ef4344fd46e87466eb198c6b8bd13c0edbbe1680dad`.
- Mature formal R9B full Run A result: `/media/jian/HIKVISION/paper rosbag/imu_deskew_experiments/p3_r9b/full_runA/ndt.csv` (SHA-256 `2fdb019dea039d45c7cbb5ed19ee41b9e8687538b9914e8229e635a0f35c562a`); profile SHA-256 `7e4cd51280e9c478ce9f9d4f33e8188407aef78dde67438d585fc0a91e2fbfeb`; algorithm commit `2804be14cb78449d3dd8afefb915719082cb7e03`.
- Legacy NDT output span: `1660857392.5927031`–`1660857809.8269341` (4136 rows); mature NDT output span: `1660857393.6012471`–`1660857809.8269341` (4126 rows).
- Shared raw bag: `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r7_floor01_full_baseline/floor01_canonical_raw_inputs.bag` (SHA-256 `6383fdbdad0e3f8375069b33cc552ed069dafb91aec88f1673bc043e0f314e0b`).
- Shared normalized H1 map SHA-256 `2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570`; Floor01 SP1 calibration SHA-256 `fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414`; NDT implementation SHA-256 `40cdf26f9dddb9d06dc16c0bfd89fc88a9dd33e42e5bbd4e9bc3b453f1e4277c`.
- Complete absolute paths and verified hashes: `p3_r9c_input_manifest.csv`; pre-registered windows and evaluator protocol: `P3_R9C_PROTOCOL.md`.

## Parameter-equivalence gate

Gate: **PASS**; unexpected differences: **0**. SAME parameters and the explicitly allowed closed-loop pipeline differences are itemized below. The old extrinsic is extracted from the hash-verified R7H adapter launch referenced by its Run A launch transcript and compared independently against the official calibration; the mature extrinsic is read from the manifest-hashed R9B profile, whose load path, calibration hash prefix, and runtime translation are checked in the formal launch record. The old NDT source SHA is parsed from R7H run provenance and compared to the hash-verified source in the R9B frozen-start workspace. The single first-common-frame initial-guess/prior-history mismatch is recorded as an expected startup-availability difference (mature PREINIT_REJECTED), not hidden or described as identical numeric initialization. From later common frames the logged source/reason and prior-use match.

| Parameter | Legacy | Mature | Status |
| --- | --- | --- | --- |
| NDT resolution (m) | 0.8 | 0.8 | SAME |
| NDT step size | 0.08 | 0.08 | SAME |
| transformation epsilon | 0.001 | 0.001 | SAME |
| maximum iterations | 40 | 40.0 | SAME |
| source voxel XY (m) | 0.25 | 0.25 | SAME |
| source voxel Z (m) | 0.25 | 0.25 | SAME |
| target voxel XY (m) | 0.15 | 0.15 | SAME |
| target voxel Z (m) | 0.15 | 0.15 | SAME |
| maximum source points | 1400 | 1400.0 | SAME |
| maximum target points | 0 | 0.0 | SAME |
| step limiter enabled | True | True | SAME |
| translation step limit (m) | 0.5 | 0.5 | SAME |
| rotation step limit (deg) | 5.0 | 5.0 | SAME |
| scan reference | start | start | SAME |
| point-offset scale | 1e-09 | 1e-09 | SAME |
| local IMU rotation prior | True | True | SAME |
| map SHA-256 | 2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570 | 2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570 | SAME |
| map frame | floor01_map_h1 | floor01_map_h1 | SAME |
| LiDAR/base frame | cmu_sp1_velodyne | cmu_sp1_velodyne | SAME |
| map voxel size (m) | 0.15 | 0.15 | SAME |
| map path | /tmp/floor01_candidates/floor01_h1_map.pcd | /tmp/floor01_candidates/floor01_h1_map.pcd | SAME |
| Floor01 calibration SHA-256 | fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414 | fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414 | SAME |
| LiDAR-to-IMU rotation (row-major 3x3) | [0.999945562,0.009074807,0.005149763,-0.009060897,0.999955255,-0.002718066,-0.005174199,0.002671256,0.999983046] | [0.999945562,0.009074807,0.005149763,-0.009060897,0.999955255,-0.002718066,-0.005174199,0.002671256,0.999983046] | SAME |
| LiDAR-to-IMU translation (m) | [0.08,0.029,0.03] | [0.08,0.029,0.03] | SAME |
| range filter (m) | [0.5, 80.0] shared-source defaults | [0.5, 80.0] shared-source defaults | SAME |
| packet decoder | ros-drivers/velodyne 1.7.0, commit 89faa698688a48d4a5080f73c04d7aed8117eec0 | ros-drivers/velodyne 1.7.0, commit 89faa698688a48d4a5080f73c04d7aed8117eec0 | SAME |
| point time semantics | official VLP16 firing-time table; first-packet/start reference | field=time; convention=seconds_from_scan_start; reference=start | SAME |
| loaded target point count | 549606 | 549606 | SAME |
| NDT implementation SHA-256 | 40cdf26f9dddb9d06dc16c0bfd89fc88a9dd33e42e5bbd4e9bc3b453f1e4277c | 40cdf26f9dddb9d06dc16c0bfd89fc88a9dd33e42e5bbd4e9bc3b453f1e4277c | SAME |
| initial-guess implementation | previous_pose_delta + local IMU rotation prior; shared code | previous_pose_delta + local IMU rotation prior; shared code | SAME |
| initial pose / map normalization | H1 map frame; initial current_pose is identity | H1 map frame; initial current_pose is identity | SAME |
| translation/rotation motion compensation | prior-NDT CV translation + IMU gyro rotation | EKF/IMU full-SE(3) deskew | EXPECTED_PIPELINE_DIFFERENCE |
| IMU state propagation and NDT feedback | prior NDT pose CV history | EKF propagation, NDT correction, later state-driven deskew | EXPECTED_PIPELINE_DIFFERENCE |
| gyro-bias static initialization | legacy adapter behavior | first 200 accepted IMU samples mean gyro | EXPECTED_PIPELINE_DIFFERENCE |
| deskew/NDT input topic plumbing | /superloc_adapter/points_deskewed | /dog_livo/points_deskewed_imu_exp | EXPECTED_PIPELINE_DIFFERENCE |
| first emitted initial guess identity | True | True | SAME |
| logged initial-guess source/reason | common=4126 | common=4126 | EXPECTED_PIPELINE_DIFFERENCE |
| logged local IMU prior enable/use | common=4126 | common=4126 | EXPECTED_PIPELINE_DIFFERENCE |

## Population, time alignment, and evaluator

- NDT outputs: legacy 4136, mature 4126; exact timestamp intersection 4126; legacy-only 10; mature-only 0.
- GT-supported full samples: legacy 4130 (early excluded 6, late excluded 0); mature 4126 (early excluded 0, late excluded 0). Paired common GT-supported samples 4126; HQ bracket `<=0.25 s` samples 4106.
- Common anchor `1660857393.6012471`; maximum absolute translation/rotation error among all eight raw/final anchor checks `1.42e-17`.
- Common paired GT bracket width mean/P95/max: 0.202589/0.201720/0.403440 s. No extrapolation.
- Reused unchanged P3-R3B conventions: translation linear interpolation, quaternion SLERP, Floor01 IMU-origin GT transformed with the official Floor01 calibration, and first-common-pair relative evaluation.

## Global metrics (translation m; rotation deg)

Final-used full and paired/HQ populations:

| Population | pipeline | n | t mean | t RMSE | t median | t P95 | t max | r mean | r RMSE | r median | r P95 | r max |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| FULL_SUPPORTED | legacy | 4130 | 31.9253030435 | 41.9642866363 | 32.3746172829 | 64.4726989112 | 68.1564586934 | 62.7727914712 | 80.5181707683 | 67.324591682 | 138.924649104 | 178.827388842 |
| FULL_SUPPORTED | mature_imu | 4126 | 42.8430766096 | 55.5987724985 | 56.8677088222 | 83.7174382986 | 85.6932648211 | 32.4418704637 | 43.276836168 | 31.3997806936 | 87.3520459076 | 111.884917645 |
| PAIRED_COMMON | legacy | 4126 | 31.9562085565 | 41.984623071 | 32.7153631148 | 64.4879344469 | 68.1564586934 | 62.8334636127 | 80.5571907189 | 67.3744896605 | 138.979379452 | 178.827388842 |
| PAIRED_COMMON | mature_imu | 4126 | 42.8430766096 | 55.5987724985 | 56.8677088222 | 83.7174382986 | 85.6932648211 | 32.4418704637 | 43.276836168 | 31.3997806936 | 87.3520459076 | 111.884917645 |
| PAIRED_COMMON | legacy_minus_mature | 4126 | -10.8868680531 | 14.9300635505 | -12.7030298716 | 0.770266107649 | 8.14312543274 | 30.391593149 | 50.9274763198 | 1.30332439249 | 98.6116921728 | 139.575795506 |
| HQ_COMMON | legacy | 4106 | 31.988345443 | 42.0009466704 | 33.0525585793 | 64.5070369292 | 68.1564586934 | 62.9423928364 | 80.6527366904 | 67.5677715005 | 139.077184604 | 178.827388842 |
| HQ_COMMON | mature_imu | 4106 | 42.9019011569 | 55.635752581 | 56.9008115703 | 83.7179638668 | 85.6932648211 | 32.5616921517 | 43.3717160136 | 31.8856457598 | 87.3653945039 | 111.884917645 |

Raw-NDT full and paired/HQ populations:

| Population | pipeline | n | t mean | t RMSE | t median | t P95 | t max | r mean | r RMSE | r median | r P95 | r max |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| FULL_SUPPORTED | legacy | 4130 | 31.9237173813 | 41.9618733857 | 32.3746172829 | 64.4241352486 | 68.1564586934 | 62.7513313074 | 80.5851988595 | 67.0637588486 | 138.66995995 | 178.399796208 |
| FULL_SUPPORTED | mature_imu | 4126 | 42.8439951076 | 55.5998877141 | 56.8677088222 | 83.7162032859 | 85.692856052 | 32.392017471 | 43.2794645021 | 31.4192059631 | 87.365394504 | 111.884917645 |
| PAIRED_COMMON | legacy | 4126 | 31.954621357 | 41.9822086509 | 32.7153631148 | 64.4270978883 | 68.1564586934 | 62.8119826442 | 80.624251293 | 67.1652890726 | 138.693804071 | 178.399796208 |
| PAIRED_COMMON | mature_imu | 4126 | 42.8439951076 | 55.5998877141 | 56.8677088222 | 83.7162032859 | 85.692856052 | 32.392017471 | 43.2794645021 | 31.419205963 | 87.3653945039 | 111.884917645 |
| PAIRED_COMMON | legacy_minus_mature | 4126 | -10.8893737506 | 14.9327215438 | -12.7016203623 | 0.770266107649 | 8.14312543274 | 30.4199651731 | 50.9463761003 | 1.31502660538 | 98.6177677541 | 139.575795506 |
| HQ_COMMON | legacy | 4106 | 31.9867507157 | 41.9985217385 | 33.0525585793 | 64.4598447045 | 68.1564586934 | 62.9203001232 | 80.719969979 | 67.3196281377 | 138.966746595 | 178.399796208 |
| HQ_COMMON | mature_imu | 4106 | 42.9028239037 | 55.6368735357 | 56.9008115703 | 83.7174382986 | 85.692856052 | 32.5114972877 | 43.3737641416 | 32.1539453776 | 87.3763880917 | 111.884917645 |

Paired bootstrap uses PCG64, seed `20260925`, `10000` resamples, percentile 95% CI. Delta is legacy minus mature; positive favors mature on reference error.

| pose | metric | n | Δ mean | mean 95% CI | Δ median | median 95% CI |
| --- | --- | --- | --- | --- | --- | --- |
| raw | translation | 4126 | -10.8893737506 | [-11.2086169479, -10.584041664] | -12.7016203623 | [-12.8501128656, -12.6244716135] |
| raw | rotation | 4126 | 30.4199651731 | [29.1584428624, 31.6592039018] | 1.31502660538 | [1.16087589429, 2.34383336693] |
| final_used | translation | 4126 | -10.8868680531 | [-11.197034733, -10.5715055842] | -12.7030298716 | [-12.8341852875, -12.6245998198] |
| final_used | rotation | 4126 | 30.391593149 | [29.1874675721, 31.6338184858] | 1.30332439249 | [1.15664394798, 2.34101679647] |

| pose | metric | n | mature better | legacy better | tie | tie tolerance |
| --- | --- | --- | --- | --- | --- | --- |
| raw | translation | 4126 | 1184 | 2941 | 1 | 1e-06 |
| raw | rotation | 4126 | 3175 | 950 | 1 | 1e-06 |
| final_used | translation | 4126 | 1181 | 2944 | 1 | 1e-06 |
| final_used | rotation | 4126 | 3186 | 939 | 1 | 1e-06 |

Raw-to-final NDT LiDAR-pose output change (map-frame translation displacement and relative rotation) is reported descriptively; it is not a causal estimate of limiter benefit:

| Pipeline | delta | mean | P95 | max |
| --- | --- | --- | --- | --- |
| legacy | m | 0.0282101651613 | 0.0249799532867 | 2.85680489809 |
| legacy | deg | 0.892425761244 | 4.34606937124 | 75.0882855107 |
| mature_imu | m | 0.00458494617735 | 0 | 1.84395151882 |
| mature_imu | deg | 0.296934067539 | 1.64450759378 | 43.411742328 |

## Frozen windows

For each cell, pipeline statistics are mean/median/P95 on the exact paired common frames. Delta is legacy minus mature.

The absolute boundaries below are taken from the frozen R7I metadata. The broad interval is mapped from the original R7I relative axis; `RAW_IMU_SHOCK` is the separately preidentified absolute IMU-only interval.

| Window | absolute start stamp | absolute end stamp | R7I relative bounds / definition |
| --- | --- | --- | --- |
| W0_PRE | 1660857408.197807074 | 1660857413.197807074 | 15–20 s |
| W1_TRANSIENT | 1660857423.197807074 | 1660857428.197807074 | 30–35 s |
| W2_P025 | 1660857429.197807074 | 1660857435.197807074 | 36–42 s |
| W3_P050 | 1660857502.197807074 | 1660857509.197807074 | 109–116 s |
| W4_RAPID | 1660857530.197807074 | 1660857535.197807074 | 137–142 s |
| RAW_IMU_SHOCK | 1660857532.043056 | 1660857534.043056 | ABSOLUTE_RAW_IMU_PEAK_PLUS_MINUS_1S |
| BROAD_HIGH_DYNAMIC | 1660857531.197807074 | 1660857561.197807074 | 138–168 s |

### Final-used translation (m)

| Window | n | Legacy mean/median/P95 | Mature mean/median/P95 | paired Δ mean | paired Δ median |
| --- | --- | --- | --- | --- | --- |
| W0_PRE | 50 | 0.0425053374499/0.0306878479036/0.121064821511 | 0.0578075825241/0.0568963118633/0.0924345889316 | -0.0153022450742 | -0.0275108338382 |
| W1_TRANSIENT | 50 | 0.854178632037/0.366661754082/1.83423027699 | 0.985883945547/1.0018250901/1.81976681057 | -0.13170531351 | -0.003552534876 |
| W2_P025 | 60 | 0.31277980049/0.329784583485/0.365048785813 | 0.286727840237/0.297394162353/0.325523626492 | 0.0260519602534 | 0.0261331601645 |
| W3_P050 | 70 | 0.719961807983/0.712691818791/0.817350364977 | 0.706485057961/0.711955713477/0.818114355889 | 0.0134767500223 | 0.0119891609725 |
| W4_RAPID | 49 | 3.18198286858/2.60597920099/6.29099793584 | 2.99401334731/2.23164912179/7.78465339732 | 0.187969521276 | 0.211104728375 |
| RAW_IMU_SHOCK | 20 | 3.52977522523/3.7265623283/5.46746573321 | 2.78356861549/2.67790743168/4.7792708206 | 0.746206609742 | 0.787564229395 |
| BROAD_HIGH_DYNAMIC | 296 | 12.9493160808/12.4811212891/21.9498872208 | 9.90644988789/9.49162784779/19.6342193661 | 3.04286619288 | 3.89003406988 |

### Final-used rotation (deg)

| Window | n | Legacy mean/median/P95 | Mature mean/median/P95 | paired Δ mean | paired Δ median |
| --- | --- | --- | --- | --- | --- |
| W0_PRE | 50 | 0.743533293277/0.70109974324/1.34050469138 | 0.363431517618/0.352616404384/0.75639190332 | 0.380101775659 | 0.325384555336 |
| W1_TRANSIENT | 50 | 9.26629106359/10.8241128434/16.4389613676 | 9.09777011858/10.8503532/16.0708274995 | 0.168520945016 | 0.14426877739 |
| W2_P025 | 60 | 2.99226881996/2.87221023359/3.81770378272 | 2.12832471545/2.1090824543/3.23050602888 | 0.863944104509 | 0.93476337273 |
| W3_P050 | 70 | 3.17662147222/1.81088333426/9.26103108859 | 2.54704524396/0.862057932382/8.17174957843 | 0.629576228264 | 0.839157041238 |
| W4_RAPID | 49 | 17.8445751403/21.2210443092/31.5487131495 | 17.6319143899/21.2657637381/31.8520573986 | 0.212660750343 | -0.0778582828 |
| RAW_IMU_SHOCK | 20 | 24.3361438162/26.2565778905/32.3873849087 | 24.4195164325/26.5314608271/32.7808599338 | -0.0833726162685 | -0.18716759395 |
| BROAD_HIGH_DYNAMIC | 296 | 37.9045200015/42.531561326/44.7118102917 | 40.3738123601/45.2833561081/47.5223324378 | -2.46929235862 | -2.67266250245 |

### Raw-NDT translation (m)

| Window | n | Legacy mean/median/P95 | Mature mean/median/P95 | paired Δ mean | paired Δ median |
| --- | --- | --- | --- | --- | --- |
| W0_PRE | 50 | 0.0425053374499/0.0306878479036/0.121064821511 | 0.0578075825241/0.0568963118633/0.0924345889316 | -0.0153022450742 | -0.0275108338382 |
| W1_TRANSIENT | 50 | 0.854043219779/0.320224167178/1.83423027699 | 0.976171895006/0.925117319757/1.81961588265 | -0.122128675227 | 0.002001240808 |
| W2_P025 | 60 | 0.31279227821/0.329784583485/0.365048785813 | 0.28682992381/0.297792714272/0.325523626492 | 0.0259623544002 | 0.0261331601645 |
| W3_P050 | 70 | 0.720124143691/0.713039092992/0.817738965074 | 0.706669329539/0.711955713477/0.818663189864 | 0.0134548141526 | 0.0119891609725 |
| W4_RAPID | 49 | 3.18143885235/2.60316114167/6.29099793584 | 2.99329773715/2.2294299691/7.78465339732 | 0.188141115199 | 0.214861006034 |
| RAW_IMU_SHOCK | 20 | 3.52842755336/3.72631230849/5.46746573321 | 2.7818532117/2.67747453016/4.7792708206 | 0.746574341659 | 0.787540603155 |
| BROAD_HIGH_DYNAMIC | 296 | 12.9485192722/12.4811212891/21.9498872208 | 9.90633627405/9.49162784779/19.6342193661 | 3.04218299815 | 3.89003406988 |

### Raw-NDT rotation (deg)

| Window | n | Legacy mean/median/P95 | Mature mean/median/P95 | paired Δ mean | paired Δ median |
| --- | --- | --- | --- | --- | --- |
| W0_PRE | 50 | 0.743533293277/0.70109974324/1.34050469138 | 0.363431517618/0.352616404384/0.75639190332 | 0.380101775659 | 0.325384555336 |
| W1_TRANSIENT | 50 | 7.72587414165/3.13466341466/15.6262542202 | 8.02787965749/9.60611496157/15.6376121908 | -0.302005515841 | 0.153009133205 |
| W2_P025 | 60 | 2.84552126879/2.84228797726/3.43257543812 | 1.94120851857/2.02277097487/2.47250082929 | 0.904312750223 | 0.940960214615 |
| W3_P050 | 70 | 2.98870897742/1.81088333426/8.44539340792 | 2.34974257559/0.862057932382/7.4657571682 | 0.638966401829 | 0.839157041238 |
| W4_RAPID | 49 | 17.1728719997/19.2226939933/31.3297754781 | 16.9272814892/19.6271379861/31.5674548359 | 0.245590510536 | 0.1214257421 |
| RAW_IMU_SHOCK | 20 | 22.7464482577/25.3689926481/31.4267592077 | 22.7609246022/25.4975893297/31.7188627594 | -0.0144763444625 | -0.0819859014 |
| BROAD_HIGH_DYNAMIC | 296 | 37.7329833099/42.4772110385/44.7118102917 | 40.1984928554/45.0954440403/47.5151243142 | -2.4655095455 | -2.67266250245 |

## Recomputed persistent crossings

Both pipelines use the same common-anchor-relative paired population and the reused rule: strict `>`, maximum continuity gap `0.25 s`, persistent duration `>=5 s`. Absolute stamps below are exact source-sample stamps.

| Pipeline | threshold | instant status / s / stamp | persistent status / s / stamp | duration s |
| --- | --- | --- | --- | --- |
| legacy | 0.25 | CROSSED / 31.8699660301 / 1660857425.4712131 | PERSISTENT / 37.8203678131 / 1660857431.4216149 | 5.95040416718 |
| legacy | 0.5 | CROSSED / 31.8699660301 / 1660857425.4712131 | PERSISTENT / 98.534662962 / 1660857492.13591 | 317.691024065 |
| legacy | 1.0 | CROSSED / 31.9708249569 / 1660857425.572072 | PERSISTENT / 138.271308899 / 1660857531.872556 | 277.954378128 |
| legacy | 2.0 | CROSSED / 138.977272987 / 1660857532.5785201 | PERSISTENT / 138.977272987 / 1660857532.5785201 | 277.24841404 |
| legacy | 5.0 | CROSSED / 140.086675882 / 1660857533.687923 | PERSISTENT / 140.086675882 / 1660857533.687923 | 276.139011145 |
| mature_imu | 0.25 | CROSSED / 30.9622809887 / 1660857424.5635281 | PERSISTENT / 37.8203678131 / 1660857431.4216149 | 5.647824049 |
| mature_imu | 0.5 | CROSSED / 31.4665260315 / 1660857425.0677731 | PERSISTENT / 98.534662962 / 1660857492.13591 | 12.5059659481 |
| mature_imu | 1.0 | CROSSED / 31.6682460308 / 1660857425.2694931 | PERSISTENT / 138.674692869 / 1660857532.2759399 | 277.550994158 |
| mature_imu | 2.0 | CROSSED / 138.977272987 / 1660857532.5785201 | PERSISTENT / 138.977272987 / 1660857532.5785201 | 277.24841404 |
| mature_imu | 5.0 | CROSSED / 140.389256001 / 1660857533.9905031 | PERSISTENT / 140.389256001 / 1660857533.9905031 | 8.47170901299 |

Persistent 1 m→5 m intervals: legacy `1.81536698341 s`; mature `1.71456313133 s`. Their relation is `WITHIN_ONE_COMMON_NDT_FRAME` (mature−legacy `-0.10080385208 s`; median exact-common NDT sample spacing `0.100859880447 s`). The classifier uses this rapid-divergence comparison: one-sided availability or a difference exceeding one measured common-frame spacing is a mixed-effect signal; a paired/window mixed effect is accepted only when both rapid intervals exist.

## High-dynamic audit

The raw-IMU preidentified peak is at `1660857533.043056`; the exact ±1 s audit has 400 IMU samples, acceleration-norm mean/P95/max `13.3785857488/25.4612811988/71.2625833236 m/s²`, and gyro norm at peak `0.931906034852 rad/s`.

| Window | pipeline | pose | n | translation mean/median/P95 (m) | rotation mean/median/P95 (deg) | fitness mean | iterations mean | convergence |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| RAW_IMU_SHOCK_WINDOW | legacy | final_used | 20 | 3.52977522523/3.7265623283/5.46746573321 | 24.3361438162/26.2565778905/32.3873849087 | 12.7298083135 | 2.9 | 1 |
| RAW_IMU_SHOCK_WINDOW | legacy | raw_NDT | 20 | 3.52842755336/3.72631230849/5.46746573321 | 22.7464482577/25.3689926481/31.4267592077 | 12.7298083135 | 2.9 | 1 |
| RAW_IMU_SHOCK_WINDOW | mature_imu | final_used | 20 | 2.78356861549/2.67790743168/4.7792708206 | 24.4195164325/26.5314608271/32.7808599338 | 12.6094306339 | 2.6 | 1 |
| RAW_IMU_SHOCK_WINDOW | mature_imu | raw_NDT | 20 | 2.7818532117/2.67747453016/4.7792708206 | 22.7609246022/25.4975893297/31.7188627594 | 12.6094306339 | 2.6 | 1 |
| BROAD_HIGH_DYNAMIC_138_168 | legacy | final_used | 296 | 12.9493160808/12.4811212891/21.9498872208 | 37.9045200015/42.531561326/44.7118102917 | 14.9539655722 | 1.84121621622 | 1 |
| BROAD_HIGH_DYNAMIC_138_168 | legacy | raw_NDT | 296 | 12.9485192722/12.4811212891/21.9498872208 | 37.7329833099/42.4772110385/44.7118102917 | 14.9539655722 | 1.84121621622 | 1 |
| BROAD_HIGH_DYNAMIC_138_168 | mature_imu | final_used | 296 | 9.90644988789/9.49162784779/19.6342193661 | 40.3738123601/45.2833561081/47.5223324378 | 15.6959545459 | 1.39189189189 | 1 |
| BROAD_HIGH_DYNAMIC_138_168 | mature_imu | raw_NDT | 296 | 9.90633627405/9.49162784779/19.6342193661 | 40.1984928554/45.0954440403/47.5151243142 | 15.6959545459 | 1.39189189189 | 1 |

Nearest NDT samples before/at/after the peak target:

| Target | pipeline | target stamp | nearest NDT stamp | dt s | final t/r error | raw t/r error | fitness | iterations | converged | limited |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| BEFORE_1S | legacy | 1660857532.043056 | 1660857532.0742199 | 0.0311639308929 | 1.15632323093 m / 7.65145958984 deg | 1.15623822312 m / 4.72989258669 deg | 0.311787927485358 | 7 | 1 | 1 |
| BEFORE_1S | mature_imu | 1660857532.043056 | 1660857532.0742199 | 0.0311639308929 | 0.890519677973 m / 7.17899093941 deg | 0.889004202036 m / 4.04248941256 deg | 0.2932838452603236 | 7 | 1 | 1 |
| PEAK_NEAREST_FRAME | legacy | 1660857533.043056 | 1660857533.0827639 | 0.0397078990936 | 3.90839276863 m / 27.3200829165 deg | 3.90839276863 m / 27.3200829165 deg | 17.160925179616314 | 4 | 1 | 0 |
| PEAK_NEAREST_FRAME | mature_imu | 1660857533.043056 | 1660857533.0827639 | 0.0397078990936 | 2.80823521637 m / 27.5795305277 deg | 2.80823521637 m / 27.5795305277 deg | 16.34017485052239 | 3 | 1 | 0 |
| AFTER_1S | legacy | 1660857534.043056 | 1660857534.0913069 | 0.04825091362 | 5.63077151167 m / 29.5372970574 deg | 5.63077151167 m / 29.5372970574 deg | 17.018184753357616 | 1 | 1 | 0 |
| AFTER_1S | mature_imu | 1660857534.043056 | 1660857534.0913069 | 0.04825091362 | 5.32428526552 m / 29.9455838211 deg | 5.32428526552 m / 29.9455838211 deg | 16.580411450802828 | 1 | 1 | 0 |

Allowed interpretation: this is a raw-IMU high-dynamic neighborhood and a descriptive pipeline comparison. It does **not** establish a wall collision, physical impact, or causal relation.

## Decision and limits

- FAILURE_CLASSIFICATION: `MIXED_STAGE_DEPENDENT_EFFECT`. Both pipelines show persistent severe thresholds. The classifier jointly uses persistent 1 m/5 m crossing status, the measured 1 m→5 m interval relation (within or beyond one median common-frame spacing), full-population paired translation/rotation delta signs, and the W0/W1-versus-W3/W4 translation-delta pattern. The mixed label means stage/metric-dependent behavior, not that the large failure regime was removed. Its one-frame timing tolerance is derived from observed common NDT timestamp spacing, not a preregistered threshold.
- OLD-MECHANISM TRANSFERABILITY: `NOT_DIRECTLY_TRANSFERABLE`. Prior R7I/R8 fixed-cloud evidence remains conditional on legacy observations and must be retested on mature pipeline; persistent failure alone does not prove identical mechanism.
- R7I/R8 findings are not upgraded to a causal explanation. No deskew-only cause, physical root cause, wrong mode, multimodality, Hessian/geometry degeneracy, or novelty claim.
- `P4_ALLOWED = NO`. No R8A, tuning, runtime edits, visual fusion, or new algorithm work.

## Reproducibility and protection

`analyze_p3_r9c.py` regenerates the CSV/PNG/Markdown result bundle from the manifest. The script verifies hashes, checks the NDT parameter gate before loading GT, extracts the frozen RAW_IMU_SHOCK directly from the shared raw bag without rerunning localization, and records all output populations. Runtime directories were not written by this stage; final Git verification is reported in the handoff.

Artifacts: `p3_r9c_common_frames.csv`, `p3_r9c_frame_metrics.csv`, `p3_r9c_gt_alignment_audit.csv`, `p3_r9c_global_summary.csv`, `p3_r9c_window_summary.csv`, `p3_r9c_crossings.csv`, `p3_r9c_high_dynamic_summary.csv`, `p3_r9c_bootstrap.csv`, `p3_r9c_better_tie_counts.csv`, parameter/input manifests, and five PNG plots.
