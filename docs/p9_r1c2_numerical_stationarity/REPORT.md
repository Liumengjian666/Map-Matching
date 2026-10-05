# R1C2 numerical stationarity closure

FINAL_RESULT = FLOAT_NUMERICAL_STATIONARITY_FLOOR_CONFIRMED
NEXT = SUPPORT_AWARE_CONTINUATION_R1C3

## Decision evidence

Original strict certificate: 0/14. New complete certificate: 11/14 logical, 6/9 independent. Six TX616 forward records are one root, not six independent observations.
13/14 original supports are exactly self-consistent. P10 reverse remains support-unresolved. Both P09 endpoints additionally retain original-h FLOAT directional FD failures and are NOT certified.
All 9 independent references converge under the stricter double control. No root requires a displacement or energy reduction exceeding its measured numerical envelope. This is approximate numerical stationarity, not an exact smooth-branch or continuation certificate.

FLOAT: PointXYZ / Matrix4f / float transform / double accumulation. DOUBLE: the same input float values exactly cast to double / double product-chart composition and transform / double accumulation.
Formula, frozen leaf ordering/membership and guards are unchanged; 126 pose probes have zero guard disagreements. No dynamic Hessian, support optimization, NDT align, continuation, GT, posterior or EKF was used.

## Value parity

| gap | mean | median | P95 | max |
|---|---:|---:|---:|---:|
| per-source mean E | 3.10916725e-08 | 2.49727639e-08 | 9.78853132e-08 | 1.47254099e-07 |
| total score | 1.31082592e-05 | 9.47596563e-06 | 5.42284635e-05 | 8.15787709e-05 |

The statistics above use 126 logical probes (81 independent poses); duplicate forward inputs are disclosed.

## Independent-root classification

| root | old g | support equal | FD valid | old/new PASS | double dN (finest) | DeltaE (finest) | EPS_G | EPS_dv | EPS_E |
|---|---:|---|---|---|---:|---:|---:|---:|---:|
| 616/SHARED_FORWARD | 5.7202831e-05 | True | True | False/True | 1.7585808e-06 | 8.3014394e-12 | 0.00086365202 | 6.0978204e-05 | 1.0115845e-07 |
| 616/P02/REVERSE | 3.4755273e-05 | True | True | False/True | 4.5312405e-06 | 7.7365103e-11 | 0.00095659519 | 3.2898501e-05 | 1.7396063e-07 |
| 616/P03/REVERSE | 2.9398329e-05 | True | True | False/True | 1.620861e-06 | 1.2979194e-11 | 0.00060127695 | 2.313937e-05 | 9.1696961e-08 |
| 616/P10/REVERSE | 0.00013340055 | False | True | False/False | 1.9779466e-05 | 1.0875355e-09 | 0.00044038898 | 4.176545e-05 | 9.8282739e-08 |
| 616/P12/REVERSE | 2.236575e-05 | True | True | False/True | 8.1086928e-07 | 7.4494793e-12 | 0.00057268862 | 2.3824195e-05 | 1.3076229e-07 |
| 616/P13/REVERSE | 7.7978199e-05 | True | True | False/True | 1.4036601e-05 | 5.1829199e-10 | 0.0010267992 | 6.980731e-05 | 8.0854912e-08 |
| 616/P17/REVERSE | 5.3035565e-05 | True | True | False/True | 1.9081262e-06 | 1.3206848e-11 | 0.00079429974 | 5.0406085e-05 | 1.5237132e-07 |
| 2226/SHARED_FORWARD | 0.00015221146 | True | False | False/False | 6.765607e-06 | 1.8710788e-10 | 0.0014428596 | 6.3565611e-05 | 2.9207907e-07 |
| 2226/P09/REVERSE | 0.00018461064 | True | False | False/False | 9.2919571e-06 | 2.8547328e-10 | 0.0018705194 | 5.2034759e-05 | 2.945887e-07 |

The 616 shared-forward row maps to P02/P03/P10/P12/P13/P17. All 14 logical classifications are in stationarity_roots.csv.

## Strict double reference and projection

| root | steps | delta v norm | translation m | geodesic rotation deg | E decrease | double g | projected float g | projected support equal |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| 616/SHARED_FORWARD | 5 | 1.75918433e-06 | 1.40532613e-06 | 5.4002186e-06 | 8.31734681e-12 | 1.43072406e-10 | 6.93871952e-05 | 1 |
| 616/P02/REVERSE | 4 | 4.53007458e-06 | 2.62542432e-06 | 0.000178916926 | 7.73217046e-11 | 1.81852829e-10 | 2.01700531e-05 | 1 |
| 616/P03/REVERSE | 3 | 1.62220764e-06 | 5.75193104e-07 | 8.33171149e-05 | 1.30269129e-11 | 9.54909988e-11 | 3.843576e-05 | 1 |
| 616/P10/REVERSE | 3 | 1.97779806e-05 | 1.54056242e-05 | 0.000258127492 | 1.08742571e-09 | 3.8209982e-10 | 4.62663547e-05 | 0 |
| 616/P12/REVERSE | 4 | 8.11170771e-07 | 1.33605192e-07 | 4.54788481e-05 | 7.49444951e-12 | 2.86428934e-10 | 3.84201087e-05 | 1 |
| 616/P13/REVERSE | 6 | 1.40363999e-05 | 1.12169992e-05 | 3.73385851e-05 | 5.18364907e-10 | 2.54261143e-10 | 4.3381109e-05 | 1 |
| 616/P17/REVERSE | 6 | 1.90766613e-06 | 1.47706811e-06 | 2.74759356e-05 | 1.31876732e-11 | 7.21899608e-11 | 4.32087113e-05 | 1 |
| 2226/SHARED_FORWARD | 5 | 6.76567426e-06 | 4.74672986e-06 | 0.000186268259 | 1.87129423e-10 | 3.99565796e-10 | 3.82291102e-05 | 1 |
| 2226/P09/REVERSE | 5 | 9.29144396e-06 | 6.49256092e-06 | 0.000258847811 | 2.8536018e-10 | 3.04277707e-10 | 6.51915946e-05 | 1 |

Reference termination is resolution-aware AND no actual decrease exists among the 16 declared dyadic Newton trials. It is not a proof against every possible descent direction. The stricter control was added after the first conservative rho-only measurements; thresholds/envelopes were NOT widened. See THEORY amendment.

## Multi-h data (all independent roots)

Adjacent gradient direction angles, displacement relative changes and full-spectrum relative changes are in multih_stability.csv. Near-zero direction labels refer to the measured FLOAT envelope; raw angles are retained rather than treated as a robust orientation measurement.

### 616/SHARED_FORWARD

Stable DOUBLE h: 0.001;0.0005;0.00025;0.000125; stable FLOAT h: NONE.
Gradient direction: DIRECTION_UNRESOLVED_NEAR_ZERO; reference endpoint stable regions: {'STAR_FLOAT': [], 'STAR_DOUBLE': []}.

| precision | h | g norm | dN norm | DeltaE | H eigenvalues |
|---|---:|---:|---:|---:|---|
| FLOAT | 0.004 | 0.000102113 | 2.2853155e-06 | 9.0851048e-11 | 5.2643957, 8.8859963, 46.742301, 74.065147 |
| FLOAT | 0.002 | 1.9197512e-05 | 8.7325229e-07 | 5.1606437e-12 | 5.2795727, 8.9065522, 46.734816, 74.099262 |
| FLOAT | 0.001 | 5.7202831e-05 | 6.2875577e-06 | 1.3670486e-10 | 5.2988125, 8.9807997, 46.820977, 74.122858 |
| FLOAT | 0.0005 | 7.2450911e-05 | 9.3854344e-06 | 3.2044027e-10 | 5.7485554, 9.3050038, 46.756851, 74.510268 |
| FLOAT | 0.00025 | 0.00019829535 | 5.5825424e-06 | 4.1101343e-10 | 7.3229968, 11.480278, 47.416782, 74.931232 |
| FLOAT | 0.000125 | 0.00037153597 | 2.8680811e-05 | 4.685507e-09 | 10.295273, 13.71214, 50.947349, 78.805727 |
| DOUBLE | 0.004 | 9.6834769e-05 | 2.3457828e-06 | 8.5157805e-11 | 5.261693, 8.8823122, 46.736798, 74.060151 |
| DOUBLE | 0.002 | 2.4330833e-05 | 1.6833818e-06 | 1.1415419e-11 | 5.2617221, 8.8824092, 46.739223, 74.068451 |
| DOUBLE | 0.001 | 1.0303752e-05 | 1.7249876e-06 | 8.0797731e-12 | 5.2617294, 8.8824335, 46.73983, 74.070526 |
| DOUBLE | 0.0005 | 9.5726817e-06 | 1.7497446e-06 | 8.1897299e-12 | 5.2617312, 8.8824396, 46.739981, 74.071045 |
| DOUBLE | 0.00025 | 9.7306859e-06 | 1.7567758e-06 | 8.2761801e-12 | 5.2617317, 8.8824411, 46.740019, 74.071174 |
| DOUBLE | 0.000125 | 9.7911176e-06 | 1.7585808e-06 | 8.3014394e-12 | 5.2617316, 8.8824414, 46.740029, 74.071207 |

### 616/P02/REVERSE

Stable DOUBLE h: 0.002;0.001;0.0005;0.00025;0.000125; stable FLOAT h: NONE.
Gradient direction: DIRECTION_UNRESOLVED_NEAR_ZERO; reference endpoint stable regions: {'STAR_FLOAT': [], 'STAR_DOUBLE': []}.

| precision | h | g norm | dN norm | DeltaE | H eigenvalues |
|---|---:|---:|---:|---:|---|
| FLOAT | 0.004 | 0.00011260976 | 5.5799234e-06 | 1.8825372e-10 | 5.2912684, 9.1799054, 46.509388, 72.638962 |
| FLOAT | 0.002 | 5.7320097e-05 | 5.2992636e-06 | 1.298232e-10 | 5.3288478, 9.2050203, 46.547886, 72.686896 |
| FLOAT | 0.001 | 3.4755273e-05 | 2.4002301e-06 | 2.3633856e-11 | 5.4964929, 9.34498, 46.667564, 72.740454 |
| FLOAT | 0.0005 | 0.00011294926 | 5.943296e-06 | 2.494709e-10 | 6.2338747, 9.8945023, 47.207396, 73.542475 |
| FLOAT | 0.00025 | 0.00015575557 | 1.070881e-05 | 6.8561774e-10 | 7.7263996, 11.434272, 47.841728, 75.395608 |
| FLOAT | 0.000125 | 0.00049189534 | 1.7663794e-05 | 3.4932486e-09 | 13.392839, 19.025764, 60.030634, 83.519983 |
| DOUBLE | 0.004 | 0.0001095081 | 5.8955249e-06 | 1.9713085e-10 | 5.2816561, 9.1680859, 46.500564, 72.627213 |
| DOUBLE | 0.002 | 4.6468129e-05 | 4.8413806e-06 | 9.3615037e-11 | 5.2816835, 9.1681768, 46.503104, 72.635144 |
| DOUBLE | 0.001 | 3.6939631e-05 | 4.6055197e-06 | 8.0539477e-11 | 5.2816904, 9.1681995, 46.503738, 72.637127 |
| DOUBLE | 0.0005 | 3.5634747e-05 | 4.5487752e-06 | 7.8070825e-11 | 5.2816921, 9.1682052, 46.503897, 72.637623 |
| DOUBLE | 0.00025 | 3.5392942e-05 | 4.534739e-06 | 7.7503738e-11 | 5.2816926, 9.1682067, 46.503937, 72.637747 |
| DOUBLE | 0.000125 | 3.5337979e-05 | 4.5312405e-06 | 7.7365103e-11 | 5.2816932, 9.1682073, 46.503947, 72.637778 |

### 616/P03/REVERSE

Stable DOUBLE h: 0.001;0.0005;0.00025;0.000125; stable FLOAT h: NONE.
Gradient direction: DIRECTION_UNRESOLVED_NEAR_ZERO; reference endpoint stable regions: {'STAR_FLOAT': [], 'STAR_DOUBLE': []}.

| precision | h | g norm | dN norm | DeltaE | H eigenvalues |
|---|---:|---:|---:|---:|---|
| FLOAT | 0.004 | 0.00010206214 | 2.1041683e-06 | 7.8944158e-11 | 5.2609543, 9.0505659, 46.634461, 72.784039 |
| FLOAT | 0.002 | 1.4861591e-05 | 2.0326951e-06 | 1.1956799e-11 | 5.277874, 9.048123, 46.668321, 72.802687 |
| FLOAT | 0.001 | 2.9398329e-05 | 2.0695981e-06 | 1.7883149e-11 | 5.2735724, 9.0283709, 46.668956, 72.860784 |
| FLOAT | 0.0005 | 8.0561569e-05 | 8.7493281e-06 | 2.7951465e-10 | 5.4327634, 9.1394442, 46.772645, 72.90282 |
| FLOAT | 0.00025 | 8.4145869e-05 | 8.0388814e-06 | 2.8809534e-10 | 5.370316, 10.12394, 47.85521, 73.717819 |
| FLOAT | 0.000125 | 0.00029232881 | 1.3904193e-05 | 1.5147223e-09 | 9.0720848, 9.5572422, 46.976874, 75.617753 |
| DOUBLE | 0.004 | 9.1542624e-05 | 1.4006343e-06 | 6.0949614e-11 | 5.2579445, 9.0444015, 46.630099, 72.780682 |
| DOUBLE | 0.002 | 1.8102439e-05 | 1.3256081e-06 | 8.8027879e-12 | 5.2579741, 9.0445001, 46.63271, 72.788687 |
| DOUBLE | 0.001 | 1.6240034e-05 | 1.5390842e-06 | 1.0950817e-11 | 5.2579815, 9.0445247, 46.633363, 72.790688 |
| DOUBLE | 0.0005 | 1.9224942e-05 | 1.600963e-06 | 1.2436952e-11 | 5.2579833, 9.0445309, 46.633526, 72.791189 |
| DOUBLE | 0.00025 | 2.0091793e-05 | 1.616864e-06 | 1.286782e-11 | 5.2579837, 9.0445323, 46.633567, 72.791314 |
| DOUBLE | 0.000125 | 2.0314405e-05 | 1.620861e-06 | 1.2979194e-11 | 5.2579834, 9.0445319, 46.633576, 72.791344 |

### 616/P10/REVERSE

Stable DOUBLE h: 0.004;0.002;0.001;0.0005;0.00025;0.000125; stable FLOAT h: NONE.
Gradient direction: DIRECTION_UNRESOLVED_NEAR_ZERO; reference endpoint stable regions: {'STAR_FLOAT': [], 'STAR_DOUBLE': []}.

| precision | h | g norm | dN norm | DeltaE | H eigenvalues |
|---|---:|---:|---:|---:|---|
| FLOAT | 0.004 | 0.00012895049 | 2.1762412e-05 | 1.3358625e-09 | 5.0997948, 8.7793226, 48.974379, 74.715455 |
| FLOAT | 0.002 | 0.00011815737 | 1.9002115e-05 | 1.0276371e-09 | 5.1211135, 8.7946135, 48.989094, 74.721943 |
| FLOAT | 0.001 | 0.00013340055 | 2.0820946e-05 | 1.1766819e-09 | 5.1472438, 8.8899053, 49.029931, 74.794572 |
| FLOAT | 0.0005 | 0.00014448282 | 1.9141744e-05 | 1.3120267e-09 | 5.4745115, 8.9929531, 49.040054, 74.86985 |
| FLOAT | 0.00025 | 0.00012586134 | 9.9066025e-06 | 4.9859484e-10 | 5.3022252, 9.5907443, 48.95906, 75.573636 |
| FLOAT | 0.000125 | 0.00018937513 | 4.6434229e-06 | 3.1429201e-10 | 8.2670423, 13.841125, 55.448965, 78.443437 |
| DOUBLE | 0.004 | 0.00012422188 | 2.1367331e-05 | 1.2635861e-09 | 5.0974689, 8.7786561, 48.968934, 74.704496 |
| DOUBLE | 0.002 | 0.00011809129 | 2.0164569e-05 | 1.1200116e-09 | 5.0974526, 8.7785808, 48.97185, 74.712973 |
| DOUBLE | 0.001 | 0.00012125167 | 1.9873899e-05 | 1.0948545e-09 | 5.0974485, 8.778562, 48.972579, 74.715092 |
| DOUBLE | 0.0005 | 0.00012231744 | 1.9801905e-05 | 1.0892361e-09 | 5.0974475, 8.7785573, 48.972761, 74.715622 |
| DOUBLE | 0.00025 | 0.00012260043 | 1.9783954e-05 | 1.0878739e-09 | 5.0974474, 8.7785562, 48.972807, 74.715755 |
| DOUBLE | 0.000125 | 0.00012267216 | 1.9779466e-05 | 1.0875355e-09 | 5.0974475, 8.7785562, 48.972818, 74.715789 |

### 616/P12/REVERSE

Stable DOUBLE h: 0.001;0.0005;0.00025;0.000125; stable FLOAT h: NONE.
Gradient direction: DIRECTION_UNRESOLVED_NEAR_ZERO; reference endpoint stable regions: {'STAR_FLOAT': [], 'STAR_DOUBLE': []}.

| precision | h | g norm | dN norm | DeltaE | H eigenvalues |
|---|---:|---:|---:|---:|---|
| FLOAT | 0.004 | 9.9761058e-05 | 2.4973041e-06 | 1.0326009e-10 | 5.2606588, 8.7163247, 46.719135, 72.610519 |
| FLOAT | 0.002 | 2.9878693e-05 | 5.0567557e-06 | 6.9600463e-11 | 5.2785751, 8.7421496, 46.72553, 72.618858 |
| FLOAT | 0.001 | 2.236575e-05 | 6.7292422e-07 | 6.0194928e-12 | 5.2083777, 8.7924172, 46.802322, 72.646294 |
| FLOAT | 0.0005 | 0.00010343578 | 1.0659281e-05 | 4.0554488e-10 | 5.3062159, 9.0369218, 46.961939, 72.85351 |
| FLOAT | 0.00025 | 0.00011789558 | 1.3357823e-05 | 5.833905e-10 | 5.8808511, 9.7253486, 47.739243, 75.091621 |
| FLOAT | 0.000125 | 0.00028609009 | 6.8098726e-06 | 7.4378905e-10 | 8.126659, 13.859295, 53.985736, 76.623729 |
| DOUBLE | 0.004 | 0.00010268803 | 2.7503045e-06 | 1.1399576e-10 | 5.2612854, 8.7127723, 46.712125, 72.608046 |
| DOUBLE | 0.002 | 2.0148461e-05 | 9.9834844e-07 | 6.3992825e-12 | 5.2613162, 8.7128705, 46.714691, 72.615989 |
| DOUBLE | 0.001 | 1.9098595e-05 | 8.0987691e-07 | 5.484089e-12 | 5.2613239, 8.712895, 46.715332, 72.617975 |
| DOUBLE | 0.0005 | 2.2591471e-05 | 8.0749079e-07 | 6.8799734e-12 | 5.2613258, 8.7129012, 46.715492, 72.618472 |
| DOUBLE | 0.00025 | 2.359079e-05 | 8.1003801e-07 | 7.3305032e-12 | 5.2613262, 8.7129027, 46.715532, 72.618596 |
| DOUBLE | 0.000125 | 2.3846843e-05 | 8.1086928e-07 | 7.4494793e-12 | 5.2613263, 8.7129032, 46.715543, 72.618627 |

### 616/P13/REVERSE

Stable DOUBLE h: 0.004;0.002;0.001;0.0005;0.00025;0.000125; stable FLOAT h: NONE.
Gradient direction: DIRECTION_UNRESOLVED_NEAR_ZERO; reference endpoint stable regions: {'STAR_FLOAT': [], 'STAR_DOUBLE': []}.

| precision | h | g norm | dN norm | DeltaE | H eigenvalues |
|---|---:|---:|---:|---:|---|
| FLOAT | 0.004 | 0.00013818555 | 1.5599835e-05 | 7.4538918e-10 | 5.1520775, 8.5589205, 47.769094, 72.926846 |
| FLOAT | 0.002 | 8.6792097e-05 | 1.6085425e-05 | 6.7403483e-10 | 5.1772609, 8.5736628, 47.77235, 72.9601 |
| FLOAT | 0.001 | 7.7978199e-05 | 1.3212685e-05 | 4.7558753e-10 | 5.2743117, 8.6600799, 47.838912, 72.979028 |
| FLOAT | 0.0005 | 0.00015951743 | 2.4020329e-05 | 1.6587062e-09 | 5.4471059, 8.9749, 47.796998, 73.455676 |
| FLOAT | 0.00025 | 0.0001869599 | 1.6527102e-05 | 1.0616774e-09 | 5.9039058, 9.9847548, 48.815123, 73.266682 |
| FLOAT | 0.000125 | 0.00052072953 | 1.4918336e-05 | 2.8101803e-09 | 11.476304, 12.436282, 57.621218, 78.480341 |
| DOUBLE | 0.004 | 0.00013679841 | 1.4597521e-05 | 6.707109e-10 | 5.1456338, 8.5551757, 47.765748, 72.925228 |
| DOUBLE | 0.002 | 7.5125768e-05 | 1.4113191e-05 | 5.1575558e-10 | 5.1456571, 8.555224, 47.768741, 72.9333 |
| DOUBLE | 0.001 | 7.7244588e-05 | 1.4051499e-05 | 5.1516037e-10 | 5.145663, 8.5552361, 47.769489, 72.935318 |
| DOUBLE | 0.0005 | 7.9196009e-05 | 1.4039908e-05 | 5.1739722e-10 | 5.1456644, 8.5552391, 47.769676, 72.935822 |
| DOUBLE | 0.00025 | 7.9764445e-05 | 1.403725e-05 | 5.1810556e-10 | 5.1456649, 8.5552397, 47.769723, 72.935948 |
| DOUBLE | 0.000125 | 7.9911419e-05 | 1.4036601e-05 | 5.1829199e-10 | 5.1456647, 8.5552396, 47.769734, 72.93598 |

### 616/P17/REVERSE

Stable DOUBLE h: 0.001;0.0005;0.00025;0.000125; stable FLOAT h: NONE.
Gradient direction: DIRECTION_UNRESOLVED_NEAR_ZERO; reference endpoint stable regions: {'STAR_FLOAT': [], 'STAR_DOUBLE': []}.

| precision | h | g norm | dN norm | DeltaE | H eigenvalues |
|---|---:|---:|---:|---:|---|
| FLOAT | 0.004 | 0.00012309132 | 4.7670665e-06 | 1.9175278e-10 | 5.1457188, 8.3782666, 47.692961, 73.328731 |
| FLOAT | 0.002 | 6.3609046e-05 | 3.9718884e-06 | 8.2974906e-11 | 5.1636711, 8.3983066, 47.726078, 73.388581 |
| FLOAT | 0.001 | 5.3035565e-05 | 7.9241857e-06 | 1.8238531e-10 | 5.374892, 8.5191089, 47.816606, 73.481603 |
| FLOAT | 0.0005 | 6.5649667e-05 | 2.5821619e-06 | 5.4039157e-11 | 5.7156915, 8.806626, 48.2379, 74.12881 |
| FLOAT | 0.00025 | 0.00012712946 | 1.2455353e-05 | 7.6988903e-10 | 7.8948536, 10.930308, 51.206553, 75.429369 |
| FLOAT | 0.000125 | 0.00041041227 | 2.3781979e-05 | 4.0916846e-09 | 10.712023, 17.947742, 58.867126, 82.880058 |
| DOUBLE | 0.004 | 0.00011751443 | 3.2929838e-06 | 1.5090273e-10 | 5.1343976, 8.3677729, 47.688805, 73.321012 |
| DOUBLE | 0.002 | 4.4324939e-05 | 2.1009709e-06 | 2.6460049e-11 | 5.1344222, 8.3677935, 47.691985, 73.329364 |
| DOUBLE | 0.001 | 2.9498801e-05 | 1.9413341e-06 | 1.5178807e-11 | 5.1344284, 8.3677987, 47.69278, 73.331453 |
| DOUBLE | 0.0005 | 2.6708106e-05 | 1.915104e-06 | 1.3598835e-11 | 5.1344299, 8.3678, 47.692979, 73.331975 |
| DOUBLE | 0.00025 | 2.6103811e-05 | 1.9094746e-06 | 1.3281374e-11 | 5.1344305, 8.3678003, 47.693029, 73.332105 |
| DOUBLE | 0.000125 | 2.5959313e-05 | 1.9081262e-06 | 1.3206848e-11 | 5.1344305, 8.3678002, 47.693041, 73.332138 |

### 2226/SHARED_FORWARD

Stable DOUBLE h: 0.004;0.002;0.001;0.0005;0.00025;0.000125; stable FLOAT h: NONE.
Gradient direction: DIRECTION_UNRESOLVED_NEAR_ZERO; reference endpoint stable regions: {'STAR_FLOAT': [], 'STAR_DOUBLE': []}.

| precision | h | g norm | dN norm | DeltaE | H eigenvalues |
|---|---:|---:|---:|---:|---|
| FLOAT | 0.004 | 0.00017479564 | 7.5318694e-06 | 3.0926287e-10 | 5.0486022, 16.066855, 81.422069, 118.99112 |
| FLOAT | 0.002 | 7.1982628e-05 | 6.9855604e-06 | 1.437701e-10 | 5.1345155, 16.099333, 81.51385, 119.07175 |
| FLOAT | 0.001 | 0.00015221146 | 1.113755e-05 | 7.1864091e-10 | 5.4410123, 16.453286, 81.814435, 119.31549 |
| FLOAT | 0.0005 | 0.00014174976 | 2.3429932e-05 | 1.6207455e-09 | 5.884509, 17.061324, 82.531835, 120.42038 |
| FLOAT | 0.00025 | 0.00023992639 | 1.1180291e-05 | 8.0514869e-10 | 9.1023493, 22.751357, 85.392896, 124.67105 |
| FLOAT | 0.000125 | 0.00071526374 | 1.4416198e-05 | 4.6998737e-09 | 25.180487, 40.632094, 102.42798, 147.93843 |
| DOUBLE | 0.004 | 0.00018711598 | 6.8647317e-06 | 3.1489242e-10 | 5.0358671, 16.058565, 81.409343, 118.96341 |
| DOUBLE | 0.002 | 7.8649025e-05 | 6.7538906e-06 | 1.9245184e-10 | 5.0358923, 16.059014, 81.417842, 118.99544 |
| DOUBLE | 0.001 | 6.7044929e-05 | 6.7604458e-06 | 1.8678947e-10 | 5.0358986, 16.059126, 81.419967, 119.00345 |
| DOUBLE | 0.0005 | 6.6463758e-05 | 6.7642381e-06 | 1.8693432e-10 | 5.0359002, 16.059154, 81.420498, 119.00545 |
| DOUBLE | 0.00025 | 6.6480101e-05 | 6.7653199e-06 | 1.8706803e-10 | 5.0359007, 16.059161, 81.420631, 119.00595 |
| DOUBLE | 0.000125 | 6.6494356e-05 | 6.765607e-06 | 1.8710788e-10 | 5.0359015, 16.059163, 81.420665, 119.00608 |

### 2226/P09/REVERSE

Stable DOUBLE h: 0.002;0.001;0.0005;0.00025;0.000125; stable FLOAT h: NONE.
Gradient direction: DIRECTION_UNRESOLVED_NEAR_ZERO; reference endpoint stable regions: {'STAR_FLOAT': [], 'STAR_DOUBLE': []}.

| precision | h | g norm | dN norm | DeltaE | H eigenvalues |
|---|---:|---:|---:|---:|---|
| FLOAT | 0.004 | 0.00054987638 | 1.1112534e-05 | 1.7445757e-09 | 4.4302159, 14.541723, 81.37693, 105.11652 |
| FLOAT | 0.002 | 0.00019276977 | 1.1057203e-05 | 6.89284e-10 | 4.4748473, 14.635134, 81.462242, 105.19184 |
| FLOAT | 0.001 | 0.00018461064 | 6.9161566e-06 | 4.2848871e-10 | 4.7070293, 14.88943, 81.66596, 105.51725 |
| FLOAT | 0.0005 | 0.00023904747 | 2.7320509e-05 | 2.3851776e-09 | 5.9039407, 16.722849, 82.384777, 105.82662 |
| FLOAT | 0.00025 | 0.00048906259 | 1.183311e-05 | 2.0218466e-09 | 13.016572, 17.811149, 87.808585, 110.89472 |
| FLOAT | 0.000125 | 0.00077157487 | 2.9134865e-05 | 9.4418148e-09 | 18.571512, 22.568146, 100.51202, 127.10731 |
| DOUBLE | 0.004 | 0.00056851963 | 1.109963e-05 | 1.8231044e-09 | 4.4068589, 14.529633, 81.357632, 105.1031 |
| DOUBLE | 0.002 | 0.00017184634 | 9.5125264e-06 | 4.0558184e-10 | 4.4068703, 14.529109, 81.369058, 105.13065 |
| DOUBLE | 0.001 | 9.0092878e-05 | 9.3302581e-06 | 2.9887788e-10 | 4.4068731, 14.528977, 81.371915, 105.13754 |
| DOUBLE | 0.0005 | 7.757391e-05 | 9.3000948e-06 | 2.8769622e-10 | 4.4068738, 14.528944, 81.372629, 105.13926 |
| DOUBLE | 0.00025 | 7.5474762e-05 | 9.2935423e-06 | 2.8586975e-10 | 4.4068738, 14.528936, 81.372808, 105.13969 |
| DOUBLE | 0.000125 | 7.5025571e-05 | 9.2919571e-06 | 2.8547328e-10 | 4.4068731, 14.528933, 81.372852, 105.1398 |

## Required cases

P12 reverse: original closure is essentially zero, support equal and H SPD; its tiny correction to a double stationary reference and persistent float residual establish the numerical-floor interpretation.
P09 T0: the original FLOAT h=.001 direction-0 failure is reproduced exactly. The DOUBLE control passes the same unchanged FD audit; this attributes that directional discrepancy to numerical representation rather than changing the old audit label. P09 endpoints are not complete certificates because their original FLOAT endpoint FD checks remain failed.
P09 reverse: raw g≈1.846e-4 does not imply a large geometric error; the double-reference correction is micrometre / sub-millidegree scale. Its derivative limitation remains explicit.
P10 reverse: stationarity calibration cannot repair its different frozen/dynamic memberships or the old ≈0.926 degree anchor closure. It remains FAIL; no support update was performed here.

## Verification and Git

{
  "csv_json": "PASS",
  "hash": "PASS",
  "inherited_sources": "UNCHANGED",
  "release_build": "PASS",
  "P9_tests": "9/9 PASS",
  "diff_check": "PASS",
  "new_file_whitespace": "PASS",
  "commands": [
    {
      "command": [
        "cmake",
        "--build",
        "/tmp/p9_r1c2_build",
        "-j1"
      ],
      "sha256": "7393443d483cc97b2c2f1f2d71d67f02a74fdda90afb7768b398af34361baa08"
    },
    {
      "command": [
        "ctest",
        "--output-on-failure"
      ],
      "sha256": "c2a26f6aeae679f7c5ea632718e82b5aa40d8ddb5aba965f5bf5bd5e49f068c3"
    },
    {
      "command": [
        "git",
        "diff",
        "--check"
      ],
      "sha256": "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
    }
  ]
}

Actual branch remains research/p9-r1c-support-aware-continuation; START/END HEAD = ad2b472d5ac5c433194444e893d7ca532b950cca. Requested new branch/commit are blocked by read-only .git (index.lock creation denied). No push occurred. User-owned .vscode/ is preserved.

## Limits / next

The complete gate recovers 11 logical roots, not 13. Numerical-floor classification and full certificate validity are distinct. EPS values are local deterministic sensitivity envelopes, not universal tolerances or posterior probabilities. Production R1C solver/certificate were not changed.
NEXT = SUPPORT_AWARE_CONTINUATION_R1C3. Start only from the certified subset; do not quietly include the P09 FD failures or P10 support failure. No continuation is run in R1C2.
