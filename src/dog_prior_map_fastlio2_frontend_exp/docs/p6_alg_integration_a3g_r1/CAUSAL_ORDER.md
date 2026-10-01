# Sensor-time chronology

All joins use exact transaction identity plus exact integer nanosecond stamps;
sequence analyses sort by terminal sensor stamp. Persistence means >=20
consecutive terminal observations, not transaction-ID arithmetic. A scan-start
can interleave a lower-ID scan-end; it is not a terminal or a timestamp fault.

| Terminal/event | Sensor stamp ns | Evidence |
|---|---:|---|
| tx83 | 1517157227458992315 | Earliest NDT–prediction translation innovation >1 m: 1.683044257 m; NIS rejection; later commits resume. Descriptive flag, not proof of incorrect NDT. |
| tx143–145 | 1517157233510227276 to 1517157233711947275 | Three NIS rejections, then recovery tx146. |
| tx151 | 1517157234317045284 | NDT fitness 0.446092, objective 1600.929290, 80 iterations, innovation 3.006156 m / 0.866979 rad; commit. Physical correctness unknown. |
| tx156–159 | 1517157234821345402 to 1517157235123863293 | Four NIS rejections, then recovery tx160. |
| tx160 | 1517157235224723412 | First completed increment >0.5 m: 0.684130637 m. First NDT rotation innovation >pi/2: 1.624361713 rad; NDT translation innovation 1.200321652 m; commit. |
| tx164–165 | 1517157235628163410 to 1517157235729024483 | NIS 22.235976/13.277 and 11.682091/11.345 reject; latest commit tx163 remains active. |
| tx166 | 1517157235829884363 | First no valid geometric correspondence. converged=1, iterations=0, objective=0; translation/rotation innovation 4.47e-7 m / 6.25e-9 rad. NIS_NOT_REACHED. |
| tx166–172 | 1517157235829884363 to 1517157236434983326 | Seven correspondence-loss terminals, not yet a >=20 persistent run. |
| tx173 | 1517157236535843445 | U_obs valid but routed reliable rank 0, rank rejection; NDT innovation 5.149193 m / 0.604309 rad; no preview/NIS. |
| tx174 | 1517157236636703325 | Geometric support and LiDAR commit recover. |
| tx176 | 1517157236838423325 | First completed increment >1 m: 1.219916666 m (12.095163 m/s implied). This terminal commits; starvation is not yet permanent. |
| tx182 | 1517157237443522287 | Last actual commit: selected rank5, NIS 14.251698043/15.086. Eight active factors remain after enforcement. |
| tx183 | 1517157237544382406 | First subsequent noncommit; begins permanent completed rejection run through tx365 (183 terminals), all map-support related. |
| tx185,187 | 1517157237746102406 / 1517157237947822405 | MAP_SUPPORT_INSUFFICIENT, not NO_VALID; break exact-status persistence, not rejection persistence. |
| tx188 | 1517157238048622442 | Starts strict continuous no-valid-correspondence run through tx365: 178 terminals. |
| tx201 | 1517157239359741284 | Last completed terminal with active LiDAR >0: count1. No current commit. |
| tx202 | 1517157239460601402 | First active LiDAR count0. Enforcement302 attempt1 removes tx182's state and consumes one LiDAR factor; attempt2 removes scan-start state. |
| tx215 | 1517157240771719290 | First completed increment >2 m: 2.024439620 m; covariance still AVAILABLE/full rank. |
| tx285 | 1517157247831493450 | First increment >5 m: 5.032843337 m. |
| tx365 | 1517157255899804426 | Largest completed increment 10.221731186 m; covariance still AVAILABLE. |
| tx367 scan-start | 1517157256000623941 | Last completed event, 40366 ns before tx366 end. Not part of terminal increment/rejection counts. |
| tx366 scan-end | 1517157256000664307 | First P15 unavailability, rank498/600. Current NDT, deskew, support, NIS, optimization and marginalization not executed. |

The ordering is **not** “persistent rejection then first motion growth”: >0.5 m
occurs at tx160; temporary support loss at tx166; >1 m at tx176 while measurement
commits still occur; permanent rejection starts tx183. Later accelerations and
covariance failure follow permanent starvation. These thresholds are requested
descriptive milestones, not detector thresholds or localization accuracy tests.

## Active-factor lifecycle versus information

[LAST_ACTIVE_LIDAR_REMOVAL.csv](LAST_ACTIVE_LIDAR_REMOVAL.csv) preserves the four
tx201/202 removal records. tx202 attempt1 removes stamp1517157237443522287
(tx182), incident LiDAR count1, rank15, SUCCESS; nodes41→40, span2.017079115→
2.017058445 s. Attempt2 removes stamp1517157237443542957, incident LiDAR0,
SUCCESS; final completed event nodes39/span1.916218996 s.

P15 is pre-measurement/pre-enforcement. tx202 covariance still includes the last
active historical factor; its completed-event active count is zero afterwards.
tx203 preopt factor counts are 40 IMU / 0 LiDAR / 0 visual; P15 remains valid.
Removal consumes information into the prior, not discards it. No direct evidence
here of factor double-counting, ID loss or premature deletion.

Persistent completed rejection tx183–365 spans 18.355422020 s; failed tx366
extends the no-commit chronology, not the map-support rejection count.
Zero-active tx202–366 spans 16.540062905 s, 165 terminal records; 164 complete,
the final one fails before measuring. No separate covariance failure precedes it.
