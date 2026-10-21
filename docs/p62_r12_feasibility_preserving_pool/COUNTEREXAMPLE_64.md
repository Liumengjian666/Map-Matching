# Frozen real counterexample64

Source: R11 CANDIDATE_POOL.csv and BLOCK_LOSSES.csv; complete identities in ASSET_IDENTITY_BEFORE.json. Arm PROPOSED_DISAGREEMENT; local64/raw10704. All numbers reuse original matrices and losses; no optimizer/pose editing.

| Item | CONTROL | V3 |
|---|---|---|
| Frozen raw candidate0 eligible |false|false|
| Frozen raw candidate1 eligible |true|true|
| Group members |0,1,2|0,1,2|
| Fixed anchor |0|0|
| Group representative |0|1|
| Selection pool |0 (illegal)|1 (legal)|
| Raw nominal evidence anchor |0|0, explicitly retained|
| Native / Mean selection |-1 / -1|1 / 1|
| Robust selection |-1|-1|
| Final observation |NO_ACCEPTABLE_MEASUREMENT|NO_ACCEPTABLE_MEASUREMENT|

T1 relative prediction .0347931420118104m /4.9395115173828303deg; overlap .9725433526011561.
Its frozen mean loss .087390864795153275 and native score2.2077597340213582 do not by themselves justify acceptance.
Physical distance to T0 .0063057489975798m /.15723682918985446deg remains within unchanged thresholds.

D(1,0)=-.0010521137862698788; D(0,1)=.00008406563164323598, B=7.
The former equals the R11 excluded-pair diagnostic up to printed floating precision. It remains below strict>.02.

FEASIBLE_REPRESENTATIVE_PRESERVED: true.
ROBUST_EVIDENCE_SUPPORTED: false.
PHYSICALLY_DISTINCT_RECOVERY: false.
ACCEPTED_REPLACEMENT: false.

This closes the approved counterexample without changing evidence thresholds. No further64-focused audit or experiment is launched.
