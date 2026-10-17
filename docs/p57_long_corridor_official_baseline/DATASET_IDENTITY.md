# Dataset identity evidence

## Publisher metadata

The SuperLoc public table identifies Corridor01 as SubT-MRS / Hawkins / RC2, with an approximately 617 m route and 279 s duration. Its downloadable assets include a Corridor01 bag, PCD map, GT trajectory, extrinsics, and an initialization folder. The SubT-MRS challenge listing identifies Long_Corridor as Hawkins / RC2 and reports approximately 616.45 m and 295 s.

The metadata strongly suggests a related route/platform, but does not identify a unique recording. The duration differs by about 16 s, the official bag file IDs differ, and their advertised archive sizes differ. File names and route length are not enough to merge these resources.

## GT lineage evidence, not bag identity

The P10 archive `src/dog_prior_map_localization/docs/superloc_corridor01/P3_R3B_GT_LINEAGE.md` reports that the SuperLoc Corridor01 GT and the official SubT-MRS challenge GT each contain 1,385 poses; translation samples match exactly, normalized quaternion discrepancies are at floating-point scale, and 1,284 timestamps match textually while the remaining timestamp differences are at most 239 ns. This is useful evidence that the published trajectory samples share lineage.

That archived comparison does **not** establish that the two raw bags are byte-identical, were captured in the same pass, have the same start/end boundaries, or have the same packet timestamps. In this P57 run the two bags were not both available for direct comparison; the SubT-MRS ZIP remains incomplete and the SuperLoc raw bag is absent. Therefore:

- GT trajectory lineage: supported by the previously archived direct comparison (not recomputed here).
- Raw acquisition identity: `DATASET_IDENTITY_UNCONFIRMED`.
- Mixing one bag with the other release's map/calibration/init: not authorized by available evidence.

## Relevant official resources

- SuperLoc dataset page: <https://superodometry.com/superloc.html>
- SuperLoc raw archive, Drive ID `1aIIqPiE10nX3IhidpxKhc4Psq-AMER1X`.
- SuperLoc map, Drive ID `1EH4NzINNLkHrneIxstrzkY9XTZ1JP5bf`.
- SuperLoc GT, Drive ID `1QbQamSktPAzJkf0GduAPTqQlxuxM0DBK`.
- SuperLoc extrinsics, Drive ID `13WmYOrqbT6VNkIGL13cRuLiENeanI6oY`.
- SuperLoc initialization folder, Drive ID `1WZsyEYyU-_8ps1CUqRq3YO7IaARMdGNm`; `corridor01.yaml` was obtained from the official folder listing.
- SubT-MRS dataset portal: <https://superodometry.com/datasets>
- ICCV23 SubT-MRS challenge page: <https://superodometry.com/iccv23_challenge_Mul>
- SubT-MRS raw archive, Drive ID `1-WF_cIPXVEzcStfYjJWoZfJkld0Ozmxk`.

No third-party mirror was used.
