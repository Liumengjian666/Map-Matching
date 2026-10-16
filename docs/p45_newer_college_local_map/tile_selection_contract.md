# Tile selection and coverage contract

- Pose input: the causal `map_T_lidar` initial guess already computed for the current CONTROL NDT alignment. The tile manager runs before `align`; current-frame NDT output and GT are not inputs.
- Source input: current scan after the unchanged P44 deskew and source preprocessing (1,400 XYZ points in the frozen run).
- Projection: `p_map = R_map_lidar * p_lidar + t_map_lidar` for every finite source point; compute the source AABB in map coordinates.
- Safety selection: expand the projected XY AABB by 10 m; retain the complete tile Z extent. Tile size is 20 m × 20 m.
- Cache/hysteresis: reuse the active target while it contains all safety-required tiles. On a miss, select with 30 m expansion, verify SHA256 and declared point count for each tile, assemble a replacement, then switch NDT target once.
- Fail-closed conditions: invalid/non-finite pose or source, no intersecting tile, manifest mismatch, tile hash/read/count failure, or invalid/empty target; no partial-target align is performed.
- Memory invariant: NDT local mode parses the small metadata manifest but never loads the global XYZ map, complete P44 target, or global KD-tree. Only active tile point data and the resulting local NDT target are retained by the manager/NDT path.
- Frozen run: initial update selected 21 tiles / 552,716 points. It remained inside this active cache for all 300 frames; one map update total. Separate remote component probe: 22 tiles / 1,186,101 points. The probe used a synthetic pose and is not end-to-end pose validation.
