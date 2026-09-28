#!/usr/bin/env python3
"""Unit tests for the pre-NDT frozen I5C case selector."""

from __future__ import annotations

import csv
import json
import tempfile
import unittest
from pathlib import Path

import p6_i5c_select_cases as selector


def candidate(tx: str, q: float) -> dict[str, str | float]:
    return {"transaction_id": tx, "frame_id": f"F{tx}", "q": q}


class SelectionProtocolTests(unittest.TestCase):
    def test_alpha_key_uses_positive_half_up_quantization(self):
        self.assertEqual(selector.alpha_key("0.0000005"), 1)
        self.assertEqual(selector.alpha_key("0.046875"), 46875)

    def test_pose_distance_is_quaternion_sign_invariant(self):
        self.assertEqual(selector.pose_distance([1, 2, 3, 0, 0, 0, 1],
                                                [1, 2, 3, 0, 0, 0, -1]), (0.0, 0.0))

    def test_endpoint_logical_key_includes_all_five_fields(self):
        row = {"transaction_id": "7", "ray_type": "EXTRA_MARGIN",
               "ray_id": "12", "sign": "-1"}
        self.assertEqual(selector.endpoint_key(row, "0.5"),
                         ("7", "EXTRA_MARGIN", 12, -1, 500000))

    def test_duplicate_raw_endpoint_conflict_blocks(self):
        row = {"transaction_id": "7", "ray_type": "EXTRA_MARGIN", "ray_id": "12",
               "sign": "-1", "alpha": "0.5", "converged": "1",
               "terminal_map_T_lidar_xyz_q_xyzw": "0;0;0;0;0;0;1",
               "terminal_objective": "12", "terminal_fitness": "0.1",
               "terminal_iterations": "3", "seed_map_T_lidar_xyz_q_xyzw": "0;0;0;0;0;0;1"}
        conflict = dict(row, terminal_objective="13")
        with self.assertRaisesRegex(selector.SelectionBlocked, "conflicting_raw_endpoint_key"):
            selector.build_probe_index([row, conflict])

    def test_inside_and_outside_endpoint_lineage_is_checked(self):
        pose = "1;2;3;0;0;0;1"
        boundary = {
            "transaction_id": "7", "ray_type": "PRINCIPAL", "ray_id": "4", "sign": "-1",
            "alpha_same": "0.25", "alpha_diff": "0.5",
            "inside_endpoint_source": "FROZEN_PROBE", "outside_endpoint_source": "FROZEN_PROBE",
            "inside_terminal_pose": pose, "outside_terminal_pose": pose,
            "inside_converged": "true", "outside_converged": "true",
            "outside_seed_pose": pose,
        }
        def probe(alpha, terminal=pose, seed=pose):
            return {"transaction_id": "7", "ray_type": "PRINCIPAL", "ray_id": "4",
                    "sign": "-1", "alpha": alpha, "converged": "true",
                    "terminal_map_T_lidar_xyz_q_xyzw": terminal,
                    "seed_map_T_lidar_xyz_q_xyzw": seed,
                    "terminal_objective": "10", "terminal_fitness": "0.1",
                    "terminal_iterations": "4"}
        indexed = selector.build_probe_index([probe("0.25"), probe("0.5")])
        selector.verify_endpoint(boundary, "inside", indexed)
        selector.verify_endpoint(boundary, "outside", indexed)

        mutated = dict(boundary, inside_terminal_pose="1.1;2;3;0;0;0;1")
        with self.assertRaisesRegex(selector.SelectionBlocked, "boundary_terminal_pose_mismatch"):
            selector.verify_endpoint(mutated, "inside", indexed)
        mutated = dict(boundary, outside_seed_pose="1.1;2;3;0;0;0;1")
        with self.assertRaisesRegex(selector.SelectionBlocked, "boundary_outside_seed_pose_mismatch"):
            selector.verify_endpoint(mutated, "outside", indexed)
        mutated = dict(boundary, inside_endpoint_source="RECOMPUTED")
        with self.assertRaisesRegex(selector.SelectionBlocked, "non_frozen_endpoint_source"):
            selector.verify_endpoint(mutated, "inside", indexed)

    def test_group_selection_is_distinct_stable_and_uses_mismatch_rays(self):
        rows = [candidate(str(tx), q) for tx, q in
                ((8, 8.0), (2, 10.0), (1, 10.0), (3, 9.0), (4, 1.0), (5, 0.5),
                 (6, 0.4), (7, 0.3), (9, 2.0))]
        exceptions = []
        for tx, count in ((9, 4), (7, 4), (6, 2)):
            for ray in range(count):
                exceptions.append({"case_type": "EXIT_PREDICTED_FINITE_ACTUAL_CENSORED",
                                   "local_support": "STRICT_LOCAL_SUPPORT",
                                   "transaction_id": str(tx), "frame_id": f"F{tx}",
                                   "direction_id": f"D{ray}", "ray_id": str(ray), "sign": "1"})
        # Non-first-exit and contaminated rows do not count toward C.
        exceptions += [
            {"case_type": "PRIMARY_ACCEPTANCE_DISAGREEMENT", "local_support": "STRICT_LOCAL_SUPPORT",
             "transaction_id": "6", "frame_id": "F6", "direction_id": "X", "ray_id": "9", "sign": "1"},
            {"case_type": "EXIT_PREDICTED_FINITE_ACTUAL_CENSORED", "local_support": "LOCAL_SUPPORT_CONTAMINATED",
             "transaction_id": "9", "frame_id": "F9", "direction_id": "Y", "ray_id": "9", "sign": "1"},
        ]
        selected, counts = selector.select_groups(rows, exceptions)
        groups = {group: [row["transaction_id"] for row in selected
                          if row["selection_group"] == group]
                  for group in ("A_HIGH_JUMP", "B_LOW_JUMP_CONTROL", "C_FIRST_EXIT_MISMATCH")}
        self.assertEqual(groups["A_HIGH_JUMP"], ["1", "2", "3", "8"])
        self.assertEqual(groups["B_LOW_JUMP_CONTROL"], ["7", "6"])
        self.assertEqual(groups["C_FIRST_EXIT_MISMATCH"], ["9"])
        self.assertEqual(counts["9"], 4)
        self.assertEqual(len({row["frame_id"] for row in selected}), len(selected))

    def test_dense_support_and_dense_manifest_crosscheck(self):
        boundary = {"margin_source": "DENSE", "transaction_id": "1", "frame_id": "F1",
                    "alpha_diff": "0.5", "terminal_geometry_available": "True",
                    "alpha_same": "0.25", "ray_type": "PRINCIPAL", "ray_id": "2",
                    "sign": "1", "terminal_jump_translation_m": "0.1",
                    "terminal_jump_rotation_deg": "0.2"}
        dense = {"transaction_id": "1", "frame_id": "F1", "dense_censored": "false",
                 "dense_boundary_ray": "2", "dense_sign": "1",
                 "dense_boundary_interval_low": "0.25", "dense_boundary_interval_high": "0.5"}
        crosscheck = {"transaction_id": "1", "frame_id": "F1", "ray_type": "PRINCIPAL",
                      "ray_id": "2", "sign": "1", "crosscheck_status": "FINITE_WINNER_MATCH"}
        support = {"transaction_id": "1", "frame_id": "F1",
                   "strict_local_support": "STRICT_LOCAL_SUPPORT"}
        self.assertEqual(len(selector._finite_dense_rows([boundary], [dense], [support], [crosscheck])), 1)
        support["strict_local_support"] = "LOCAL_SUPPORT_CONTAMINATED"
        self.assertEqual(selector._finite_dense_rows([boundary], [dense], [support], [crosscheck]), [])

    def test_frozen_selection_readback_checks_identity_and_raw_pose_tokens(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "selected.csv"
            manifest_path = root / "manifest.json"
            integrity_path = root / "input_integrity.sha256"
            row = {field: "" for field in selector.selection_columns()}
            row.update({"frame_id": "F1", "transaction_id": "1", "time_s": "0.5",
                        "selection_group": "A_HIGH_JUMP", "selection_q": "1.25",
                        "ray_type": "PRINCIPAL", "ray_id": "2", "sign": "1",
                        "alpha_inside": "0.25", "alpha_outside": "0.5",
                        "jump_t_m": "0.1", "jump_r_deg": "0.2"})
            for field in (f"{prefix}_{axis}" for prefix in
                          ("seed_inside", "seed_outside", "terminal_inside", "terminal_outside")
                          for axis in selector.POSE_FIELDS):
                row[field] = "0"
            row["seed_inside_qw"] = "1"
            columns = selector.selection_columns()
            def write_selected(saved_row):
                with path.open("w", newline="", encoding="utf-8") as stream:
                    writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n")
                    writer.writeheader()
                    writer.writerow(saved_row)

            expected_manifest = {"selected_cases": [{
                "frame_id": "F1", "transaction_id": "1", "group": "A_HIGH_JUMP"}],
                "input_hashes": {"frozen.csv": "abc123"}}
            expected_integrity = "abc123  frozen.csv\n"
            write_selected(row)
            manifest_path.write_text(json.dumps(expected_manifest))
            integrity_path.write_text(expected_integrity)
            selector.verify_frozen_selection(path, manifest_path, integrity_path,
                                             [row], expected_manifest, expected_integrity)

            for field, mutated_value in (("selection_q", "99"), ("jump_t_m", "9"),
                                         ("terminal_inside_x", "1")):
                mutated = dict(row, **{field: mutated_value})
                write_selected(mutated)
                with self.assertRaisesRegex(selector.SelectionBlocked, "selected_cases_readback_mismatch"):
                    selector.verify_frozen_selection(path, manifest_path, integrity_path,
                                                     [row], expected_manifest, expected_integrity)
            write_selected(row)

            manifest_path.write_text(json.dumps(dict(expected_manifest,
                                                      input_hashes={"frozen.csv": "tampered"})))
            with self.assertRaisesRegex(selector.SelectionBlocked, "selection_manifest_readback_content_mismatch"):
                selector.verify_frozen_selection(path, manifest_path, integrity_path,
                                                 [row], expected_manifest, expected_integrity)
            manifest_path.write_text(json.dumps(expected_manifest))

            integrity_path.write_text("tampered  frozen.csv\n")
            with self.assertRaisesRegex(selector.SelectionBlocked, "input_integrity_readback_content_mismatch"):
                selector.verify_frozen_selection(path, manifest_path, integrity_path,
                                                 [row], expected_manifest, expected_integrity)


if __name__ == "__main__":
    unittest.main()
