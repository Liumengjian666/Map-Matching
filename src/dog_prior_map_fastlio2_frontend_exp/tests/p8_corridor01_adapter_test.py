import csv
import hashlib
import importlib.util
import tempfile
import unittest
from pathlib import Path

import numpy as np


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "p8"
    / "convert_corridor01_p7_timed.py"
)
SPEC = importlib.util.spec_from_file_location("p8_corridor01_adapter", SCRIPT)
ADAPTER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ADAPTER)


class Corridor01AdapterTest(unittest.TestCase):
    @staticmethod
    def _write_source_bundle(source_dir, catalog_point_count=2):
        source_dir.mkdir(parents=True, exist_ok=True)
        points = np.zeros(2, dtype=ADAPTER.SOURCE_DTYPE)
        points["x"] = [1.0, 2.0]
        points["y"] = [-1.0, -2.0]
        points["z"] = [0.1, 0.2]
        points["intensity"] = [4.0, 5.0]
        points["stamp_ns"] = [1_000, 1_025]
        (source_dir / ADAPTER.SOURCE_POINTS).write_bytes(points.tobytes())

        with (source_dir / ADAPTER.SOURCE_CATALOG).open("w", newline="") as stream:
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerow(
                ["transaction_id", "scan_start_ns", "scan_end_ns", "byte_offset", "point_count", "provenance"]
            )
            writer.writerow([1, 1_000, 1_100, 0, catalog_point_count, "RAW_TIMED_SENSOR"])
        with (source_dir / ADAPTER.SOURCE_FILTER).open("w", newline="") as stream:
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerow(["transaction_id", "stamp_ns"])
            writer.writerow([1, 1_100])

        manifest = {
            "dataset": "SuperLoc Corridor01",
            "provenance": "RAW_TIMED_SENSOR",
            "raw_lidar_topic": "/velodyne_packets",
            "raw_lidar_message_type": "velodyne_msgs/VelodyneScan",
            "sensor_frame_id": "cmu_rc2_velodyne",
            "point_time_unit": "absolute_sensor_nanoseconds",
            "point_time_reference": "packet.stamp; packet-local scan_start_time",
            "point_time_conversion_formula": "packet.stamp.toNSec()+llround(unpack(packet,scan_start_time=packet.stamp).time*1e9)",
            "scan_start_semantic": "minimum timestamp of decoded valid range returns",
            "scan_end_semantic": "last_packet.stamp_ns + 1306368 ns",
            "GT_USED": "false",
            "LEGACY_STATE_USED": "false",
            "WINDOW_STATE_USED": "false",
            "DESKEW_PERFORMED": "false",
            "raw_timed_points_file": ADAPTER.SOURCE_POINTS,
            "raw_timed_points_sha256": ADAPTER.sha256(source_dir / ADAPTER.SOURCE_POINTS),
            "raw_timed_catalog_file": ADAPTER.SOURCE_CATALOG,
            "raw_timed_catalog_sha256": ADAPTER.sha256(source_dir / ADAPTER.SOURCE_CATALOG),
            "filter_scans_file": ADAPTER.SOURCE_FILTER,
            "filter_scans_sha256": ADAPTER.sha256(source_dir / ADAPTER.SOURCE_FILTER),
            "original_bag_sha256": "test-bag-sha256",
            "scan_count": "1",
            "point_count": "2",
        }
        with (source_dir / ADAPTER.SOURCE_MANIFEST).open("w") as stream:
            for key, value in manifest.items():
                stream.write(f"{key}={value}\n")

    def test_absolute_timestamps_become_order_preserving_relative_ns(self):
        points = np.zeros(3, dtype=ADAPTER.SOURCE_DTYPE)
        points["x"] = [1.0, 2.0, 3.0]
        points["y"] = [-1.0, -2.0, -3.0]
        points["z"] = [0.1, 0.2, 0.3]
        points["intensity"] = [4.0, 5.0, 6.0]
        points["stamp_ns"] = [1_000_000_050, 1_000_000_020, 1_000_000_100]

        packed = ADAPTER.pack_points(points, 1_000_000_000, 1_000_000_200)

        self.assertEqual(packed.dtype.itemsize, 16)
        np.testing.assert_array_equal(packed["x"], [1.0, 2.0, 3.0])
        np.testing.assert_array_equal(packed["offset_ns"], [50, 20, 100])

    def test_rejects_time_outside_catalog_interval(self):
        points = np.zeros(1, dtype=ADAPTER.SOURCE_DTYPE)
        points["stamp_ns"] = [900]
        with self.assertRaisesRegex(ADAPTER.AdapterError, "outside_catalog"):
            ADAPTER.pack_points(points, 1_000, 2_000)

    def test_rejects_offset_that_cannot_fit_p7_record(self):
        points = np.zeros(1, dtype=ADAPTER.SOURCE_DTYPE)
        points["stamp_ns"] = [2**32 + 1]
        with self.assertRaisesRegex(ADAPTER.AdapterError, "exceeds_uint32"):
            ADAPTER.pack_points(points, 0, 2**32 + 1)

    def test_rejects_nonfinite_xyz(self):
        points = np.zeros(1, dtype=ADAPTER.SOURCE_DTYPE)
        points["x"] = [np.inf]
        points["stamp_ns"] = [1]
        with self.assertRaisesRegex(ADAPTER.AdapterError, "nonfinite_source_point"):
            ADAPTER.pack_points(points, 0, 2)

    def test_rejects_catalog_that_omits_binary_tail(self):
        with tempfile.TemporaryDirectory() as temporary:
            source_dir = Path(temporary) / "source"
            self._write_source_bundle(source_dir, catalog_point_count=1)
            with self.assertRaisesRegex(
                ADAPTER.AdapterError, "source_catalog_does_not_cover_point_binary"
            ):
                ADAPTER.validate_source(source_dir)

    def test_convert_emits_expected_p7_records_and_manifest(self):
        with tempfile.TemporaryDirectory() as temporary:
            source_dir = Path(temporary) / "source"
            output_dir = Path(temporary) / "output"
            self._write_source_bundle(source_dir)

            output_manifest = ADAPTER.convert(source_dir, output_dir)

            converted = np.fromfile(output_dir / "p7_timed_points.bin", dtype=ADAPTER.P7_DTYPE)
            np.testing.assert_array_equal(converted["x"], [1.0, 2.0])
            np.testing.assert_array_equal(converted["offset_ns"], [0, 25])
            self.assertEqual(output_manifest["point_count"], "2")
            self.assertEqual(
                output_manifest["output_points_sha256"],
                ADAPTER.sha256(output_dir / "p7_timed_points.bin"),
            )

    def test_convert_rejects_source_changed_after_manifest_validation(self):
        with tempfile.TemporaryDirectory() as temporary:
            source_dir = Path(temporary) / "source"
            output_dir = Path(temporary) / "output"
            self._write_source_bundle(source_dir)
            original_validate = ADAPTER.validate_source

            def validate_then_mutate(path):
                result = original_validate(path)
                points_path = Path(path) / ADAPTER.SOURCE_POINTS
                points = np.fromfile(points_path, dtype=ADAPTER.SOURCE_DTYPE)
                points[0]["x"] = 9.0
                points.tofile(points_path)
                return result

            ADAPTER.validate_source = validate_then_mutate
            try:
                with self.assertRaisesRegex(
                    ADAPTER.AdapterError, "source_points_sha256_mismatch_during_conversion"
                ):
                    ADAPTER.convert(source_dir, output_dir)
            finally:
                ADAPTER.validate_source = original_validate

    def test_convert_refuses_nonempty_output_directory(self):
        with tempfile.TemporaryDirectory() as temporary:
            source_dir = Path(temporary) / "source"
            output_dir = Path(temporary) / "output"
            self._write_source_bundle(source_dir)
            output_dir.mkdir()
            (output_dir / "keep.txt").write_text("do not overwrite")

            with self.assertRaisesRegex(ADAPTER.AdapterError, "refusing_nonempty_output_directory"):
                ADAPTER.convert(source_dir, output_dir)
            self.assertEqual((output_dir / "keep.txt").read_text(), "do not overwrite")


if __name__ == "__main__":
    unittest.main()
