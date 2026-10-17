import hashlib
import sys
import tempfile
import unittest
from pathlib import Path
import csv

sys.path.insert(0, str(Path(__file__).parent))
import mirror_inputs
import audit_partial_p59_mirror


class MirrorInputsTest(unittest.TestCase):
    def test_enumerates_unique_canonical_scans_and_ignores_duplicate_suffix(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "cloud_1_000000000.pcd").write_bytes(b"a")
            (root / "cloud_1_100000000.pcd").write_bytes(b"b")
            (root / "cloud_1_100000000(1).pcd").write_bytes(b"b")
            scans, ignored = mirror_inputs.enumerate_scans(root, expected_count=2)
            self.assertEqual([row[0] for row in scans], [1_000_000_000, 1_100_000_000])
            self.assertEqual([name for name in ignored], ["cloud_1_100000000(1).pcd"])

    def test_duplicate_canonical_timestamp_fails_closed(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "cloud_1_000000000.pcd").write_bytes(b"a")
            (root / "cloud_01_000000000.pcd").write_bytes(b"a")
            with self.assertRaisesRegex(RuntimeError, "duplicate canonical scan timestamp"):
                mirror_inputs.enumerate_scans(root, expected_count=1)

    def test_stream_copy_hashes_and_refuses_to_overwrite(self):
        payload = b"P59 internal-only fixture\n" * 4096
        expected = hashlib.sha256(payload).hexdigest()
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "source.bin"
            source.write_bytes(payload)
            item = mirror_inputs.InputFile(source, Path("nested/destination.bin"), expected)
            src_sha, dst_sha, copied, _ = mirror_inputs.copy_one(item, root / "out", 0, 0.0, 0)
            self.assertEqual((src_sha, dst_sha), (expected, expected))
            self.assertEqual(copied, len(payload))
            with self.assertRaisesRegex(RuntimeError, "refusing to overwrite"):
                mirror_inputs.copy_one(item, root / "out", 0, 0.0, 0)

    def test_hash_mismatch_preserves_partial_and_does_not_publish_final(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "source.bin"
            source.write_bytes(b"payload")
            item = mirror_inputs.InputFile(source, Path("out.bin"), "0" * 64)
            with self.assertRaisesRegex(RuntimeError, "source SHA mismatch"):
                mirror_inputs.copy_one(item, root / "dest", 0, 0.0, 0)
            self.assertTrue((root / "dest/out.bin.p59part").is_file())
            self.assertFalse((root / "dest/out.bin").exists())

    def test_retry_uses_distinct_partial_and_preserves_interrupted_prefix(self):
        payload = b"verified retry fixture\n" * 1024
        expected = hashlib.sha256(payload).hexdigest()
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "source.bin"
            source.write_bytes(payload)
            destination_root = root / "out"
            old_partial = destination_root / "nested/destination.bin.p59part"
            old_partial.parent.mkdir(parents=True)
            old_partial.write_bytes(b"preserve this interrupted prefix")
            item = mirror_inputs.InputFile(source, Path("nested/destination.bin"), expected)
            src_sha, dst_sha, copied, _ = mirror_inputs.copy_one(
                item, destination_root, 0, 0.0, 0, retry_partial=True)
            self.assertEqual((src_sha, dst_sha), (expected, expected))
            self.assertEqual(copied, len(payload))
            self.assertEqual(old_partial.read_bytes(), b"preserve this interrupted prefix")
            self.assertFalse((destination_root / "nested/destination.bin.p59part.retry1").exists())

    def test_resume_accepts_only_verified_three_scan_prefix_and_preserves_partial(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            original_data_root = mirror_inputs.DATA_ROOT
            mirror_inputs.DATA_ROOT = root / "data"
            try:
                mirror_inputs.DATA_ROOT.mkdir()
                inputs = []
                for index in range(4):
                    payload = (f"scan-{index + 1}\n".encode() if index < 3 else b"next-scan")
                    source = root / f"source-{index + 1}.pcd"
                    source.write_bytes(payload)
                    inputs.append(mirror_inputs.InputFile(
                        source, Path(f"scan/cloud_{index + 1}.pcd"),
                        hashlib.sha256(payload).hexdigest(), index + 1, index + 1))
                plan = {"frozen": "fixture"}
                (mirror_inputs.DATA_ROOT / "transfer_plan.json").write_text(
                    __import__("json").dumps(plan), encoding="utf-8")
                (mirror_inputs.DATA_ROOT / "transfer_manifest.jsonl").write_text("")
                (mirror_inputs.DATA_ROOT / "transfer_checkpoints.jsonl").write_text("")
                for item in inputs[:3]:
                    dest = mirror_inputs.DATA_ROOT / item.relative_path
                    dest.parent.mkdir(parents=True, exist_ok=True)
                    dest.write_bytes(item.source.read_bytes())
                partial = mirror_inputs.DATA_ROOT / (inputs[3].relative_path.as_posix() + ".p59part")
                partial.parent.mkdir(parents=True, exist_ok=True)
                partial.write_bytes(b"x" * mirror_inputs.CHUNK_BYTES)

                completed, copied_bytes = mirror_inputs.prepare_destination(inputs, plan)

                self.assertEqual(completed, {1, 2, 3})
                self.assertEqual(copied_bytes, sum(item.source.stat().st_size for item in inputs[:3]))
                self.assertEqual(partial.stat().st_size, mirror_inputs.CHUNK_BYTES)
                self.assertEqual(len((mirror_inputs.DATA_ROOT / "transfer_manifest.jsonl").read_text().splitlines()), 3)
            finally:
                mirror_inputs.DATA_ROOT = original_data_root

    def test_partial_prefix_audit_rejects_gap_and_accepts_hash_match(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            scans = root / "scans"
            scans.mkdir()
            payloads = [b"first", b"second"]
            references = []
            for index, payload in enumerate(payloads, 1):
                stamp = 1_000_000_000 + (index - 1) * 100_000_000
                name = f"cloud_{stamp // 1_000_000_000}_{stamp % 1_000_000_000:09d}.pcd"
                (scans / name).write_bytes(payload)
                references.append({"scan_index": str(index), "stamp_ns": str(stamp),
                                   "size_bytes": str(len(payload)),
                                   "sha256": hashlib.sha256(payload).hexdigest()})
            manifest = root / "p46.csv"
            with manifest.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(references[0]))
                writer.writeheader()
                writer.writerows(references)
            rows = audit_partial_p59_mirror.audit(scans, manifest)
            self.assertEqual([row["frame_index"] for row in rows], ["1", "2"])
            self.assertTrue(all(row["match"] == "true" for row in rows))

            (scans / "cloud_1_300000000.pcd").write_bytes(b"gap")
            with self.assertRaisesRegex(ValueError, "not contiguous"):
                audit_partial_p59_mirror.audit(scans, manifest)


if __name__ == "__main__":
    unittest.main()
