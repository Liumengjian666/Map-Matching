import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import run

BINARY = Path(sys.argv.pop(1)) if len(sys.argv) > 1 else None

class ContractTests(unittest.TestCase):
    def test_streamed_sha256(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'fixture'
            data = b'causal' * 300000
            path.write_bytes(data)
            self.assertEqual(run.sha(path), hashlib.sha256(data).hexdigest())

    def test_relative_and_absolute_source_paths(self):
        relative = Path(os.path.relpath(Path(__file__).parent, Path.cwd()))
        self.assertEqual(run.source_hashes(relative), run.source_hashes(Path(__file__).parent.resolve()))
        self.assertTrue(all(not Path(path).is_absolute() for path in run.source_hashes(relative)))

    def test_failure_receipt_without_data_or_ndt(self):
        self.assertIsNotNone(BINARY)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            extrinsic = root / 'extrinsic.txt'
            extrinsic.write_text('1 0 0 0\n0 1 0 0\n0 0 1 0\n0 0 0 1\n')
            result = subprocess.run([str(BINARY), str(root / 'absent_input'),
                str(root / 'absent_map'), str(root), str(extrinsic)], capture_output=True,
                text=True, env=dict(os.environ, LD_LIBRARY_PATH='/lib/x86_64-linux-gnu'))
            self.assertEqual(result.returncode, 2)
            receipt = json.loads((root / 'execution_failure.json').read_text())
            self.assertEqual(receipt['attempted_align_calls'], 0)
            self.assertEqual(receipt['completed_frames_per_arm'], 0)
            self.assertFalse(receipt['GT_LOADED'])
            self.assertFalse(receipt['current_state_committed'])
            self.assertFalse(receipt['archival_acceptance'])
            self.assertFalse((root / 'frames.csv').exists())

    def test_canonical_algorithms_not_reimplemented(self):
        source = (Path(__file__).parent / 'benchmark.cpp').read_text()
        self.assertIn('registration.align(', source)
        self.assertIn('registration.weakRefinement(', source)
        self.assertIn('p::advanceCoupledAnchor(', source)
        self.assertIn('p::settleWeakRefinementAnchor(', source)
        self.assertNotIn('applyPoseMeasurement', source)
        self.assertNotIn('buildDirectionalCovariance', source)
        self.assertNotIn('getGroundTruth', source)

if __name__ == '__main__':
    unittest.main()
