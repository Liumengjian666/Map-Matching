import csv
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from prepare_basin import identity, write_csv
from run_frozen import bind_preflight


class RunGuardTests(unittest.TestCase):
    def test_preflight_rejects_changed_binary_or_manifest(self):
        with TemporaryDirectory(prefix='p61_r3_test_') as directory:
            binary, manifest = Path(directory)/'binary', Path(directory)/'manifest'
            binary.write_bytes(b'original'); manifest.write_bytes(b'fixed')
            receipt = {'binary': {str(binary): identity(binary)}, 'preregistration': identity(manifest)}
            bind_preflight(receipt, manifest, binary)
            binary.write_bytes(b'different')
            with self.assertRaises(ValueError): bind_preflight(receipt, manifest, binary)
            binary.write_bytes(b'original'); manifest.write_bytes(b'new-selection')
            with self.assertRaises(ValueError): bind_preflight(receipt, manifest, binary)

    def test_empty_failure_table_has_explicit_header(self):
        with TemporaryDirectory(prefix='p61_r3_test_') as directory:
            output = Path(directory)/'clusters.csv'
            write_csv(output, [], fields=['cluster_id', 'members'])
            with output.open() as stream:
                reader = csv.DictReader(stream)
                self.assertEqual(reader.fieldnames, ['cluster_id', 'members'])
                self.assertEqual(list(reader), [])


if __name__ == '__main__': unittest.main()
