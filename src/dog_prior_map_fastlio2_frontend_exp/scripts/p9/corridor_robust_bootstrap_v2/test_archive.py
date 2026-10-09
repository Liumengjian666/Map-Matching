"""Post-run audit fixtures; no input extraction, GICP or NDT."""
import pathlib
import tempfile
import unittest
from archive_v2 import check_csv, normalize_snapshot


class ArchiveTests(unittest.TestCase):
    def fixture(self, content):
        root = tempfile.TemporaryDirectory(prefix="p9_v2_archive_test.")
        self.addCleanup(root.cleanup)
        path = pathlib.Path(root.name) / "data.csv"
        path.write_text(content)
        return path

    def test_missing_metrics_are_not_fake_zero(self):
        check_csv(self.fixture("status,value\nNOT_RUN,\nPASS,0.2\n"))

    def test_reject_ragged_csv(self):
        with self.assertRaises(RuntimeError):
            check_csv(self.fixture("a,b\n1\n"))

    def test_reject_nonfinite_and_duplicate_columns(self):
        for content in ("a,b\nPASS,nan\n", "a,b\nPASS,inf\n", "a,a\n1,2\n"):
            with self.subTest(content=content), self.assertRaises(RuntimeError):
                check_csv(self.fixture(content))

    def test_generated_snapshot_hygiene_keeps_values_and_final_newline(self):
        self.assertEqual(normalize_snapshot("# comment  \nvalue=x\n\n"), "# comment\nvalue=x\n")
        self.assertEqual(normalize_snapshot(normalize_snapshot("v=x\n\n")), "v=x\n")


if __name__ == "__main__":
    unittest.main()
