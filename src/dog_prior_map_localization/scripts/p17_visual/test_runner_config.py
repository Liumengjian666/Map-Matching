"""P18 map-path-only adaptation checks; starts no ROS nodes and reads no GT."""
import copy
import hashlib
import unittest
from pathlib import Path
import yaml
import run

REPO = Path(__file__).resolve().parents[4]
CONFIG = REPO/'src/dog_prior_map_localization/config'


class ConfigTests(unittest.TestCase):
    def test_default_remains_original(self):
        self.assertEqual(run.resolve_dataset_config(REPO),
                         CONFIG/'p10_corridor01_prospective_causal.yaml')

    def test_explicit_override(self):
        self.assertEqual(run.resolve_dataset_config(REPO, CONFIG/'p18_corridor01_ssd_map.yaml'),
                         CONFIG/'p18_corridor01_ssd_map.yaml')

    def test_missing_override_is_not_a_fallback(self):
        with self.assertRaisesRegex(ValueError, 'DATASET_CONFIG_NOT_FOUND'):
            run.resolve_dataset_config(REPO, CONFIG/'p18_missing_config.yaml')

    def test_only_map_path_changes_and_map_hash_is_exact(self):
        original = yaml.safe_load((CONFIG/'p10_corridor01_prospective_causal.yaml').read_text())
        override = yaml.safe_load((CONFIG/'p18_corridor01_ssd_map.yaml').read_text())
        changed = copy.deepcopy(override)
        changed['map']['pcd_fallback_path'] = original['map']['pcd_fallback_path']
        self.assertEqual(original, changed)
        target = Path(override['map']['pcd_fallback_path'])
        self.assertEqual(target.stat().st_size, 4058751)
        self.assertEqual(hashlib.sha256(target.read_bytes()).hexdigest(),
                         '103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f')


if __name__ == '__main__': unittest.main()
