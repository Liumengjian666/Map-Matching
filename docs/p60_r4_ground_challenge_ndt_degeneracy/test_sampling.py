import unittest
import numpy as np
from prepare_inputs import uniform_indices


class SamplingTest(unittest.TestCase):
    def test_uniform_100_unique_previous_exists(self):
        indices = uniform_indices(np.arange(974, dtype=np.int64) * 100000000)
        self.assertEqual((len(indices), len(set(indices)), indices[0], indices[-1]), (100, 100, 2, 974))

    def test_relative_clock_prevents_epoch_roundoff(self):
        relative = np.arange(974, dtype=np.int64) * 100000000
        self.assertEqual(uniform_indices(relative), uniform_indices(relative + 1658917771606140375))

    def test_bad_times_rejected(self):
        for values in [np.arange(10), np.zeros(974), -np.arange(974)]:
            with self.assertRaises(ValueError):
                uniform_indices(values)


if __name__ == '__main__':
    unittest.main()
