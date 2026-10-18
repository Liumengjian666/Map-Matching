import unittest
from statistics_core import ARMS, classify, axis_similarity, parse_resource


def sample(ratio=.01, tr=.4):
    return dict(spd='1', comparable='1', ratio6=str(ratio), ratio_translation=str(tr),
                translation_weak0='1', translation_weak1='0', translation_weak2='0')


class StatisticsTest(unittest.TestCase):
    def test_resource_log_is_parsed_not_inferred(self):
        text = 'User time (seconds): 2.14\nSystem time (seconds): 0.59\nElapsed (wall clock) time (h:mm:ss or m:ss): 1:02.74\nMaximum resident set size (kbytes): 29552'
        self.assertEqual(parse_resource(text)['wall_sec'], 62.74)
        with self.assertRaises(ValueError):
            parse_resource('')

    def test_insufficient_cannot_pass(self):
        self.assertEqual(classify({a: {k: sample() for k in range(79)} for a in ARMS})['final_result'], 'GEOMETRY_SCREEN_INSUFFICIENT')

    def test_direction_loss_alone_not_severe_degeneracy(self):
        p = {a: {k: sample(.002 if a == 'FOV_90' else .01) for k in range(100)} for a in ARMS}
        self.assertEqual(classify(p)['final_result'], 'DEGENERACY_NOT_DEMONSTRATED')

    def test_native_label_requires_consistent_run(self):
        p = {a: {k: sample() for k in range(100)} for a in ARMS}
        for k in range(3):
            p[ARMS[0]][k] = sample(.0001, .001)
        self.assertEqual(classify(p)['final_result'], 'NATIVE_DEGENERACY_EVIDENCE_OBSERVED')

    def test_induced_requires_fov_specific_weak_labels(self):
        p = {a: {k: sample(.0001, .001) if a == ARMS[1] else sample() for k in range(100)} for a in ARMS}
        self.assertEqual(classify(p)['final_result'], 'FOV_INDUCED_DEGENERACY_OBSERVED')

    def test_eigenvector_sign_is_not_a_direction_change(self):
        a, b = sample(), sample(); b['translation_weak0'] = '-1'
        self.assertAlmostEqual(axis_similarity(a, b), 1)


if __name__ == '__main__':
    unittest.main()
