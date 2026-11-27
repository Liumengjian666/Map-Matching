"""Small deterministic checks of common, visible offset and occluded ray evidence."""
import numpy as np

from geometry import COMMON, DISCRIMINATIVE, UNKNOWN, compute


def plane(z, half_width):
    x = np.arange(-half_width, half_width + 0.01, 0.025)
    xy = np.array(np.meshgrid(x, x)).reshape(2, -1).T
    return np.column_stack((xy, np.full(len(xy), z)))


def single_center_class(z_offset=0., occluder=False):
    source = np.asarray([[0., 0., 2.]])
    world = plane(2., .30)
    if occluder:
        world = np.concatenate((world, plane(1., .16)))
    pose = np.stack((np.eye(4), np.eye(4)))
    pose[1, 2, 3] = z_offset
    result, _ = compute(source, world, pose, np.repeat(pose[:, None], 26, axis=1))
    return result["classes"][0], result["observable"][0]


def main():
    same, support = single_center_class()
    assert np.all(same == COMMON) and support.all(), (same, support)
    separated, support = single_center_class(z_offset=.12)
    assert np.all(separated == DISCRIMINATIVE) and support.all(), (separated, support)
    obscured, support = single_center_class(occluder=True)
    assert np.all(obscured == UNKNOWN) and not support.any(), (obscured, support)
    print("Geometry QA PASS: common, 12cm separation, nearer occluder veto")


if __name__ == "__main__":
    main()
