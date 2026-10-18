"""Deterministic complete-link pose clustering. No score-based candidate selection."""
import numpy as np


def rotation(R):
    # Analysis-only projection removes float carrier nonorthogonality; stored poses are unchanged.
    U, _, V = np.linalg.svd(R)
    if np.linalg.det(U @ V) < 0: raise ValueError('improper rotation')
    return U @ V


def distance(a, b):
    relative = rotation(a[:3, :3]).T @ rotation(b[:3, :3])
    skew = np.array([relative[2, 1]-relative[1, 2], relative[0, 2]-relative[2, 0], relative[1, 0]-relative[0, 1]])
    angle = np.arctan2(np.linalg.norm(skew) / 2, (np.trace(relative)-1) / 2)
    return float(np.linalg.norm(a[:3, 3]-b[:3, 3])), float(angle * 180 / np.pi)


def normalized(a, b):
    t, r = distance(a, b)
    return max(t/.1, r/1.)


def complete_link(poses):
    groups = [(k,) for k in sorted(poses)]
    while True:
        options = []
        for i, a in enumerate(groups):
            for j in range(i+1, len(groups)):
                b = groups[j]
                separation = max(normalized(poses[x], poses[y]) for x in a for y in b)
                if separation <= 1:
                    options.append((separation, tuple(sorted(a+b)), i, j))
        if not options: break
        _, merged, i, j = min(options)
        groups = [g for k, g in enumerate(groups) if k not in (i, j)] + [merged]
        groups.sort()
    return groups


def unique_initials(ids, initials):
    accepted = []
    for k in sorted(ids):
        if not any(distance(initials[k], initials[j])[0] <= 1e-6 and
                   distance(initials[k], initials[j])[1] <= 1e-4 for j in accepted):
            accepted.append(k)
    return accepted


def separated(a, b):
    t, r = distance(a, b)
    return t > .30 or r > 3.


def medoid(ids, poses):
    return min(ids, key=lambda k: (sum(normalized(poses[k], poses[j]) for j in ids), k))


def competing_pair(groups, poses, nominal):
    """Groups already require >=2 unique stationary starts and nominal-near energies."""
    for i, a in enumerate(groups):
        for b in groups[i+1:]:
            if all(separated(poses[x], poses[y]) for x in a for y in b) and (
                all(separated(poses[x], nominal) for x in a) or
                all(separated(poses[x], nominal) for x in b)):
                return True
    return False
