#!/usr/bin/env python3
"""Regression tests for reviewer-found gates; no ROS, ICP or dataset access."""
from pathlib import Path
import shutil
import tempfile
import numpy as np
from run_once import result_is_legal, sha, validate_input_manifest, validate_extraction_sources


def result(T):
    return {'result' + str(i) + str(j): str(T[i, j]) for i in range(4) for j in range(4)}


def main():
    assert result_is_legal(result(np.eye(4)))
    for wrong in (float('nan'), -1., 1.1):
        T = np.eye(4)
        T[0, 0] = wrong
        assert not result_is_legal(result(T)), 'Illegal final matrix accepted'
    with tempfile.TemporaryDirectory(prefix='p32-transfer-guard-') as name:
        sources = Path(name) / 'scripts/p32_init_transfer'
        sources.mkdir(parents=True)
        prepare = sources / 'prepare_input.py'
        decoder = sources.parent / 'p32_corridor02/prepare_probe.py'
        decoder.parent.mkdir()
        prepare.write_text('# fixture only\n')
        decoder.write_text('# decoder fixture only\n')
        provenance = dict(executed_sources_sha256={str(path): sha(path) for path in (prepare, decoder)})
        validate_extraction_sources(sources, provenance)
        decoder.write_text('# changed decoder fixture\n')
        try:
            validate_extraction_sources(sources, provenance)
        except ValueError:
            pass
        else:
            raise AssertionError('Changed extraction decoder accepted')
        original = Path(name) / 'A'
        original.mkdir()
        (original / 'scans.txt').write_text('1 2 1 scan.xyzt\n')
        (original / 'scan.xyzt').write_bytes(bytes(16))
        (original / 'imu.txt').write_text('real-test-input\n')
        (original / 'T_imu_lidar.f64').write_bytes(bytes(128))
        manifest = dict(files_sha256={str(path): sha(path) for path in original.iterdir()})
        validate_input_manifest(original, manifest)
        copied = Path(name) / 'B'
        shutil.copytree(original, copied)
        (copied / 'imu.txt').write_text('changed-test-input\n')
        try:
            validate_input_manifest(copied, manifest)
        except ValueError:
            pass
        else:
            raise AssertionError('A manifest validated B input')
        (original / 'imu.txt').write_text('changed-test-input\n')
        try:
            validate_input_manifest(original, manifest)
        except ValueError:
            pass
        else:
            raise AssertionError('Changed consumed input accepted')
    print('GUARD_REGRESSION_PASS final_matrix=PASS manifest_directory=PASS consumed_hash=PASS decoder_binding=PASS')


if __name__ == '__main__':
    main()
