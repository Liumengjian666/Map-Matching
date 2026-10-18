#!/usr/bin/env python3
"""Copy only small, frozen R5 outputs into the new archive; never overwrite."""
from pathlib import Path
import shutil
import sys


def main(run):
    destination = Path(__file__).resolve().parent
    files = [run / name for name in ('input_freeze.json', 'run_code_freeze.json',
             'output_freeze.json', 'formal.log', 'resource.txt')]
    files += sorted((run / 'formal').glob('*.csv'))
    files += sorted((run / 'analysis').glob('*'))
    if len({path.name for path in files}) != len(files):
        raise ValueError('duplicate archive filename')
    if any((destination / path.name).exists() for path in files):
        raise FileExistsError('archive must not overwrite an existing artifact')
    for path in files:
        if not path.is_file() or not str(path.resolve()).startswith('/home/jian/'):
            raise ValueError('internal regular artifact required')
    for path in files:
        shutil.copyfile(path, destination / path.name)
    print('ARCHIVED', len(files), 'new artifacts')


if __name__ == '__main__':
    main(Path(sys.argv[1]).resolve())
