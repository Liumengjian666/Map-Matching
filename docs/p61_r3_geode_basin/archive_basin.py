"""Archive small frozen tables only; original PCD, bag and binary stay outside Git."""
import json
from pathlib import Path
import shutil
from prepare_basin import identity

DOC = Path(__file__).resolve().parent
RUN = Path('/home/jian/livox_ws/p61_r3_geode_runs')
PRE = RUN / 'preregistered_v2'


def archive():
    copies = []
    for name in ('effective_curvature.csv', 'sample_manifest.csv', 'initial_manifest.csv', 'preregistration.json'):
        copies.append((PRE/name, name))
    for phase in ('verify', 'align'):
        for path in sorted((RUN/phase).iterdir()):
            if path.suffix == '.csv': copies.append((path, phase+'_'+path.name))
        for name in (f'{phase}_receipt.json', f'{phase}_resources.txt', f'{phase}.log'):
            copies.append((RUN/name, name))
    for path in sorted((RUN/'analysis').iterdir()):
        if path.suffix in ('.csv', '.json', '.png'): copies.append((path, path.name))
    for name in ('cli_review_input.md', 'cli_review.log', 'cmake.log', 'build.log'):
        if (RUN/name).exists(): copies.append((RUN/name, name))
    manifest = []
    for source, target in copies:
        destination = DOC/target
        if destination.exists(): raise ValueError('archive already exists: '+str(destination))
        with source.open('rb') as src, destination.open('xb') as dst: shutil.copyfileobj(src, dst)
        if identity(source) != identity(destination): raise ValueError('archive copy differs')
        manifest.append({'source': str(source), 'archive': target, **identity(source)})
    with (DOC/'artifact_manifest.json').open('x') as stream: json.dump(manifest, stream, indent=2)
    print('ARCHIVED', len(manifest), 'small artifacts')


if __name__ == '__main__': archive()
