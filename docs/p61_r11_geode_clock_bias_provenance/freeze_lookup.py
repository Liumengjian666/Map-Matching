"""Additive metadata pin; this old ledger is a lookup aid, not sync proof."""
import json
from pathlib import Path
from check_assets import HERE, sha


if __name__ == '__main__':
    path = Path('/home/jian/geode/Shield_tunnel1_gamma/inspection/imu_records.csv')
    result = dict(status='PASS', assets=[dict(path=str(path), sha256=sha(path),
        size_bytes=path.stat().st_size,
        frozen_source='existing R2 traversal metadata; first R11 SHA snapshot; every selected record independently checked in frozen bag')])
    with (HERE/'lookup_inputs_frozen.json').open('x') as f:
        json.dump(result, f, indent=2); f.write('\n')
    print('Lookup snapshot', result)
