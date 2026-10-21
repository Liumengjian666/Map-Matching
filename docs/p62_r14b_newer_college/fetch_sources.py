"""Fetch a fixed small set of official documentation/code, never execute it."""
import hashlib
import json
from pathlib import Path
import urllib.request

ROOT = Path(__file__).resolve().parent / 'sources'
SOCC = '557a98e39a163f0b369a1612db7fc69f8b5aae60'
FRONTIER = '0b039393334a7767e24a711bd05816bfc7f7485a'
OUSTER = 'fa9a53888b3ccd878e8b179c763363d8d3edfff9'
RAW = 'https://raw.githubusercontent.com/'
URLS = {
    'socc_DATASETS.md': RAW + f'josch14/socc_icp/{SOCC}/DATASETS.md',
    'socc_newer_college.py': RAW + f'josch14/socc_icp/{SOCC}/src/socc_icp/socc_icp/dataset/newer_college.py',
    'socc_run.py': RAW + f'josch14/socc_icp/{SOCC}/src/socc_icp/run/run_newer_college.py',
    'frontier_README.md': RAW + f'ori-drs/frontier_description/{FRONTIER}/README.md',
    'ouster_ros.cpp': RAW + f'ori-drs/ouster_example/{OUSTER}/ouster_ros/src/ros.cpp',
    'ouster_lidar_scan.h': RAW + f'ori-drs/ouster_example/{OUSTER}/ouster_client/include/ouster/lidar_scan.h',
    'ground_truth.html': 'https://ori-drs.github.io/newer-college-dataset/ground-truth/',
    'platform.html': 'https://ori-drs.github.io/newer-college-dataset/stereo-cam/platform-stereo/',
    'calibration.html': 'https://ori-drs.github.io/newer-college-dataset/stereo-cam/calibration-stereo/',
    'stereo.html': 'https://ori-drs.github.io/newer-college-dataset/stereo-cam/',
}


def main():
    ROOT.mkdir(exist_ok=False)
    rows = []
    for name, url in URLS.items():
        row = {'name': name, 'url': url}
        try:
            with urllib.request.urlopen(url, timeout=30) as response:
                data = response.read(512 * 1024 + 1)
                if len(data) > 512 * 1024:
                    raise ValueError('Small source exceeded 512KiB cap')
                row.update(status=response.status, final_url=response.url)
            with (ROOT / name).open('xb') as stream:
                stream.write(data)
            row.update(bytes=len(data), sha256=hashlib.sha256(data).hexdigest())
        except Exception as exc:
            row['error'] = f'{type(exc).__name__}: {exc}'
        rows.append(row)
    with (ROOT / 'manifest.json').open('x') as stream:
        json.dump(rows, stream, indent=2)
    print(json.dumps(rows, indent=2))


if __name__ == '__main__':
    main()
