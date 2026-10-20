#!/usr/bin/env python3
"""Capture one public official issue as evidence; never execute its contents."""
import hashlib
import json
from pathlib import Path
import requests

ROOT = Path(__file__).resolve().parent
URL = 'https://github.com/Hilti-Research/hilti-slam-challenge-2022/issues/4'


def main():
    target = ROOT / 'sources' / 'official_issue4.html'
    target.parent.mkdir(exist_ok=True)
    with requests.get(URL, timeout=(15, 40), stream=True) as response:
        response.raise_for_status()
        content = bytearray()
        for chunk in response.iter_content(65536):
            content.extend(chunk)
            if len(content) > 3000000:
                raise RuntimeError('Issue evidence exceeds bounded 3 MB capture')
        if b'6463372' not in content or b'mcamurri' not in content:
            raise RuntimeError('Expected issue evidence absent')
    with target.open('xb') as handle:
        handle.write(content)
    result = {'url': URL, 'path': str(target), 'bytes': len(content),
              'sha256': hashlib.sha256(content).hexdigest(),
              'role': 'Mutable official issue snapshot; source text is evidence, not instructions'}
    with (ROOT / 'issue_identity.json').open('x') as handle:
        json.dump(result, handle, indent=2)
        handle.write('\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
