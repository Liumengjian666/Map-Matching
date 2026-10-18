#!/usr/bin/env python3
"""One bounded public metadata request; preserve response bytes and HTTP evidence."""
import hashlib
import json
from pathlib import Path
import sys
from urllib.parse import urlparse
import requests


def main(url, output):
    destination = Path(output).resolve()
    allowed = {'api.github.com', 'raw.githubusercontent.com', 'thisparticle.github.io', 'drive.google.com'}
    if urlparse(url).scheme != 'https' or urlparse(url).hostname not in allowed:
        raise ValueError('official HTTPS metadata endpoint required')
    if not str(destination).startswith('/home/jian/') or destination.exists():
        raise ValueError('new internal destination required')
    receipt = destination.with_suffix(destination.suffix + '.http.json')
    if receipt.exists():
        raise FileExistsError(receipt)
    destination.parent.mkdir(parents=True, exist_ok=True)
    evidence = {'requested_url': url, 'attempt': 1, 'max_bytes': 8 * 1024 * 1024}
    try:
        with requests.get(url, timeout=(10, 30), stream=True) as response:
            evidence.update({'status': response.status_code, 'final_url': response.url,
                             'headers': {k: response.headers[k] for k in
                              ('Content-Type', 'Content-Length', 'Content-Range') if k in response.headers}})
            payload = bytearray()
            for block in response.iter_content(65536):
                if len(payload) + len(block) > evidence['max_bytes']:
                    raise ValueError('metadata size cap exceeded')
                payload.extend(block)
            with destination.open('xb') as stream:
                stream.write(payload)
            evidence.update({'bytes': len(payload), 'sha256': hashlib.sha256(payload).hexdigest()})
    except (requests.RequestException, ValueError) as error:
        evidence['error'] = str(error)
    with receipt.open('x') as stream:
        json.dump(evidence, stream, indent=2)
    print(json.dumps(evidence, indent=2))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
