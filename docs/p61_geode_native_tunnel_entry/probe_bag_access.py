#!/usr/bin/env python3
"""One anonymous official confirmation request, at most 64KiB body retained.

This is only access evidence, never a complete bag or a replayable sensor input.
"""
import hashlib
from html.parser import HTMLParser
import json
from pathlib import Path
import sys
from urllib.parse import urlparse
import requests

FILE_ID = '11Z4tmdEXGMuiLDKazd9MoALYJEbiGpE9'
LIMIT = 65536


class Confirmation(HTMLParser):
    def __init__(self):
        super().__init__(); self.action = None; self.values = {}; self.in_form = False

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if tag == 'form':
            if self.action is not None:
                raise ValueError('multiple confirmation forms')
            self.action = attrs.get('action'); self.in_form = True
        if tag == 'input' and self.in_form and attrs.get('name'):
            self.values[attrs['name']] = attrs.get('value', '')

    def handle_endtag(self, tag):
        if tag == 'form': self.in_form = False


def parse_confirmation(text):
    form = Confirmation(); form.feed(text)
    parsed = urlparse(form.action or '')
    if (parsed.scheme != 'https' or parsed.hostname != 'drive.usercontent.google.com'
            or parsed.path != '/download' or form.values.get('id') != FILE_ID
            or form.values.get('export') != 'download' or not form.values.get('confirm')):
        raise ValueError('unexpected official confirmation; do not send request')
    return form


def main(page, output_directory):
    page, output_directory = Path(page).resolve(), Path(output_directory).resolve()
    if not all(str(p).startswith('/home/jian/') for p in (page, output_directory)):
        raise ValueError('internal paths required')
    output_directory.mkdir(exist_ok=False)
    form = parse_confirmation(page.read_text())
    record = {'file_id': FILE_ID, 'attempts': 1, 'body_read_limit_bytes': LIMIT,
              'requested_range': 'bytes=0-65535', 'complete_bag': False,
              'authenticated': False, 'confirmation_source_sha256': hashlib.sha256(page.read_bytes()).hexdigest()}
    try:
        with requests.get(form.action, params=form.values, headers={'Range': 'bytes=0-65535'},
                          timeout=(10, 30), stream=True, allow_redirects=False) as response:
            record.update({'status': response.status_code, 'headers': {k: response.headers[k] for k in
                           ('Content-Type', 'Content-Length', 'Content-Range') if k in response.headers}})
            body = response.raw.read(LIMIT, decode_content=True)
            binary = body.startswith(b'#ROSBAG V2.0\n')
            filename = 'bag_prefix.bin' if binary else 'response_body.bin'
            with (output_directory / filename).open('xb') as stream:
                stream.write(body)
            record.update({'retained_bytes': len(body), 'sha256': hashlib.sha256(body).hexdigest(),
                           'rosbag_magic_pass': binary, 'body_filename': filename,
                           'quota_message': b'Quota exceeded' in body or b'Too many users' in body})
    except requests.RequestException as error:
        record['error'] = str(error)
    with (output_directory / 'access_receipt.json').open('x') as stream:
        json.dump(record, stream, indent=2)
    print(json.dumps(record, indent=2))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
