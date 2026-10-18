#!/usr/bin/env python3
"""Decode the public folder page's own file list; no downloads or API bypass."""
import json
from pathlib import Path
import re
import sys


def listing(text):
    match = re.search(r"window\['_DRIVE_ivd'\] = '((?:\\.|[^'])*)';", text)
    if not match:
        raise ValueError('no public folder metadata; stop, do not infer children')
    # Decode JavaScript hex escapes only, then let JSON parse its own escapes.
    decoded = re.sub(r'\\x([0-9a-fA-F]{2})', lambda m: chr(int(m[1], 16)), match[1])
    data = json.loads(decoded)
    result = []
    for item in data[0] or []:
        if len(item) < 4 or not all(isinstance(item[k], str) for k in (0, 2, 3)):
            raise ValueError('unrecognized public metadata item')
        result.append({'id': item[0], 'name': item[2], 'mime': item[3],
                       'raw_metadata': item})
    return result


def main(source, destination):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if not all(str(path).startswith('/home/jian/') for path in (source, destination)):
        raise ValueError('internal metadata files only')
    rows = listing(source.read_text())
    with destination.open('x') as stream:
        json.dump(rows, stream, indent=2, ensure_ascii=False)
    for row in rows:
        print(row['id'], row['mime'], row['name'])


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
