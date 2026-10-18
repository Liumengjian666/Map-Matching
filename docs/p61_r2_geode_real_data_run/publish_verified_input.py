#!/usr/bin/env python3
"""Publish a completed internal copy exclusively after two independent hashes."""
import hashlib
import json
import os
from pathlib import Path
import re
from storage_transfer import DEST, SIZE, SOURCE


def main():
    partial = DEST / 'Shield_tunnel1_gamma.bag.part'
    final = DEST / 'Shield_tunnel1_gamma.bag'
    receipt = DEST / 'input_freeze.json'
    if final.exists() or receipt.exists(): raise FileExistsError('do not overwrite verified input')
    for phase in ('copy', 'source_hash'):
        status = json.loads((DEST / f'storage_evidence/{phase}/receipt.json').read_text())
        if not status['success']: raise ValueError('storage phase failed')
    matches = re.findall(r'^SOURCE_SHA256 ([0-9a-f]{64})$', (DEST / 'storage_evidence/source_hash/worker.log').read_text(), re.M)
    if len(matches) != 1 or partial.stat().st_size != SIZE: raise ValueError('incomplete source identity')
    digest = hashlib.sha256()
    with partial.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''): digest.update(block)
    if digest.hexdigest() != matches[0]: raise ValueError('source/destination SHA256 mismatch')
    os.link(partial, final)  # Atomic no-clobber publish; preserve the partial name/inode.
    result = {'external_source': str(SOURCE), 'internal_bag': str(final), 'bytes': SIZE,
              'external_sha256': matches[0], 'internal_sha256': digest.hexdigest(), 'equal': True,
              'partial_preserved_as_hardlink': str(partial), 'official_checksum': None}
    with receipt.open('x') as stream: json.dump(result, stream, indent=2)
    print(json.dumps(result, indent=2))


if __name__ == '__main__': main()
