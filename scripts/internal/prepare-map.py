#!/usr/bin/env python3
"""Download and verify the pinned, public upstream simulation point cloud."""
import hashlib
import json
import urllib.request
from pathlib import Path

root = Path(__file__).resolve().parent.parent.parent
lock = json.loads((root / 'config/maps.lock.json').read_text())
path = root / lock['path']
def valid(p):
    return p.stat().st_size == lock['bytes'] and hashlib.sha256(p.read_bytes()).hexdigest() == lock['sha256']
if path.exists():
    if not valid(path):
        raise RuntimeError(f'Existing map differs from lock; preserved: {path}')
else:
    path.parent.mkdir(parents=True, exist_ok=True)
    body = {'batch': [{'blockId': '606c1d5e-cdcc-4423-8bf0-4958e759ade3',
        'ossName': 'oss/4d5a915c-c2eb-40c4-aeeb-1520ccfa16da/rmuc_2025.pcd',
        'download': 'rmuc_2025.pcd', 'isPublic': True}]}
    request = urllib.request.Request('https://flowus.cn/api/file/create_urls',
        data=json.dumps(body).encode(), headers={'Content-Type': 'application/json',
        'Referer': 'https://flowus.cn/'})
    with urllib.request.urlopen(request, timeout=30) as response:
        data = json.load(response)
    if data.get('code') != 200 or len(data.get('data', [])) != 1:
        raise RuntimeError('Public share API did not return a download URL')
    url = data['data'][0]['url']
    if not url.startswith(lock['public_file_url'] + '?'):
        raise RuntimeError('Public share returned an unexpected file URL')
    partial = path.with_suffix('.pcd.part')
    if partial.exists():
        raise RuntimeError(f'Partial download exists; preserved for inspection: {partial}')
    request = urllib.request.Request(url, headers={'Referer': 'https://flowus.cn/'})
    with urllib.request.urlopen(request, timeout=60) as response, partial.open('xb') as output:
        while chunk := response.read(1024 * 1024):
            output.write(chunk)
    if not valid(partial):
        raise RuntimeError(f'Download checksum mismatch; preserved: {partial}')
    partial.rename(path)
print(f'OK {lock["path"]} {lock["sha256"]}')
