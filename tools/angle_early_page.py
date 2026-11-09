"""Package and verify the pinned experimental early-input flasher."""
import argparse
import hashlib
import json
import re
import shutil
import urllib.request
from pathlib import Path

COMMIT = '9939ff1da6550ac9de21451c92c1106edd8b374f'
BUILD_ID = 'angle-early-eta-v1'
PARTS = {'firmware/bootloader.bin': 0, 'firmware/partitions.bin': 32768,
         'firmware/boot_app0.bin': 57344, 'firmware/firmware.bin': 65536}
LIMITS = {'firmware/bootloader.bin': 0x8000, 'firmware/partitions.bin': 0x1000,
          'firmware/boot_app0.bin': 0x2000, 'firmware/firmware.bin': 0x330000}


def validate_config():
    if not re.fullmatch(r'[0-9a-f]{40}', COMMIT) or COMMIT == '0' * 40:
        raise ValueError('Early-input source SHA is not set to a reviewed immutable commit')
    if BUILD_ID != 'angle-early-eta-v1':
        raise ValueError('Unexpected experimental firmware identity')


def digest(data):
    if not data:
        raise ValueError('empty flash part')
    return hashlib.sha256(data).hexdigest()


def manifest():
    validate_config()
    return {'name': 'Experimental angle ETA early input',
            'version': BUILD_ID + '-' + COMMIT[:12],
            'new_install_prompt_erase': True,
            'new_install_improv_wait_time': 0,
            'builds': [{'chipFamily': 'ESP32-S3', 'parts': [
                {'path': path, 'offset': offset} for path, offset in PARTS.items()]}]}


def verify(read):
    validate_config()
    info = json.loads(read('BUILD.json'))
    if set(info) != {'commit', 'experimental_build_id', 'firmware_sha256', 'bundle_sha256'} or info['commit'] != COMMIT or info['experimental_build_id'] != BUILD_ID:
        raise ValueError('unexpected build identity')
    if not all(re.fullmatch(r'[0-9a-f]{64}', info[k]) for k in ('firmware_sha256', 'bundle_sha256')):
        raise ValueError('invalid build hashes')
    prefix = 'builds/' + COMMIT + '-' + info['firmware_sha256'] + '-' + info['bundle_sha256'] + '/'
    if json.loads(read(prefix + 'manifest.json')) != manifest():
        raise ValueError('unexpected manifest')
    page = read('index.html').decode('utf8')
    if COMMIT not in page or BUILD_ID not in page or 'manifest="' + prefix + 'manifest.json"' not in page:
        raise ValueError('page identity mismatch')
    if page.index('../flasher-completion.js') >= page.index('esp-web-tools@10.1.1') or 'Erase device' not in page:
        raise ValueError('non-erase patch or fallback warning missing')
    sums = {}
    for line in read('SHA256SUMS.txt').decode('ascii').splitlines():
        match = re.fullmatch(r'([0-9a-f]{64})  (firmware/[a-z0-9_.-]+)', line)
        if not match or match[2] in sums:
            raise ValueError('invalid hash list')
        sums[match[2]] = match[1]
    if set(sums) != set(PARTS):
        raise ValueError('missing or extra flash part')
    if sums['firmware/firmware.bin'] != info['firmware_sha256']:
        raise ValueError('firmware identity mismatch')
    calculated_bundle = hashlib.sha256(''.join(sums[path] for path in PARTS).encode('ascii')).hexdigest()
    if calculated_bundle != info['bundle_sha256']:
        raise ValueError('bundle identity mismatch')
    for path in PARTS:
        data = read(prefix + path)
        if len(data) > LIMITS[path] or digest(data) != sums[path]:
            raise ValueError('hash mismatch: ' + path)


def package(source, stable, dest):
    validate_config()
    if dest.exists():
        raise ValueError('destination already exists')
    if (source / 'source-sha.txt').read_text(encoding='ascii').strip() != COMMIT:
        raise ValueError('wrong source checkout')
    for name in ('bootloader.bin', 'partitions.bin', 'boot_app0.bin'):
        if digest((source / 'firmware' / name).read_bytes()) != digest((stable / name).read_bytes()):
            raise ValueError('different shared flash part: ' + name)
    for name in PARTS:
        if not 0 < (source / name).stat().st_size <= LIMITS[name]:
            raise ValueError('flash part outside allowed extent: ' + name)
    hashes = {name: digest((source / name).read_bytes()) for name in PARTS}
    firmware_sha = hashes['firmware/firmware.bin']
    bundle_sha = hashlib.sha256(''.join(hashes[name] for name in PARTS).encode('ascii')).hexdigest()
    prefix = 'builds/' + COMMIT + '-' + firmware_sha + '-' + bundle_sha + '/'
    dest.mkdir(parents=True)
    for name in PARTS:
        target = dest / prefix / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source / name, target)
    (dest / prefix / 'manifest.json').write_text(json.dumps(manifest(), indent=2) + '\n', encoding='utf8')
    (dest / 'BUILD.json').write_text(json.dumps({'commit': COMMIT, 'experimental_build_id': BUILD_ID, 'firmware_sha256': firmware_sha, 'bundle_sha256': bundle_sha}, indent=2) + '\n', encoding='utf8')
    sums = ''.join(hashes[name] + '  ' + name + '\n' for name in PARTS)
    (dest / 'SHA256SUMS.txt').write_text(sums, encoding='ascii')
    page = f'''<!doctype html><html lang="ja"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>\u59ff\u52e2\u306b\u3088\u308b\u65e9\u671f\u5165\u529b\u8a66\u9a13</title><script src="../flasher-completion.js"></script>
<script type="module" src="https://unpkg.com/esp-web-tools@10.1.1/dist/web/install-button.js?module"></script>
<style>body{{font:16px/1.7 system-ui,sans-serif;max-width:760px;margin:3rem auto;padding:0 1rem}}code{{word-break:break-all}}button{{font-size:1.1rem;padding:.8rem 1rem}}</style></head>
<body><h1>\u59ff\u52e2\u306b\u3088\u308b\u65e9\u671f\u5165\u529b\u8a66\u9a13</h1>
<p><strong>\u5b9f\u6a5f\u306e\u30e2\u30fc\u30bf\u51fa\u529b\u304c\u5909\u308f\u308b\u8a66\u9a13\u7248\u3067\u3059\u3002\u5b9f\u6a5f\u3067\u306e\u52b9\u679c\u3084\u5b89\u5b9a\u6027\u306f\u672a\u691c\u8a3c\u3067\u3059\u3002</strong></p>
<p>\u59ff\u52e2\u3068\u89d2\u901f\u5ea6\u304b\u3089 ZERO \u307e\u3067\u306e\u6642\u9593\u3092\u63a8\u5b9a\u3057\u3001\u6761\u4ef6\u304c\u6210\u7acb\u3057\u305f\u5834\u5408\u306b\u9650\u308a\u65e9\u304f\u5165\u529b\u3057\u307e\u3059\u3002\u73fe\u5728\u306e\u8a66\u9a13\u95be\u5024\u306f 20\u301c30 ms\u3001\u65e9\u671f\u5165\u529b\u306e\u30d1\u30eb\u30b9\u5e45\u306e\u4e0a\u9650\u306f 15 ms \u3067\u3059\u3002\u6761\u4ef6\u304c\u6210\u7acb\u3057\u306a\u3044\u5834\u5408\u306f\u65e2\u5b58\u306e ZERO \u5165\u529b\u306b\u623b\u308a\u307e\u3059\u3002</p>
<p>設定目標が 8° の場合も、起動時の内部目標 4.5° の段階から条件が成立すれば動作します。早期入力のパルス幅は最大 15 ms です。通常の ZERO 入力と比較すると、入力タイミングとパルス幅の両方が変わります。カメラ認識は既存の処理のままです。</p>
<p>Chrome \u307e\u305f\u306f Edge \u3067 USB \u63a5\u7d9a\u3057\u3066\u304f\u3060\u3055\u3044\u3002\u66f8\u304d\u8fbc\u307f\u78ba\u8a8d\u304c\u8868\u793a\u3055\u308c\u305f\u5834\u5408\u3001<strong>Erase device \u306b\u30c1\u30a7\u30c3\u30af\u3057\u306a\u3044\u3067\u304f\u3060\u3055\u3044\u3002</strong></p>
<esp-web-install-button manifest="{prefix}manifest.json"><button slot="activate">\u8a66\u9a13\u7248\u3092\u66f8\u304d\u8fbc\u3080</button><span slot="unsupported">Chrome \u307e\u305f\u306f Edge \u3092\u4f7f\u7528\u3057\u3066\u304f\u3060\u3055\u3044\u3002</span></esp-web-install-button>
<p>ID: <code>{BUILD_ID}</code><br>\u30bd\u30fc\u30b9: <code>{COMMIT}</code></p>
<p><a href="../">\u901a\u5e38\u7248</a>\u30fb<a href="../camera/">\u30ab\u30e1\u30e9\u8a3a\u65ad</a>\u30fb<a href="../foot-observation/">\u8db3\u89b3\u6e2c</a>\u30fb<a href="../early-input/">\u5e7e\u4f55\u65e9\u671f\u5165\u529b</a></p></body></html>'''
    (dest / 'index.html').write_text(page, encoding='utf8')
    verify(lambda name: (dest / name).read_bytes())


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--package', action='store_true')
    group.add_argument('--path', type=Path)
    group.add_argument('--url')
    parser.add_argument('--source', type=Path)
    parser.add_argument('--stable', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.package:
        if not all((args.source, args.stable, args.output)):
            parser.error('--package needs --source, --stable and --output')
        package(args.source, args.stable, args.output)
    elif args.path:
        verify(lambda name: (args.path / name).read_bytes())
    else:
        base = args.url.rstrip('/') + '/'
        def read(name):
            request = urllib.request.Request(base + name + '?verify=' + COMMIT,
                                             headers={'Cache-Control': 'no-cache'})
            with urllib.request.urlopen(request, timeout=30) as response:
                return response.read()
        verify(read)
