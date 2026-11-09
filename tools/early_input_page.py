"""Package and verify the pinned experimental early-input flasher."""
import argparse
import hashlib
import json
import re
import shutil
import urllib.request
from pathlib import Path

COMMIT = 'c0ee995bd17230df221b5fc0a92380356828a08f'
BUILD_ID = 'early-input-geometry-v1'
PARTS = {'firmware/bootloader.bin': 0, 'firmware/partitions.bin': 32768,
         'firmware/boot_app0.bin': 57344, 'firmware/firmware.bin': 65536}
LIMITS = {'firmware/bootloader.bin': 0x8000, 'firmware/partitions.bin': 0x1000,
          'firmware/boot_app0.bin': 0x2000, 'firmware/firmware.bin': 0x330000}


def validate_config():
    if not re.fullmatch(r'[0-9a-f]{40}', COMMIT) or COMMIT == '0' * 40:
        raise ValueError('Early-input source SHA is not set to a reviewed immutable commit')
    if BUILD_ID != 'early-input-geometry-v1':
        raise ValueError('Unexpected experimental firmware identity')


def digest(data):
    if not data:
        raise ValueError('empty flash part')
    return hashlib.sha256(data).hexdigest()


def manifest():
    validate_config()
    return {'name': 'Experimental geometry-based early input',
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
<title>早期入力の実機試験版</title><script src="../flasher-completion.js"></script>
<script type="module" src="https://unpkg.com/esp-web-tools@10.1.1/dist/web/install-button.js?module"></script>
<style>body{{font:16px/1.7 system-ui,sans-serif;max-width:760px;margin:3rem auto;padding:0 1rem}}code{{word-break:break-all}}button{{font-size:1.1rem;padding:.8rem 1rem}}</style></head>
<body><h1>早期入力の実機試験版</h1>
<p><strong>モータ出力が変化し得る実機試験版です。実機での効果・動作は未検証です。</strong></p>
<p>足の観測と幾何の条件が成立した場合、ZEROより前に入力を試みます。条件が成立しない場合は通常のタイミングに戻ります。候補が一度も成立しない可能性があり、安全条件や故障等により出力自体も保証されません。</p>
<p>Chrome または Edge で機体を USB 接続し、下のボタンからポートを選んでください。消去確認が表示された場合、<strong>Erase device にチェックを入れないでください。</strong></p>
<esp-web-install-button manifest="{prefix}manifest.json"><button slot="activate">早期入力試験版を書き込む</button><span slot="unsupported">Chrome または Edge を使用してください。</span></esp-web-install-button>
<p>試験版 ID: <code>{BUILD_ID}</code><br>ソース: <code>{COMMIT}</code></p>
<p>ページからの書込みと設定の保持は実機未確認です。実験後は通常版に書き戻せます。</p>
<p><a href="../">通常版に戻る</a>・<a href="../camera/">カメラ診断</a>・<a href="../foot-observation/">足観測診断</a></p></body></html>'''
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
