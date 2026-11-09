import json
import tempfile
import unittest
from pathlib import Path
import early_input_page as page

FAKE_SHA='1234567890abcdef1234567890abcdef12345678'
class Packaging(unittest.TestCase):
    def setUp(self):
        self.original=page.COMMIT;self.tmp=tempfile.TemporaryDirectory();root=Path(self.tmp.name)
        self.source=root/'source';self.stable=root/'stable';self.dest=root/'site'/'early-input'
        (self.source/'firmware').mkdir(parents=True);self.stable.mkdir()
        (self.source/'source-sha.txt').write_text(FAKE_SHA,encoding='ascii')
        for part in page.PARTS:
            (self.source/part).write_bytes(part.encode())
            if not part.endswith('firmware.bin'):(self.stable/Path(part).name).write_bytes(part.encode())
    def tearDown(self):page.COMMIT=self.original;self.tmp.cleanup()
    def test_unset_placeholder_fails_closed_before_creating_page(self):
        page.COMMIT='__EARLY_'+'TRIAL_SHA__'
        with self.assertRaises(ValueError):page.package(self.source,self.stable,self.dest)
        self.assertFalse(self.dest.exists())
        page.COMMIT='0'*40
        with self.assertRaises(ValueError):page.package(self.source,self.stable,self.dest)
    def test_success_preserves_other_pages_and_has_non_erase_fallback(self):
        page.COMMIT=FAKE_SHA
        for name in ('index.html','camera/index.html','foot-observation/index.html'):
            p=self.dest.parent/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(name.encode())
        page.package(self.source,self.stable,self.dest)
        for name in ('index.html','camera/index.html','foot-observation/index.html'):
            self.assertEqual((self.dest.parent/name).read_bytes(),name.encode())
        page.verify(lambda n:(self.dest/n).read_bytes())
        html=(self.dest/'index.html').read_text(encoding='utf8')
        self.assertIn('実機試験版',html);self.assertIn('ZEROより前',html);self.assertIn('一度も成立しない可能性',html)
        self.assertIn('未検証',html);self.assertIn('モータ出力が変化',html)
        self.assertLess(html.index('../flasher-completion.js'),html.index('esp-web-tools@10.1.1'))
        self.assertTrue(page.manifest()['new_install_prompt_erase'])
    def test_rejects_source_mismatch_shared_part_mismatch_and_corruption(self):
        page.COMMIT=FAKE_SHA
        (self.source/'source-sha.txt').write_text('wrong')
        with self.assertRaises(ValueError):page.package(self.source,self.stable,self.dest)
        (self.source/'source-sha.txt').write_text(FAKE_SHA)
        (self.stable/'partitions.bin').write_bytes(b'changed')
        with self.assertRaises(ValueError):page.package(self.source,self.stable,self.dest)
        (self.stable/'partitions.bin').write_bytes(b'firmware/partitions.bin')
        page.package(self.source,self.stable,self.dest)
        info=json.loads((self.dest/'BUILD.json').read_text());prefix='builds/'+FAKE_SHA+'-'+info['firmware_sha256']+'-'+info['bundle_sha256']+'/'
        (self.dest/prefix/'firmware/firmware.bin').write_bytes(b'tampered')
        with self.assertRaises(ValueError):page.verify(lambda n:(self.dest/n).read_bytes())
    def test_rejects_oversized_and_empty_binaries(self):
        page.COMMIT=FAKE_SHA
        (self.source/'firmware/firmware.bin').write_bytes(b'')
        with self.assertRaises(ValueError):page.package(self.source,self.stable,self.dest)
        (self.source/'firmware/firmware.bin').write_bytes(b'x'*(0x330000+1))
        with self.assertRaises(ValueError):page.package(self.source,self.stable,self.dest)

class Workflow(unittest.TestCase):
    def test_fail_closed_and_other_sites_preserved(self):
        s=(Path(__file__).resolve().parents[1]/'.github/workflows/pages.yml').read_text(encoding='utf8')
        self.assertIn('TRIAL_SHA: "'+page.COMMIT+'"',s)
        self.assertIn('[[ "$TRIAL_SHA" =~ ^[0-9a-f]{40}$ ]]',s)
        self.assertIn("if: github.event_name != 'pull_request'",s)
        self.assertIn('build-camera-diagnostics, build-foot-observation, build-early-input-trial',s)
        self.assertIn('--output _site/foot-observation',s)
        self.assertIn('path: _site/camera',s)
        self.assertIn('--stable _site/firmware --output _site/early-input',s)
        self.assertIn('RUNTIME_DIAGNOSTIC_BUILD_ID "early-input-geometry-v1"',s)

    def test_trial_env_and_linked_firmware_proofs(self):
        s=(Path(__file__).resolve().parents[1]/'.github/workflows/pages.yml').read_text(encoding='utf8')
        job=s.split('  build-early-input-trial:',1)[1].split('  build-site:',1)[0]
        for text in ('[env:early_trial]', 'extends = env:atoms3cam',
                     '-DFOOT_EARLY_TRIAL=1', '-DFOOT_GEOMETRY_PROVISIONAL_EXPOSURE=1',
                     'FOOT_EARLY_TRIAL != 1', 'FOOT_GEOMETRY_PROVISIONAL_EXPOSURE != 1',
                     'pio run -e early_trial', 'ELF=.pio/build/early_trial/firmware.elf',
                     '.pio/libdeps/early_trial/Adafruit AHRS',
                     'ExperimentRunner::tryEarlyFootCandidate',
                     'foot_geometry_worker::Async::entry', 'early_trial_build_flag_proof',
                     'early-trial-flags:1,1', 'git checkout -- platformio.ini'):
            self.assertIn(text,job)
        self.assertNotIn('pio run -e atoms3cam',job)
        self.assertIn('cp .pio/build/early_trial/{bootloader.bin,partitions.bin,firmware.bin}',job)
        self.assertLess(job.index('git checkout -- platformio.ini'),job.index('git diff --exit-code'))

if __name__=='__main__':unittest.main()
