import json
import tempfile
import unittest
from pathlib import Path
import angle_early_page as page

FAKE_SHA='1234567890abcdef1234567890abcdef12345678'
class Packaging(unittest.TestCase):
    def setUp(self):
        self.original=page.COMMIT;self.tmp=tempfile.TemporaryDirectory();root=Path(self.tmp.name)
        self.source=root/'source';self.stable=root/'stable';self.dest=root/'site'/'angle-early'
        (self.source/'firmware').mkdir(parents=True);self.stable.mkdir()
        (self.source/'source-sha.txt').write_text(FAKE_SHA,encoding='ascii')
        for part in page.PARTS:
            (self.source/part).write_bytes(part.encode())
            if not part.endswith('firmware.bin'):(self.stable/Path(part).name).write_bytes(part.encode())
    def tearDown(self):page.COMMIT=self.original;self.tmp.cleanup()
    def test_unset_placeholder_fails_closed_before_creating_page(self):
        page.COMMIT='__ANGLE_'+'TRIAL_SHA__'
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
        for expected in ('20\u301c30 ms', '15 ms', 'ZERO', '4.5', '8°', '../early-input/', '../camera/', '../foot-observation/'):
            self.assertIn(expected, html)
        self.assertIn('起動時の内部目標', html)
        self.assertIn('パルス幅は最大 15 ms', html)
        self.assertIn('カメラ認識は既存の処理のまま', html)
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
    def test_preserves_existing_and_is_fail_closed(self):
        s=(Path(__file__).resolve().parents[1]/'.github/workflows/pages.yml').read_text(encoding='utf8')
        self.assertIn('TRIAL_SHA: "'+page.COMMIT+'"',s)
        self.assertIn('build-early-input-trial, build-angle-early-trial',s)
        for path in ('_site/camera', '_site/foot-observation', '_site/early-input', '_site/angle-early'):
            self.assertIn(path,s)
        job=s.split('  build-angle-early-trial:',1)[1].split('  build-site:',1)[0]
        for value in ('-DFOOT_EARLY_TRIAL=1','-DANGLE_EARLY_TRIAL=1',
                      '-DFOOT_GEOMETRY_PROVISIONAL_EXPOSURE=0',
                      'ANGLE_EARLY_TRIAL != 1','FOOT_GEOMETRY_PROVISIONAL_EXPOSURE != 0',
                      'ExperimentRunner::tryAngleEarlyCandidate',
                      'ExperimentRunner::decideAngleEarlyCandidate',
                      "if grep -E ' [tTwW] .*foot_geometry_worker::Async::entry", 
                      'angle_trial_build_flag_proof', 'git checkout -- platformio.ini',
                      'ELF=.pio/build/angle_trial/firmware.elf'):
            self.assertIn(value,job)

if __name__=='__main__':unittest.main()
