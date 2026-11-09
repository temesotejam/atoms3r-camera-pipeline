import json
import re
import tempfile
import unittest
from pathlib import Path
import foot_observation_page as page


class PackagingTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = Path(self.tmp.name)
        self.source = root / 'source'
        self.stable = root / 'stable'
        self.dest = root / 'site' / 'foot-observation'
        (self.source / 'firmware').mkdir(parents=True)
        self.stable.mkdir()
        (self.source / 'source-sha.txt').write_text(page.COMMIT, encoding='ascii')
        for part in page.PARTS:
            (self.source / part).write_bytes(part.encode())
            if not part.endswith('firmware.bin'):
                (self.stable / Path(part).name).write_bytes(part.encode())

    def tearDown(self):
        self.tmp.cleanup()

    def make(self):
        page.package(self.source, self.stable, self.dest)

    def bundle(self):
        info = json.loads((self.dest / 'BUILD.json').read_text())
        return self.dest / 'builds' / (page.COMMIT + '-' + info['firmware_sha256'] + '-' + info['bundle_sha256'])

    def test_success_and_preserves_other_site(self):
        self.dest.parent.mkdir()
        old = self.dest.parent / 'index.html'
        old.write_bytes(b'normal')
        camera = self.dest.parent / 'camera' / 'index.html'
        camera.parent.mkdir()
        camera.write_bytes(b'camera')
        self.make()
        self.assertEqual(old.read_bytes(), b'normal')
        self.assertEqual(camera.read_bytes(), b'camera')
        page.verify(lambda name: (self.dest / name).read_bytes())
        self.assertIn('href="../camera/"', (self.dest / 'index.html').read_text(encoding='utf8'))
        self.assertIn('ブラウザと実機での動作確認がまだ済んでいません', (self.dest / 'index.html').read_text(encoding='utf8'))
        with self.assertRaises(ValueError):
            self.make()

    def test_rejects_wrong_source_and_shared_flash(self):
        (self.source / 'source-sha.txt').write_text('bad')
        with self.assertRaises(ValueError):
            self.make()
        self.assertFalse(self.dest.exists())
        (self.source / 'source-sha.txt').write_text(page.COMMIT)
        (self.stable / 'bootloader.bin').write_bytes(b'wrong')
        with self.assertRaises(ValueError):
            self.make()
        self.assertFalse(self.dest.exists())

    def test_rejects_changed_firmware_and_manifest(self):
        self.make()
        (self.bundle() / 'firmware/firmware.bin').write_bytes(b'tampered')
        with self.assertRaises(ValueError):
            page.verify(lambda name: (self.dest / name).read_bytes())
        (self.bundle() / 'firmware/firmware.bin').write_bytes(b'firmware/firmware.bin')
        mf = self.bundle() / 'manifest.json'
        data = json.loads(mf.read_text())
        data['builds'][0]['parts'][3]['offset'] = 0
        mf.write_text(json.dumps(data))
        with self.assertRaises(ValueError):
            page.verify(lambda name: (self.dest / name).read_bytes())

    def test_paths_change_when_firmware_or_shared_part_changes(self):
        self.make()
        first = self.bundle().name
        new_dest = self.dest.parent / 'other'
        (self.source / 'firmware/firmware.bin').write_bytes(b'other firmware')
        page.package(self.source, self.stable, new_dest)
        info = json.loads((new_dest / 'BUILD.json').read_text())
        self.assertNotEqual(first, page.COMMIT + '-' + info['firmware_sha256'] + '-' + info['bundle_sha256'])
        (self.source / 'firmware/firmware.bin').write_bytes(b'firmware/firmware.bin')
        (self.source / 'firmware/bootloader.bin').write_bytes(b'new bootloader')
        (self.stable / 'bootloader.bin').write_bytes(b'new bootloader')
        page.package(self.source, self.stable, self.dest.parent / 'third')
        other = json.loads((self.dest.parent / 'third/BUILD.json').read_text())
        self.assertNotEqual(first, page.COMMIT + '-' + other['firmware_sha256'] + '-' + other['bundle_sha256'])


def verify_workflow_structure(workflow):
    # Scope assertions to actual jobs, not similarly named text elsewhere.
    def job(name):
        pattern = (r'^  ' + re.escape(name) + r':\s*\n'
                   r'(.*?)(?=^  [A-Za-z0-9_-]+:\s*$|\Z)')
        matches = re.findall(pattern, workflow, re.MULTILINE | re.DOTALL)
        if len(matches) != 1:
            raise AssertionError('missing or duplicate job: ' + name)
        return matches[0]

    build = job('build-site')
    needs = re.search(r'^    needs:\s*\[([^\]]+)\]\s*$', build, re.MULTILINE)
    if not needs:
        raise AssertionError('build-site dependency list missing')
    dependencies = {x.strip() for x in needs.group(1).split(',')}
    required = {'pages-ready', 'build-camera-diagnostics', 'build-foot-observation',
                'build-early-input-trial'}
    if not required.issubset(dependencies):
        raise AssertionError('build-site must depend on every firmware package')

    deploy = job('deploy')
    if not re.search(r"^    if:\s*github\.event_name\s*!=\s*'pull_request'\s*$", deploy, re.MULTILINE):
        raise AssertionError('deploy must skip pull requests')
    if not re.search(r'^    needs:\s*build-site\s*$', deploy, re.MULTILINE):
        raise AssertionError('deploy must depend on build-site')


class WorkflowTest(unittest.TestCase):
    def setUp(self):
        self.workflow = (Path(__file__).resolve().parents[1] / '.github/workflows/pages.yml').read_text(encoding='utf8')

    def test_deployment_waits_for_all_packages_and_skips_pr(self):
        workflow = self.workflow
        verify_workflow_structure(workflow)
        self.assertIn('ref: ' + page.COMMIT, workflow)
        self.assertIn('--stable _site/firmware --output _site/foot-observation', workflow)
        self.assertIn('python3 tools/verify_camera_pages.py --path', workflow)
        self.assertIn('python3 tools/foot_observation_page.py --path', workflow)

    def test_each_dependency_is_required(self):
        for dependency in ('pages-ready', 'build-camera-diagnostics', 'build-foot-observation', 'build-early-input-trial'):
            with self.subTest(dependency=dependency):
                altered = self.workflow.replace(dependency + ']', 'missing]', 1).replace(dependency + ',', 'missing,', 1)
                with self.assertRaises(AssertionError):
                    verify_workflow_structure(altered)

    def test_pr_skip_must_belong_to_deploy_job(self):
        original = "    if: github.event_name != 'pull_request'"
        self.assertIn(original, self.workflow)
        altered = self.workflow.replace(original, "    if: github.event_name == 'pull_request'", 1)
        # Text outside the deploy job cannot satisfy the check.
        altered += "\n# " + original + "\n"
        with self.assertRaises(AssertionError):
            verify_workflow_structure(altered)


if __name__ == '__main__':
    unittest.main()
