"""Exercise clean reconstruction and local patch branches without touching real sources."""

import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from internal.source_control import diff_from_base, git, validate
import sources


class SourceControlTest(unittest.TestCase):
    def test_cli_restores_empty_submodule_directory_and_preserves_extra_files(self):
        consumer = self.root / 'consumer'
        scripts = consumer / 'scripts'
        (scripts / 'internal').mkdir(parents=True)
        project = Path(__file__).resolve().parents[1]
        for name in ('prepare-sources.py', 'internal/source_control.py'):
            shutil.copy2(project / 'scripts' / name, scripts / name)
        (consumer / 'docker').mkdir()
        repo = dict(self.repo, url=self.path.as_uri())
        (consumer / 'docker/sources.lock.json').write_text(json.dumps({'repositories': [repo]}))
        target = consumer / repo['path']
        target.mkdir(parents=True)  # A plain root clone leaves empty submodule directories.
        command = [sys.executable, str(scripts / 'prepare-sources.py')]
        for _ in range(2):
            result = subprocess.run(command, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(git(target, 'rev-parse', 'HEAD'), repo['commit'])
            self.assertEqual((target / 'code.txt').read_text(), 'upstream\n')
        extra = target / 'personal.txt'
        extra.write_text('preserve this work\n')
        result = subprocess.run(command, text=True, capture_output=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(extra.read_text(), 'preserve this work\n')

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.path = self.root / "src/fixture"
        self.path.mkdir(parents=True)
        git(self.path, "init", "-q")
        git(self.path, "config", "user.name", "Fixture")
        git(self.path, "config", "user.email", "fixture@example.invalid")
        (self.path / "code.txt").write_text("upstream\n")
        git(self.path, "add", "code.txt")
        git(self.path, "commit", "-qm", "base")
        self.repo = {"path": "src/fixture", "commit": git(self.path, "rev-parse", "HEAD"),
                     "recursive": False}

    def add_patch(self):
        (self.path / "code.txt").write_text("local repair\n")
        content = diff_from_base(self.path, self.repo)
        (self.root / "patches").mkdir()
        (self.root / "patches/fixture.patch").write_bytes(content)
        self.repo["patch"] = {"path": "patches/fixture.patch",
                              "sha256": hashlib.sha256(content).hexdigest()}

    def test_clean_reconstruction_and_committed_branch(self):
        self.add_patch()
        git(self.path, "restore", "code.txt")
        with self.assertRaisesRegex(RuntimeError, "locked patch"):
            validate(self.root, self.repo)
        self.assertEqual(validate(self.root, self.repo, apply_missing=True)["state"], "applied-patch")
        git(self.path, "switch", "-qc", "intramural/local")
        git(self.path, "add", "code.txt")
        git(self.path, "commit", "-qm", "repair")
        self.assertEqual(validate(self.root, self.repo)["state"], "committed-patch")
        # Applying again must only validate, never reapply an already committed repair.
        validate(self.root, self.repo, apply_missing=True)

    def test_checksum_and_extra_change_rejected(self):
        self.add_patch()
        (self.path / "code.txt").write_text("unexpected\n")
        with self.assertRaisesRegex(RuntimeError, "locked patch"):
            validate(self.root, self.repo, apply_missing=True)
        (self.root / "patches/fixture.patch").write_text("corrupt")
        with self.assertRaisesRegex(RuntimeError, "checksum"):
            validate(self.root, self.repo)

    def test_untracked_and_staged_changes_rejected(self):
        self.add_patch()
        (self.path / "extra.txt").write_text("untracked")
        with self.assertRaisesRegex(RuntimeError, "Untracked"):
            validate(self.root, self.repo)
        (self.path / "extra.txt").unlink()
        git(self.path, "add", "code.txt")
        with self.assertRaisesRegex(RuntimeError, "Staged"):
            validate(self.root, self.repo)

    def test_unpatched_commit_drift_rejected(self):
        self.assertEqual(validate(self.root, self.repo)["state"], "upstream")
        git(self.path, "commit", "--allow-empty", "-qm", "unlocked commit")
        with self.assertRaisesRegex(RuntimeError, "pinned base"):
            validate(self.root, self.repo)

    def test_export_committed_repair_and_lock(self):
        self.add_patch()
        git(self.path, "switch", "-qc", "intramural/local")
        git(self.path, "add", "code.txt")
        git(self.path, "commit", "-qm", "repair")
        (self.path / "code.txt").write_text("second reviewed repair\n")
        git(self.path, "commit", "-am", "second repair", "-q")
        (self.root / "docker").mkdir()
        lock_path = self.root / "docker/sources.lock.json"
        lock_path.write_text(json.dumps({"repositories": [self.repo]}))
        with patch.object(sources, "ROOT", self.root), patch.object(sys, "argv",
                ["sources.py", "export", "fixture"]):
            sources.main()
        locked = json.loads(lock_path.read_text())["repositories"][0]
        self.assertEqual(validate(self.root, locked)["state"], "committed-patch")
        self.assertEqual((self.root / locked["patch"]["path"]).read_bytes(),
                         diff_from_base(self.path, locked))

    def test_patch_added_file_survives_repeated_preparation(self):
        git(self.path, "switch", "-qc", "intramural/local")
        (self.path / "added.txt").write_text("new source\n")
        git(self.path, "add", "added.txt")
        git(self.path, "commit", "-qm", "add source")
        content = diff_from_base(self.path, self.repo)
        (self.root / "patches").mkdir()
        (self.root / "patches/fixture.patch").write_bytes(content)
        self.repo["patch"] = {"path": "patches/fixture.patch",
                              "sha256": hashlib.sha256(content).hexdigest()}
        git(self.path, "switch", "--detach", self.repo["commit"])
        validate(self.root, self.repo, apply_missing=True)
        validate(self.root, self.repo, apply_missing=True)
        self.assertEqual((self.path / "added.txt").read_text(), "new source\n")


if __name__ == "__main__":
    unittest.main()
