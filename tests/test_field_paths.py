"""Exercise default field selection through the actual shell CLI without hardware."""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('field_paths', ROOT/'scripts/internal/field_paths.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class FieldPathsTests(unittest.TestCase):
    def setUp(self):
        (ROOT/'cache').mkdir(exist_ok=True)
        temporary = tempfile.TemporaryDirectory(dir=ROOT/'cache')
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        (self.root/'scripts/internal').mkdir(parents=True)
        for name in ('srm-nav', 'vehicle-navigation.sh', 'internal/env.sh',
                     'internal/field_paths.py', 'internal/site-mapping.py'):
            shutil.copy2(ROOT/'scripts'/name, self.root/'scripts'/name)
        run = self.root/'scripts/run.sh'
        run.write_text('#!/usr/bin/env python3\nimport json,sys\nprint(json.dumps(sys.argv[1:]))\n')
        run.chmod(0o755)
        self.env = {k:v for k,v in os.environ.items() if k not in
                    ('INTRAMURAL_NAVIGATION_CONFIG','INTRAMURAL_VEHICLE_CONFIG')}
        self.env['INTRAMURAL_SERIAL_DEVICE'] = '/dev/null'

    def output(self, relative, stamp, accepted=True):
        folder = self.root/relative
        folder.mkdir(parents=True)
        for name in ('arena.json','nav.yaml','map.yaml','map.pgm','prior.pcd'):
            (folder/name).write_text('{}')
        marker = folder/'manifest.json'
        marker.write_text(json.dumps({'field_geometry_verified':accepted}))
        os.utime(marker, (stamp,stamp))
        return folder

    def bag(self, relative, stamp):
        folder = self.root/relative
        folder.mkdir(parents=True)
        marker = folder/'metadata.yaml'
        marker.write_text('rosbag2_bagfile_information: {}\n')
        os.utime(marker,(stamp,stamp))
        return folder

    def cli(self, *args, check=True):
        return subprocess.run([str(self.root/'scripts/srm-nav'),*args], cwd=self.root.parent,
                              env=self.env, text=True, capture_output=True, check=check, timeout=5)

    def test_latest_navigation_skips_unaccepted_incomplete_and_archived_outputs(self):
        self.output('cache/navigation-old',100)
        chosen=self.output('artifacts/mapping/20261016-100000/navigation',200)
        self.output('cache/navigation-unaccepted',300,accepted=False)
        incomplete=self.output('cache/navigation-incomplete',400)
        (incomplete/'nav.yaml').unlink()
        self.output('artifacts/field/closeout/cache/navigation-copy',500)
        self.assertEqual(module.latest('navigation',self.root),chosen)
        result=self.cli('drive')
        self.assertIn(str(chosen),result.stderr)
        self.assertIn('navigation_config:=/work/'+str(chosen.relative_to(self.root)),json.loads(result.stdout.splitlines()[-1]))
        self.assertIn('serial_device:=/dev/null',json.loads(result.stdout.splitlines()[-1]))

    def test_explicit_project_relative_path_overrides_latest(self):
        chosen=self.output('cache/navigation-old',100)
        self.output('cache/navigation-new',200)
        result=self.cli('drive','cache/navigation-old','/dev/null')
        self.assertIn('navigation_config:=/work/cache/navigation-old',json.loads(result.stdout.splitlines()[-1]))
        self.assertEqual(module.latest('navigation',self.root),self.root/'cache/navigation-new')

    def test_map_commands_default_to_latest_exported_map(self):
        self.output('cache/vehicle-navigation-old',100,accepted=False)
        chosen=self.output('artifacts/mapping/20261016-100000/map',200,accepted=False)
        for command in ('field','standby','check-standby','map-check'):
            with self.subTest(command=command):
                result=self.cli(command)
                self.assertIn(str(chosen),result.stderr)
                self.assertIn('/work/'+str(chosen.relative_to(self.root)),result.stdout)

    def test_bag_commands_skip_unfinished_directory(self):
        self.bag('artifacts/mapping/20261015-100000/bag',100)
        chosen=self.bag('artifacts/mapping/20261016-100000/bag',200)
        (self.root/'artifacts/mapping/20261016-110000/bag').mkdir(parents=True)
        for command in ('bag-info','bag-check'):
            result=self.cli(command)
            self.assertIn(str(chosen),result.stderr)
            self.assertIn('/work/'+str(chosen.relative_to(self.root)),result.stdout)

    def test_no_candidate_refuses_before_launch(self):
        result=self.cli('drive',check=False)
        self.assertNotEqual(result.returncode,0)
        self.assertIn('没有找到',result.stderr)
        self.assertEqual(result.stdout,'')

    def test_mapping_bag_without_value_selects_latest_before_device_checks(self):
        chosen=self.bag('artifacts/mapping/20261016-100000/bag',200)
        result=subprocess.run(['python3',str(self.root/'scripts/internal/site-mapping.py'),
                               '--bag','--vehicle-config','missing'],cwd=self.root,
                              env=self.env,text=True,capture_output=True,timeout=5)
        self.assertEqual(result.returncode,2)
        self.assertIn(str(chosen),result.stdout)
        self.assertIn('缺少实车配置',result.stderr)


if __name__=='__main__':
    unittest.main()
