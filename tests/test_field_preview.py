"""Exercise the real read-only CLI with saved map/route artifacts; never start ROS."""
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from PIL import Image, ImageDraw
import yaml

ROOT = Path(__file__).resolve().parents[1]


class FieldPreviewTests(unittest.TestCase):
    def fixture(self, root, route=True, yaw=0.):
        folder = root/('navigation' if route else 'map');folder.mkdir()
        image = Image.new('L',(200,160),254)
        draw = ImageDraw.Draw(image);draw.rectangle((0,0,199,8),fill=0)
        draw.rectangle((0,9,22,159),fill=205)
        image.save(folder/'map.pgm')
        (folder/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.05,origin=[-5.,-4.,yaw],free_thresh=.2,occupied_thresh=.65)))
        def world(p):
            x,y = p[0]+5,p[1]+4
            return [-5+math.cos(yaw)*x-math.sin(yaw)*y,-4+math.sin(yaw)*x+math.cos(yaw)*y]
        (folder/'arena.json').write_text(json.dumps(dict(robot_radius=.29,parking=world([0,-2]),lower_pole=world([-.8,0]),upper_pole=world([.8,1]))))
        (folder/'manifest.json').write_text(json.dumps(dict(field_geometry_verified=route)))
        (folder/'nav.yaml').write_text('{}')
        (folder/'prior.pcd').write_text('DATA binary\n')
        if route:
            points=[world(p) for p in [[0,-2],[-1,-1],[-2,0],[-1,2],[1,2],[2,0],[0,-2]]]
            (folder/'route-plan.json').write_text(json.dumps(dict(points=points,s=list(range(7)),speed=[0,.4,.8,1.2,.8,.4,0])))
        return folder

    def cli(self,*args):
        return subprocess.run([str(ROOT/'scripts/srm-nav'),'preview',*map(str,args)],
                              cwd=ROOT.parent,text=True,capture_output=True,timeout=5)

    def test_saved_route_speed_and_markers_are_read_only_including_rotated_maps(self):
        for yaw in (0.,.7):
            with self.subTest(yaw=yaw),tempfile.TemporaryDirectory(dir=ROOT/'cache') as temp:
                folder=self.fixture(Path(temp),yaw=yaw)
                before={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in folder.iterdir()}
                result=self.cli(folder,'--width',60)
                self.assertEqual(result.returncode,0,result.stderr)
                rows=[line for line in result.stdout.splitlines() if line.startswith('|')]
                self.assertLessEqual(max(map(len,rows)),62)
                grid=''.join(rows)
                for marker in ('S','L','U','*'):self.assertIn(marker,grid)
                self.assertIn('1.20',result.stdout)
                self.assertIn(str(folder/'route-plan.json'),result.stdout)
                self.assertNotIn('\x1b',result.stdout)
                self.assertEqual(before,{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in folder.iterdir()})

    def test_session_without_route_shows_map_unknown_and_obstacles(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'cache') as temp:
            self.fixture(Path(temp),route=False)
            result=self.cli(temp,'--width',36,'--no-color')
            self.assertEqual(result.returncode,0,result.stderr)
            grid=''.join(line for line in result.stdout.splitlines() if line.startswith('|'))
            self.assertIn('?',grid);self.assertIn('#',grid)
            self.assertNotIn('S',grid)

    def test_invalid_width_and_invalid_plan_fail_without_touching_files(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'cache') as temp:
            folder=self.fixture(Path(temp))
            self.assertEqual(self.cli(folder,'--width',4).returncode,2)
            p=folder/'route-plan.json';p.write_text('{broken')
            self.assertEqual(self.cli(folder).returncode,2)
            self.assertEqual(p.read_text(),'{broken')

    def test_default_preview_uses_new_map_instead_of_an_older_route(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'cache') as temp:
            root=Path(temp)
            for relative in ('scripts/srm-nav','scripts/internal/field_preview.py','scripts/internal/field_paths.py'):
                target=root/relative;target.parent.mkdir(parents=True,exist_ok=True)
                shutil.copy2(ROOT/relative,target)
            (root/'cache').mkdir()
            old=self.fixture(root/'cache');old=old.rename(old.with_name('navigation-old'))
            parent=root/'artifacts/mapping/20261019';parent.mkdir(parents=True)
            new=self.fixture(parent,route=False)
            os.utime(old/'manifest.json',(100,100));os.utime(new/'manifest.json',(200,200))
            result=subprocess.run([str(root/'scripts/srm-nav'),'preview'],cwd=ROOT.parent,
                                  text=True,capture_output=True,timeout=5)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertIn(str(new/'map.pgm'),result.stdout)
            self.assertNotIn(str(old),result.stdout)
            self.assertNotIn('参考速度',result.stdout)


if __name__=='__main__':
    unittest.main()
