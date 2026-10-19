"""Exercise PNG export through the saved-map CLI and mapping completion hook."""
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

    def staged_cli(self, root):
        for relative in ('scripts/srm-nav','scripts/internal/field_preview.py','scripts/internal/field_paths.py'):
            target=root/relative;target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(ROOT/relative,target)
        (root/'cache').mkdir()
        return root/'scripts/srm-nav'

    def test_route_png_is_read_only_and_rotated_map_markers_align(self):
        for yaw in (0.,.7):
            with self.subTest(yaw=yaw),tempfile.TemporaryDirectory(dir=ROOT/'cache') as temp:
                root=Path(temp);folder=self.fixture(root,yaw=yaw);output=root/'export.png'
                before={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in folder.iterdir()}
                result=self.cli(folder,'--output',output)
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                with Image.open(output) as image:
                    self.assertEqual(image.format,'PNG')
                    self.assertEqual(image.getpixel((660,770)),(255,0,0))
                self.assertEqual(result.stdout.strip(),str(output))
                self.assertEqual(before,{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in folder.iterdir()})

    def test_raw_map_png_preserves_unknown_and_obstacles(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'cache') as temp:
            root=Path(temp);self.fixture(root,route=False);output=root/'export.png'
            result=self.cli(temp,'--output',output)
            self.assertEqual(result.returncode,0,result.stderr)
            with Image.open(output) as image:
                self.assertEqual(image.getpixel((90,200)),(205,205,205))
                self.assertEqual(image.getpixel((300,60)),(0,0,0))

    def test_invalid_plan_and_source_overwrite_are_rejected(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'cache') as temp:
            root=Path(temp);folder=self.fixture(root);output=root/'export.png'
            plan=folder/'route-plan.json';plan.write_text('{broken')
            self.assertEqual(self.cli(folder,'--output',output).returncode,2)
            self.assertFalse(output.exists())
            self.assertEqual(plan.read_text(),'{broken')
            plan.unlink();source=folder/'map.pgm'
            self.assertEqual(self.cli(folder,'--output',source).returncode,2)
            with Image.open(source) as image:
                self.assertEqual(image.format,'PPM')

    def test_existing_route_png_is_reused(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'cache') as temp:
            root=Path(temp);folder=self.fixture(root);output=root/'export.png'
            Image.new('RGB',(320,240),'purple').save(folder/'route-plan.png')
            self.assertEqual(self.cli(folder,'--output',output).returncode,0)
            with Image.open(output) as image:
                self.assertEqual(image.size,(320,240))
                self.assertEqual(image.getpixel((100,100)),(128,0,128))

    def test_latest_map_avoids_old_route_and_all_exports_every_map(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'cache') as temp:
            root=Path(temp);cli=self.staged_cli(root)
            old=self.fixture(root/'cache');old=old.rename(old.with_name('navigation-old'))
            parent=root/'artifacts/mapping/20261019';parent.mkdir(parents=True)
            new=self.fixture(parent,route=False)
            os.utime(old/'manifest.json',(100,100));os.utime(new/'manifest.json',(200,200))
            result=subprocess.run([str(cli),'preview'],cwd=ROOT.parent,text=True,capture_output=True,timeout=5)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertIn('mapping--20261019--map.png',result.stdout)
            result=subprocess.run([str(cli),'preview','--all'],text=True,capture_output=True,timeout=5)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertIn('2 成功，0 失败',result.stdout)
            self.assertEqual(len(list((root/'artifacts/.png').glob('*.png'))),2)

    def test_mapping_completion_exports_png_without_terminal_drawing(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'cache') as temp:
            root=Path(temp);self.staged_cli(root)
            self.fixture(root,route=False)
            env=os.environ.copy();env['PYTHONPATH']=str(root/'scripts/internal')
            code='from pathlib import Path; from field_preview import mapping_result; mapping_result(Path('+repr(str(root))+'), {"status":"complete", "field_geometry_detected":False})'
            result=subprocess.run(['python3','-c',code],env=env,text=True,capture_output=True,timeout=5)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertIn('PNG:',result.stdout)
            self.assertNotIn('地图 / 路线预览',result.stdout)
            self.assertEqual(len(list((root/'artifacts/.png').glob('*.png'))),1)


if __name__=='__main__':
    unittest.main()
