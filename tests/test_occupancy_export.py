"""Recorded-scene tests through the production map export CLI, without hardware."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
from PIL import Image
import rosbag2_py
from rclpy.serialization import serialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
import yaml


class OccupancyExportTests(unittest.TestCase):
    def test_isolated_hits_stay_unconfirmed_but_thin_and_low_objects_survive(self):
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/internal'))
        from occupancy import raycast_grid
        floor=np.array([[x,y,0.] for x in np.arange(-.5,2.1,.05) for y in np.arange(-1.,1.1,.05)])
        scans=[np.vstack((floor,[[1.,.6,.6],[.6,-.6,.055]])) for _ in range(10)]
        scans[0]=np.vstack((scans[0],[[1.,0.,.6],[3.,3.,.6],
                                    [.81,-.29,.31],[.86,-.29,.31],[.81,-.24,.31]]))
        grid,report=raycast_grid(scans,np.tile([0.,0.,.25],(len(scans),1)),np.zeros(3),
                                [-1.,-1.],np.array([100,100]),.05,Path(__file__).resolve().parents[1])
        def pixel(x,y):return grid[int(round((y+1)/.05)),int(round((x+1)/.05))]
        self.assertEqual(pixel(1.,0.),254)
        self.assertEqual(pixel(3.,3.),205,'An unconfirmed endpoint must not invent free space')
        self.assertEqual(pixel(1.,.6),0,'Repeated sparse thin obstacle was lost')
        self.assertTrue(np.any(grid[7:10,31:34]==0),'Low obstacle was lost')
        self.assertTrue(np.any(grid[13:16,35:38]==0),'Spatially supported single-frame object was lost')
        self.assertGreater(report['octomap']['unsupported_occupied_voxels'],0)

    def test_high_rays_clear_a_departed_object_without_filling_behind_a_wall(self):
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/internal'))
        from occupancy import raycast_grid
        # The ray to z=1.6 crosses the old object at x=1, z=.925.
        scans=[np.array([[1.,0.,.925]]) for _ in range(3)]
        scans += [np.array([[2.,0.,1.6]]) for _ in range(14)]
        grid,_=raycast_grid(scans,np.tile([0.,0.,.25],(len(scans),1)),np.zeros(3),
                            [-1.,-1.],np.array([100,100]),.05,Path(__file__).resolve().parents[1])
        self.assertEqual(grid[20,40],254)
        self.assertEqual(grid[20,80],205)

    def scene(self, with_pose=True, with_ground=True):
        root = Path(__file__).resolve().parents[1]
        base = root / "artifacts/tests/occupancy-export"
        base.mkdir(parents=True, exist_ok=True)
        folder = Path(tempfile.mkdtemp(dir=base))
        vehicle = folder / "vehicle"
        vehicle.mkdir()
        for name in ("handoff.json", "base-vehicle.json"):
            (vehicle / name).write_text("{}")
        (vehicle / "point-lio.yaml").write_text(yaml.safe_dump({"/**": {"ros__parameters": {"mapping": {
            "extrinsic_est_en": False, "extrinsic_T": [-.011, -.02329, .04412]}}}}))
        points = []
        # Floor is at camera_init z=-0.09, hence map z=+0.13, not z=0.
        if with_ground:
            for x in np.arange(-1.4, 1.41, .12):
                for y in np.arange(-1.4, 1.41, .12):
                    points.append([x, y, -.09 + .005*np.sin(3*x+y)])
            for x in np.arange(.55,.651,.025):
                for y in np.arange(-.7,-.599,.025):
                    points.append([x,y,-.09+.055])
        # A wall terminates the rays. No measurements exist behind it.
        # Sample finer than a cell so float rounding at voxel boundaries cannot leave unobserved wall gaps.
        for y in np.arange(-1.5, 1.51, .025):
            for z in np.arange(-.02, .91, .05):
                points.append([1.5, y, z])
        xyz = np.asarray(points, dtype="<f4")
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(folder / "bag"), storage_id="sqlite3"),
                    rosbag2_py.ConverterOptions("", ""))
        cloud_topic = "/intramural_robot/cloud_registered"
        odom_topic = "/intramural_robot/aft_mapped_to_init"
        writer.create_topic(rosbag2_py.TopicMetadata(name=cloud_topic, type="sensor_msgs/msg/PointCloud2", serialization_format="cdr"))
        if with_pose:
            writer.create_topic(rosbag2_py.TopicMetadata(name=odom_topic, type="nav_msgs/msg/Odometry", serialization_format="cdr"))
        for index in range(20):
            msg = PointCloud2()
            msg.header.frame_id = "camera_init"
            msg.header.stamp.sec = 1000 + index//10
            msg.header.stamp.nanosec = index%10 * 100000000
            msg.height, msg.width, msg.point_step, msg.row_step = 1, len(xyz), 12, len(xyz)*12
            msg.fields = [PointField(name=k, offset=i*4, datatype=7, count=1) for i,k in enumerate("xyz")]
            msg.is_dense = True
            msg.data = xyz.tobytes()
            stamp = msg.header.stamp.sec*1000000000 + msg.header.stamp.nanosec
            if with_pose:
                pose = Odometry()
                pose.header = msg.header
                pose.pose.pose.orientation.w = 1.
                writer.write(odom_topic, serialize_message(pose), stamp)
            writer.write(cloud_topic, serialize_message(msg), stamp+1)
        del writer
        result = subprocess.run([sys.executable, str(root / "scripts/prepare-vehicle-navigation.py"),
                                 "--bag", str(folder / "bag"), "--vehicle-config", str(vehicle),
                                 "--output", str(folder / "map")], capture_output=True, text=True, timeout=60)
        (folder / "process.log").write_text(result.stdout + result.stderr)
        return folder, result

    def test_rays_create_corridor_keep_wall_and_unknown(self):
        folder, result = self.scene()
        self.assertEqual(result.returncode, 0, result.stderr)
        folder = folder / "map"
        data = json.loads((folder / "occupancy.json").read_text())
        self.assertAlmostEqual(data["floor_plane"]["z_equals_ax_by_c"][2], .13, delta=.01)
        metadata = yaml.safe_load((folder / "map.yaml").read_text())
        grid = np.asarray(Image.open(folder / "map.pgm"))[::-1]
        def region(x0,x1,y0,y1):
            ox,oy,_ = metadata["origin"]
            res = metadata["resolution"]
            return grid[int((y0-oy)/res):int((y1-oy)/res), int((x0-ox)/res):int((x1-ox)/res)]
        self.assertGreater(np.mean(region(-.5,.5,-.5,.5)==254), .95)
        self.assertGreater(np.mean(np.any(region(1.60,1.75,-1,1)==0, axis=1)), .9)
        self.assertTrue(np.all(region(1.8,2.5,-1,1)==205))
        self.assertTrue(np.any(region(.68,.84,-.90,-.70)==0), "Low obstacle was lost at the vertical voxel boundary")
        # Check the exported image/YAML with the production Nav2 loader, including unknown cells.
        source = Path(__file__).resolve().parent / "fixtures/load-occupancy-map.cpp"
        binary = folder.parent / "load-occupancy-map"
        includes = ["-I/opt/ros/humble/include"] + ["-I"+str(p) for p in Path("/opt/ros/humble/include").iterdir() if p.is_dir()]
        subprocess.run(["g++", "-std=c++17", str(source), *includes, "-L/opt/ros/humble/lib",
                        "-Wl,-rpath,/opt/ros/humble/lib", "-lmap_io", "-o", str(binary)], check=True)
        decoded = folder.parent / "nav2-grid.bin"
        subprocess.run([str(binary), str(folder/"map.yaml"), str(decoded)], check=True)
        expected = np.where(grid == 254, 0, np.where(grid == 205, -1, 100)).ravel()
        self.assertTrue(np.array_equal(np.fromfile(decoded, dtype=np.int8), expected),
                        "Nav2 must preserve exported free, occupied and unknown cells")
        # Pass the generated raster through the same vehicle-radius audit as the CLI.
        check = subprocess.run([sys.executable, str(Path(__file__).resolve().parents[1] / "scripts/validation/verify-map-quality.py"),
                                "--map-config", str(folder), "--output", str(folder / "quality.json")], capture_output=True, text=True)
        self.assertEqual(check.returncode, 0, check.stderr)
        quality = json.loads((folder / "quality.json").read_text())
        self.assertTrue(quality["navigation_grid_has_clearance"])
        self.assertGreater(quality["largest_robot_clear_area_m2"], 1.)

    def test_missing_pose_cannot_invent_rays(self):
        folder, result = self.scene(with_pose=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("timestamp-matched odometry", result.stderr)
        self.assertFalse((folder / "map/map.pgm").exists())

    def test_missing_floor_cannot_fill_unknown(self):
        folder, result = self.scene(with_ground=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse((folder / "map/map.pgm").exists())


if __name__ == "__main__":
    unittest.main()
