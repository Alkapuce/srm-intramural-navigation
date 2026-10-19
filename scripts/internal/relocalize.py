#!/usr/bin/env python3
"""Match a stationary Point-LIO scan to the saved map before navigation starts."""
import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.ndimage import distance_transform_edt, maximum_filter
from scipy.signal import fftconvolve
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import yaml

from intramural_mission.proximity import lidar_to_body
from occupancy import fit_floor


def downsample(points, resolution=.08):
    points = np.asarray(points)
    points = points[np.isfinite(points).all(axis=1)]
    _, indices = np.unique(np.floor(points/resolution).astype(np.int64), axis=0, return_index=True)
    return points[indices]


def level(plane):
    a, b, c = plane
    normal = np.array([-a, -b, 1.])
    normal /= np.linalg.norm(normal)
    axis = np.cross(normal, [0., 0., 1.])
    angle = math.atan2(np.linalg.norm(axis), normal[2])
    transform = np.eye(4)
    if angle > 1e-10:
        transform[:3, :3] = Rotation.from_rotvec(axis/np.linalg.norm(axis)*angle).as_matrix()
    transform[2, 3] = -c/math.sqrt(1+a*a+b*b)
    return transform


def apply(points, transform):
    return points @ transform[:3, :3].T + transform[:3, 3]


def match(target, source, target_plane, source_plane):
    """Gravity/floor alignment, full-yaw grid correlation, then trimmed 3D ICP.

    shortcut: a near-horizontal floor is required, as for the map exporter;
    use a full 6D global registration method if the venue includes ramps.
    """
    target_level, source_level = level(target_plane), level(source_plane)
    target, source = apply(target, target_level), apply(source, source_level)
    target = downsample(target[(target[:, 2] > .08) & (target[:, 2] < 1.8)])
    source = downsample(source[(source[:, 2] > .08) & (source[:, 2] < 1.8)])
    if min(len(target), len(source)) < 100:
        raise ValueError('Too few non-ground points for map localization')
    resolution = .15
    origin = np.floor(target[:, :2].min(axis=0)/resolution)*resolution
    cells = np.floor((target[:, :2]-origin)/resolution).astype(int)
    shape = cells.max(axis=0)+1
    if max(shape) > 600 or max(np.ptp(source[:, :2], axis=0)) > 90:
        raise ValueError('Map/scan exceeds the local field registration extent (90 m)')
    grid = np.zeros(shape[::-1], dtype=bool)
    grid[cells[:, 1], cells[:, 0]] = True
    likelihood = np.exp(-.5*(distance_transform_edt(~grid)/1.2)**2)
    tree = cKDTree(target)
    proposals = []
    for yaw in np.arange(-math.pi, math.pi, math.pi/36):
        rotation = Rotation.from_euler('z', yaw).as_matrix()
        rotated = source @ rotation.T
        low = np.floor(rotated[:, :2].min(axis=0)/resolution)*resolution
        indices = np.floor((rotated[:, :2]-low)/resolution).astype(int)
        scan_grid = np.zeros((indices[:, 1].max()+1, indices[:, 0].max()+1))
        scan_grid[indices[:, 1], indices[:, 0]] = 1.
        scores = fftconvolve(likelihood, scan_grid[::-1, ::-1], mode='full')
        peaks = np.where(scores == maximum_filter(scores, size=7), scores, -np.inf)
        for index in np.argpartition(peaks.ravel(), -2)[-2:]:
            row, col = np.unravel_index(index, scores.shape)
            translation = origin-low+resolution*np.array([col-scan_grid.shape[1]+1, row-scan_grid.shape[0]+1])
            transform = np.eye(4)
            transform[:3, :3], transform[:2, 3] = rotation, translation
            distance = tree.query(apply(source, transform))[0]
            proposals.append((float(np.mean(np.minimum(distance, .4))), transform))
    solutions = []
    for _, transform in sorted(proposals, key=lambda item: item[0])[:8]:
        for _ in range(35):
            aligned = apply(source, transform)
            distance, indices = tree.query(aligned)
            keep = distance < min(.4, max(.08, np.quantile(distance, .75)))
            a, b = aligned[keep], target[indices[keep]]
            if len(a) < 50:
                break
            ca, cb = a.mean(axis=0), b.mean(axis=0)
            u, _, vh = np.linalg.svd((a[:, :2]-ca[:2]).T @ (b[:, :2]-cb[:2]))
            rotation = vh.T @ u.T
            if np.linalg.det(rotation) < 0:
                vh[-1] *= -1
                rotation = vh.T @ u.T
            delta = np.eye(4)
            delta[:2, :2] = rotation
            delta[:2, 3] = cb[:2]-rotation@ca[:2]
            delta[2, 3] = cb[2]-ca[2]
            transform = delta@transform
            if np.linalg.norm(delta[:3, 3]) < 1e-4 and abs(rotation[1, 0]) < 1e-4:
                break
        distance = tree.query(apply(source, transform))[0]
        solutions.append((float(np.mean(np.minimum(distance, .4))), transform, distance))
    solutions.sort(key=lambda item: item[0])
    score, transform, distance = solutions[0]
    residual = np.quantile(distance, [.5, .9]).tolist()
    overlap = float(np.mean(distance < .15))
    ambiguous = any(
        other_score < score*1.1+.005 and
        (np.linalg.norm(other[:2, 3]-transform[:2, 3]) > .5 or
         abs(math.atan2((other[:3, :3]@transform[:3, :3].T)[1, 0],
                        (other[:3, :3]@transform[:3, :3].T)[0, 0])) > .25)
        for other_score, other, _ in solutions[1:])
    report = dict(source_points=len(source), map_points=len(target),
        residual_median_p90_m=residual, overlap_within_15cm=overlap, ambiguous=ambiguous,
        passed=residual[0] < .1 and overlap > .65 and not ambiguous)
    return np.linalg.inv(target_level)@transform@source_level, report


def capture(namespace):
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init()
    node = rclpy.create_node('field_relocalization_capture')
    clouds, poses = [], []
    def cloud(msg):
        if msg.header.frame_id != 'camera_init' or msg.is_bigendian:
            raise ValueError('Expected Point-LIO camera_init little-endian cloud')
        fields = {f.name: f for f in msg.fields}
        if any(fields[k].datatype != 7 for k in 'xyz'):
            raise ValueError('Expected float32 XYZ')
        age = node.get_clock().now().nanoseconds*1e-9-msg.header.stamp.sec-msg.header.stamp.nanosec*1e-9
        if abs(age) > 1.:
            raise RuntimeError('Point-LIO scan is stale; check sensor input')
        clouds.append(np.stack([np.ndarray((msg.height, msg.width), dtype='<f4', buffer=msg.data,
            offset=fields[k].offset, strides=(msg.row_step, msg.point_step)).ravel() for k in 'xyz'], axis=1))
    node.create_subscription(PointCloud2, namespace+'/cloud_registered', cloud, qos_profile_sensor_data)
    node.create_subscription(Odometry, namespace+'/aft_mapped_to_init', lambda msg: poses.append(msg.pose.pose), qos_profile_sensor_data)
    print('自动定位：保持静止，采集当前点云；不会发送运动指令。', flush=True)
    try:
        deadline = time.monotonic()+15
        while time.monotonic() < deadline and (len(clouds) < 20 or not poses):
            rclpy.spin_once(node, timeout_sec=.1)
        if len(clouds) < 20 or not poses:
            raise RuntimeError('No complete live scan; first run srm-nav sensors')
        if max(math.dist([p.position.x, p.position.y, p.position.z],
                         [poses[-1].position.x, poses[-1].position.y, poses[-1].position.z]) for p in poses) > .05:
            raise RuntimeError('Vehicle moved during localization; keep stationary and retry')
        start_q, end_q = poses[0].orientation, poses[-1].orientation
        rotations = Rotation.from_quat([[q.x, q.y, q.z, q.w] for q in (start_q, end_q)])
        if (rotations[0].inv()*rotations[1]).magnitude() > .05:
            raise RuntimeError('Vehicle rotated during localization; keep stationary and retry')
        pose = poses[-1]
        odometry = np.eye(4)
        q = pose.orientation
        odometry[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        odometry[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        cloud = np.concatenate(clouds)
        cloud = cloud[np.linalg.norm(cloud-odometry[:3, 3], axis=1) < 12.]
        return downsample(cloud), odometry
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-config', required=True)
    parser.add_argument('--scan', help='Replay a saved localization scan.npz instead of reading sensors')
    args = parser.parse_args()
    folder = Path(args.map_config).resolve()
    root = Path(__file__).resolve().parents[2]
    out = root/'artifacts/localization'/datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    out.mkdir(parents=True)
    c = json.loads((folder/'arena.json').read_text())
    raw = (folder/'prior.pcd').read_bytes()
    header, binary = raw.split(b'DATA binary\n', 1)
    if b'FIELDS x y z\n' not in header or len(binary)%12:
        raise ValueError('Expected exported XYZ prior map')
    target = lidar_to_body(np.frombuffer(binary, dtype='<f4').reshape(-1, 3), c['lidar_pose'])
    if args.scan:
        saved = np.load(args.scan)
        cloud, odometry = saved['cloud'], saved['odometry']
    else:
        cloud, odometry = capture('/'+c['namespace'].strip('/'))
    np.savez_compressed(out/'scan.npz', cloud=cloud, odometry=odometry)
    mount = np.eye(4)
    mount[:3, :3] = Rotation.from_euler('xyz', c['lidar_pose'][3:]).as_matrix()
    mount[:3, 3] = c['lidar_pose'][:3]
    source = apply(cloud, mount)
    plane, _ = fit_floor(source, apply(odometry[None, :3, 3], mount))
    target_plane = json.loads((folder/'occupancy.json').read_text())['floor_plane']['z_equals_ax_by_c']
    transform, report = match(target, source, target_plane, plane)
    body = transform@mount@odometry@np.linalg.inv(mount)
    initial = [*transform[:3, 3].tolist(), *Rotation.from_matrix(transform[:3, :3]).as_euler('xyz').tolist()]
    report.update(map_config=str(folder), init_pose=initial,
        body_pose_xyz_rpy=[*body[:3, 3].tolist(), *Rotation.from_matrix(body[:3, :3]).as_euler('xyz').tolist()],
        prior_sha256=hashlib.sha256(raw).hexdigest(), physical_motion_commanded=False,
        scope='Near-horizontal mapped scene; geometric consistency, not independent pose precision')
    (out/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2), flush=True)
    print('定位记录: '+str(out), flush=True)
    if not report['passed']:
        raise RuntimeError('Scan/map match insufficient or ambiguous; old localization seed was not changed')
    (out/'nav-before.yaml').write_bytes((folder/'nav.yaml').read_bytes())
    (out/'manifest-before.json').write_bytes((folder/'manifest.json').read_bytes())
    params = yaml.safe_load((folder/'nav.yaml').read_text())
    params['small_gicp_relocalization']['ros__parameters']['init_pose'] = initial
    temporary = folder/'nav.yaml.tmp'
    temporary.write_text(yaml.safe_dump(params, sort_keys=False))
    temporary.replace(folder/'nav.yaml')
    manifest = json.loads((folder/'manifest.json').read_text())
    manifest['files_sha256']['nav.yaml'] = hashlib.sha256((folder/'nav.yaml').read_bytes()).hexdigest()
    (folder/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')


if __name__ == '__main__':
    main()
