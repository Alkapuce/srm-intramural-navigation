"""Adapt the organizer's fixed-joint description to the existing task simulator."""

import copy
import hashlib
import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory
from intramural_mission.proximity import lidar_to_body


def build_robot(root, c, public_robot, out):
    source = root / "simulation/models/sentry_robot_cylinder.xacro"
    pose = c["lidar_pose"]
    if len(pose) != 6 or not all(math.isfinite(v) for v in pose):
        raise ValueError("Six finite lidar installation coordinates required")
    urdf = ET.fromstring(xacro.process_file(str(source), mappings={
        "xyz": " ".join(map(str, pose[:3])),
        "rpy": " ".join(map(str, pose[3:])),
    }).toxml())
    urdf.set("name", c["namespace"])
    body = urdf.find("link[@name='base_link']")
    cylinder = body.find("collision/geometry/cylinder")
    body_radius = float(cylinder.get("radius"))
    radius = body_radius
    for i in range(1, 5):
        joint = urdf.find(f"joint[@name='base_to_wheel{i}']")
        if joint.get("type") != "fixed":
            raise ValueError("Task-level drive requires the supplied fixed wheel model")
        origin = joint.find("origin")
        xyz = list(map(float, origin.get("xyz").split()))
        rpy = list(map(float, origin.get("rpy").split()))
        axis = lidar_to_body(np.array([[0., 0., 1.]]), [0., 0., 0., *rpy])[0]
        # Supplied wheel cylinder axes point radially; include both end rims.
        if abs(np.dot(axis[:2], xyz[:2]) / math.hypot(*xyz[:2])) < 1 - 1e-6:
            raise ValueError("Wheel layout changed; recompute the collision envelope")
        wheel = urdf.find(f"link[@name='wheel_{i}']/collision/geometry/cylinder")
        radius = max(radius, math.hypot(math.hypot(*xyz[:2]) + float(wheel.get("length"))/2,
                                        float(wheel.get("radius"))))
    if not radius <= c["robot_radius"] <= radius + .02:
        raise ValueError("Configured navigation radius must conservatively enclose the supplied model")
    if not math.isclose(c["robot_length"], 2*body_radius) or not math.isclose(c["robot_width"], 2*body_radius):
        raise ValueError("Parking dimensions must match the organizer body diameter")

    mesh = Path(get_package_share_directory("pb2025_robot_description")) / "resource/models/mid360/meshes/mid360.dae"
    if not mesh.is_file():
        raise ValueError("Pinned public MID-360 visual is missing")
    visual = urdf.find("link[@name='livox_frame']/visual")
    visual.find("origin").attrib.update(xyz="0 0 0", rpy=f"0 0 {math.pi}")
    visual.find("geometry/mesh").attrib.update(filename=mesh.as_uri(), scale="1 1 1")
    conversion_input = out / "organizer-expanded.urdf"
    ET.ElementTree(urdf).write(conversion_input, encoding="unicode", xml_declaration=True)
    robot = ET.fromstring(subprocess.check_output(["ign", "sdf", "-p", str(conversion_input)], text=True))
    robot.set('version', '1.9')
    model = robot.find("model")
    chassis = model.find("link[@name='base_link']")
    chassis.set("name", "base_footprint")
    for frame in model.findall("frame"):
        if frame.get("attached_to") == "base_link":
            frame.set("attached_to", "base_footprint")
    ET.SubElement(model, "frame", name="base_link", attached_to="base_footprint")
    # shortcut: frictionless planar drive; real traction/motor dynamics need measurements.
    for collision in chassis.findall("collision"):
        surface = ET.SubElement(collision, "surface")
        friction = ET.SubElement(ET.SubElement(surface, "friction"), "ode")
        ET.SubElement(friction, "mu").text = "0"
        ET.SubElement(friction, "mu2").text = "0"
    # The reference scan origin is inside the simplified cylinder, not a measured optical shell.
    for visual in chassis.findall('visual'):
        ET.SubElement(visual, 'visibility_flags').text = '1'

    link = copy.deepcopy(public_robot.find("link[@name='front_mid360']"))
    for child in link.findall("visual"):
        link.remove(child)
    joint = copy.deepcopy(public_robot.find("joint[@name='front_livox_joint']"))
    joint.find("parent").text = "base_footprint"
    joint.find("pose").set("relative_to", "base_footprint")
    joint.find("pose").text = " ".join(map(str, pose))
    link.find("sensor[@type='gpu_lidar']/pose").text = "0 0 0 0 0 0"
    link.find("sensor[@type='gpu_lidar']/ray/scan/horizontal/samples").text = str(c["lidar_samples"])
    ET.SubElement(link.find("sensor[@type='gpu_lidar']/ray"), 'visibility_mask').text = '4'
    model.extend([joint, link])

    ET.SubElement(urdf, "link", name="base_footprint")
    alias = ET.SubElement(urdf, "joint", name="footprint_to_base", type="fixed")
    ET.SubElement(alias, "parent", link="base_footprint")
    ET.SubElement(alias, "child", link="base_link")
    ET.SubElement(alias, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(urdf, "link", name="front_mid360")
    alias = ET.SubElement(urdf, "joint", name="livox_to_sim_sensor", type="fixed")
    ET.SubElement(alias, "parent", link="livox_frame")
    ET.SubElement(alias, "child", link="front_mid360")
    ET.SubElement(alias, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.indent(urdf)
    ET.ElementTree(urdf).write(out / "robot.urdf", encoding="unicode", xml_declaration=True)
    return robot, {"source": str(source.relative_to(root)),
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "body_radius_m": body_radius, "collision_envelope_radius_m": radius,
        "navigation_radius_m": c["robot_radius"], "lidar_pose": pose,
        "public_lidar_mesh": str(mesh), "drive": "ideal planar velocity; fixed wheels, zero ground friction",
        "lidar_visibility": "exclude organizer coarse body visuals from GPU rays; preserve GUI visuals and collisions"}
