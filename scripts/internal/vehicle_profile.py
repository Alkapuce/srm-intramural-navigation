"""Generate device-free configuration using public defaults and selected interface facts."""

import copy
import hashlib
import ipaddress
import json
from pathlib import Path

import yaml


def prepare(root, output, host_ip=None, lidar_ip=None, runtime_root=None):
    root, output = Path(root).resolve(), Path(output).resolve()
    if not any(output.is_relative_to(root / name) for name in ("cache", "artifacts")):
        raise ValueError("Vehicle configuration output must be under project cache or artifacts")
    if output.exists():
        raise ValueError("Output already exists; choose a new directory")
    source = root / "config/vehicle-srm-reference.json"
    profile = json.loads(source.read_text())
    lidar = profile["mid360"]
    host = str(ipaddress.IPv4Address(host_ip or lidar["host_ip"]))
    sensor = str(ipaddress.IPv4Address(lidar_ip or lidar["lidar_ip"]))
    if host == sensor or any(ipaddress.IPv4Address(ip).is_unspecified for ip in (host, sensor)):
        raise ValueError("Distinct nonzero host and lidar IPv4 addresses required")
    if lidar["public_driver_xfer_format"] != 1:
        raise ValueError("Reference generator selects xfer_format=1 for Livox CustomMsg")

    network = {"lidar_summary_info": {"lidar_type": 8}, "MID360": {
        "lidar_net_info": {key + "_port": value for key, value in lidar["lidar_ports"].items()},
        "host_net_info": {}}, "lidar_configs": [{"ip": sensor, "pcl_data_type": 1,
        "pattern_mode": 0, "extrinsic_parameter": dict(zip(
            ("roll", "pitch", "yaw", "x", "y", "z"), lidar["driver_extrinsic_degrees"] + [0, 0, 0]))}]}
    for key, value in lidar["host_ports"].items():
        network["MID360"]["host_net_info"][key + "_ip"] = "" if key == "log_data" else host
        network["MID360"]["host_net_info"][key + "_port"] = value
    if runtime_root:
        runtime = Path(runtime_root).resolve() / output.relative_to(root)
    elif Path("/work").exists():
        runtime = Path("/work") / output.relative_to(root)
    else:
        runtime = root / output.relative_to(root)
    driver = {"/**": {"ros__parameters": {"use_sim_time": False, "xfer_format": 1,
        "multi_topic": 0, "data_src": 0, "publish_freq": lidar["publish_frequency_hz"],
        "output_data_type": 0, "frame_id": lidar["frame_id"],
        "user_config_path": str(runtime / "MID360_config.json"),
        "cmdline_input_bd_code": "livox0000000001"}}}

    public = root / "nav_ws/src/pb2025_sentry_nav/point_lio/config/mid360.yaml"
    point = copy.deepcopy(yaml.safe_load(public.read_text()))
    params = point["/**"]["ros__parameters"]
    params["use_sim_time"] = False
    params["common"].update(lid_topic="/livox/lidar", imu_topic="/livox/imu")
    facts = profile["point_lio"]
    params["preprocess"].update({key: facts[key] for key in ("lidar_type", "scan_line", "timestamp_unit")})
    params["preprocess"]["blind"] = facts["blind_m"]
    params["mapping"].update(imu_time_inte=facts["imu_interval_seconds"], acc_norm=facts["acc_norm"],
        extrinsic_T=facts["lidar_to_imu_translation_m"], extrinsic_R=facts["lidar_to_imu_rotation"])

    calibration = json.loads((root / "config/base-vehicle-template.json").read_text())
    serial = {"/**": {"ros__parameters": {"simulation": False, "device": "",
        "calibration": str(runtime / "base-vehicle.json")}}}
    handoff = {"vehicle_validated": False, "references": profile["references"],
        "vehicle_context": copy.deepcopy(profile["vehicle_context"]),
        "profile_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "public_point_lio_config_sha256": hashlib.sha256(public.read_bytes()).hexdigest(),
        "host_ip": host, "lidar_ip": sensor, "serial": profile["serial"],
        "geometry": profile["geometry"], "extrinsics_measured": False,
        "base_to_lidar_static_tf_arguments": ["--x", str(lidar["base_to_lidar_xyz_m"][0]),
            "--y", str(lidar["base_to_lidar_xyz_m"][1]), "--z", str(lidar["base_to_lidar_xyz_m"][2]),
            "--roll", str(lidar["base_to_lidar_rpy_rad"][0]),
            "--pitch", str(lidar["base_to_lidar_rpy_rad"][1]),
            "--yaw", str(lidar["base_to_lidar_rpy_rad"][2]),
            "--frame-id", "base_link", "--child-frame-id", lidar["frame_id"]]}
    output.mkdir(parents=True)
    for name, value in (("MID360_config.json", network), ("base-vehicle.json", calibration),
                        ("handoff.json", handoff)):
        (output / name).write_text(json.dumps(value, indent=2) + "\n")
    for name, value in (("livox-driver.yaml", driver), ("point-lio.yaml", point), ("serial-bridge.yaml", serial)):
        (output / name).write_text(yaml.safe_dump(value, sort_keys=False))
    return handoff
