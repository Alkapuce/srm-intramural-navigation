#!/usr/bin/env python3
"""Prepare SRM interface parameters; never open devices or change network settings."""

import argparse
from pathlib import Path
from internal.vehicle_profile import prepare

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output", required=True)
parser.add_argument("--host-ip")
parser.add_argument("--lidar-ip")
parser.add_argument("--runtime-root")
args = parser.parse_args()
prepare(Path(__file__).resolve().parents[1], args.output, args.host_ip, args.lidar_ip, args.runtime_root)
print(f"Prepared unverified vehicle parameters in {args.output}; real serial motion remains blocked")
