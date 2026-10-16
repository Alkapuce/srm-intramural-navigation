#!/usr/bin/env python3
"""Send only zero command frames; inspect feedback without enabling the ROS base bridge."""
import argparse
from collections import Counter
import fcntl
import json
import os
from pathlib import Path
import termios
import time

from intramural_base.protocol import FeedbackParser, command_frame
from intramural_base.serial_bridge import open_serial, write_frame

p=argparse.ArgumentParser(description=__doc__)
p.add_argument("--device",required=True)
p.add_argument("--output",required=True)
args=p.parse_args()
device=Path(args.device).resolve(strict=True)
if not str(device).startswith("/dev/") or not device.is_char_device():
    raise ValueError("Explicit serial character device required")
out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=False)
fd=open_serial(str(device));owned=False
parser=FeedbackParser();packets=[];received=bytearray();sent=0;times=[]
try:
    fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
    fcntl.ioctl(fd,termios.TIOCEXCL)
    owned=True
    start=time.monotonic();deadline=start+8;next_frame=start
    while time.monotonic()<deadline:
        now=time.monotonic()
        if now>=next_frame:
            write_frame(fd,command_frame(0.0,0.0,0.0,sent&255))
            sent+=1;times.append(now);next_frame=now+.01
        try:
            chunk=os.read(fd,4096)
            if chunk:
                received.extend(chunk);packets.extend(parser.feed(chunk))
        except BlockingIOError:
            pass
        time.sleep(.0005)
finally:
    # Every frame in this diagnostic, including cleanup, is fixed zero.
    try:
        for _ in range(3 if owned else 0):
            write_frame(fd,command_frame(0.0,0.0,0.0,sent&255));sent+=1;time.sleep(.01)
    finally:os.close(fd)
report={"device":str(device),"baud":115200,"mode":"8N1, no flow control","command_bytes":19,
        "zero_frames_sent":sent,"nonzero_frames_sent":0,"normal_send_hz":(len(times)-1)/(times[-1]-times[0]),
        "maximum_send_interval_s":max(b-a for a,b in zip(times,times[1:])),"received_bytes":len(received),
        "valid_feedback_packets":len(packets),"feedback_commands":dict(Counter(p.cmd_id for p in packets)),
        "discarded_bytes":parser.discarded,"buffered_bytes":len(parser.buffer),
        "latest_feedback":[p.__dict__ for p in packets[-4:]],"mcu_speed_command_ack_verified":False,
        "link_loss_stop_tested":False,"physical_motion_commanded":False}
(out/"received.bin").write_bytes(received)
(out/"result.json").write_text(json.dumps(report,indent=2)+"\n")
print(json.dumps(report,indent=2))
if not packets:raise SystemExit("No valid feedback; do not mark MCU communication verified")
