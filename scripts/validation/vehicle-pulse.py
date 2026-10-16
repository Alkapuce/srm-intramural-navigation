#!/usr/bin/env python3
"""One bounded commissioning pulse in protocol units, with zero commands before/after."""
import argparse
from datetime import datetime
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import struct
import termios
import time

from intramural_base.protocol import FeedbackParser, command_frame, crc8, crc16
from intramural_base.serial_bridge import open_serial


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("direction",nargs="?",choices=["x","y"],help="Protocol axis; not yet calibrated to vehicle forward/left")
    p.add_argument("--device",default=os.environ.get("INTRAMURAL_SERIAL_DEVICE"),help="Default: the single matching 1a86 USB Single Serial by-id device")
    p.add_argument("--output",help="Default: a new timestamped directory under project artifacts/field/")
    p.add_argument("--axis",choices=["x","y"],help="Alias for positional direction")
    p.add_argument("--value",type=float,default=.02)
    p.add_argument("--seconds",type=float,default=.3)
    p.add_argument("--legacy20",action="store_true",help="Diagnostic only: old sender layout/seq and 5 ms post-send wait, is_recovering=0; default is competition 19-byte frames")
    p.add_argument("--wheels-lifted",action="store_true",help="Confirm all wheels are securely lifted")
    p.add_argument("--stop-available",action="store_true",help="Confirm someone can immediately cut motor power")
    args=p.parse_args()
    if args.direction and args.axis and args.direction!=args.axis:p.error("Conflicting axis arguments")
    args.axis=args.direction or args.axis
    if not args.axis:
        p.print_help();return
    if not args.wheels_lifted or not args.stop_available:
        p.error("Both --wheels-lifted and --stop-available are required for a pulse")
    if not math.isfinite(args.value) or not 0<abs(args.value)<=.1:
        p.error("Protocol magnitude must be in (0, 0.10]; it is not metres/second")
    if not math.isfinite(args.seconds) or not 0<args.seconds<=.5:
        p.error("Pulse duration must be in (0, 0.5] seconds")
    if not args.device:
        candidates=sorted(Path("/dev/serial/by-id").glob("usb-1a86_USB_Single_Serial_*-if00"))
        if len(candidates)!=1:p.error("No unique matching serial device; specify --device explicitly")
        args.device=str(candidates[0])
    device=Path(args.device).resolve(strict=True)
    if not str(device).startswith("/dev/") or not device.is_char_device():
        p.error("Explicit serial character device required")
    root=Path(__file__).resolve().parents[2]
    out=Path(args.output).resolve() if args.output else root/"artifacts/field"/datetime.now().strftime("manual-pulse-%Y%m%d-%H%M%S-%f")
    out.mkdir(parents=True,exist_ok=False)
    print("DEVICE:",args.device,"OUTPUT:",out,flush=True)
    stopped=False
    def stop(signum,frame):
        nonlocal stopped
        stopped=True
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,stop)
    fd=open_serial(str(device));owned=False;seq=1 if args.legacy20 else 0;tx=[];packets=[];raw=bytearray();parser=FeedbackParser()
    error=None
    def send(values):
        nonlocal seq
        frame=command_frame(*values,seq)
        if args.legacy20:
            header=bytes((0xA5,13,0,seq))
            body=header+bytes((crc8(header),))+struct.pack("<fffB",*values,0)
            frame=body+struct.pack("<H",crc16(body))
        count=os.write(fd,frame)
        if count!=len(frame):raise OSError(f"Partial serial frame: {count}/{len(frame)}")
        tx.append({"time":time.monotonic(),"values":values,"seq":seq,"hex":frame.hex()})
        seq=(seq+1)&255
    def phase(seconds,values):
        end=time.monotonic()+seconds;due=time.monotonic()
        while time.monotonic()<end:
            if stopped:raise InterruptedError("Operator interrupted")
            now=time.monotonic()
            if now-due>.05:raise RuntimeError("Sender missed deadline; returning to zero")
            if now>=due:
                send(values)
                due=time.monotonic()+.005 if args.legacy20 else max(due+.01,now+.003)
            try:
                data=os.read(fd,4096)
                if data:raw.extend(data);packets.extend(parser.feed(data))
            except BlockingIOError:pass
            time.sleep(.0005)
    zero=[0.0,0.0,0.0];pulse=zero.copy();pulse[0 if args.axis=="x" else 1]=args.value
    try:
        fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB);fcntl.ioctl(fd,termios.TIOCEXCL);owned=True
        print("PROTOCOL:","legacy20 (diagnostic only)" if args.legacy20 else "competition19",flush=True)
        print("ZERO warmup: 3 seconds",flush=True);phase(3,zero)
        if not packets:raise RuntimeError("No valid MCU feedback; pulse cancelled")
        print("PULSE",pulse,"for",args.seconds,"seconds",flush=True);phase(args.seconds,pulse)
        print("ZERO",flush=True);phase(1,zero)
    except Exception as exc:
        error=str(exc)
    finally:
        try:
            if owned:
                for _ in range(10):send(zero);time.sleep(.005 if args.legacy20 else .01)
        finally:
            os.close(fd)
            report={"device":str(device),"axis":args.axis,"protocol_value":args.value,"requested_pulse_seconds":args.seconds,
                    "protocol":"legacy20" if args.legacy20 else "competition19","command_bytes":20 if args.legacy20 else 19,
                    "send_timing":"5ms_after_send" if args.legacy20 else "100hz_deadline",
                    "transmitted_frames":len(tx),"nonzero_frames":sum(any(t["values"]) for t in tx),
                    "valid_feedback_packets":len(packets),"error":error,"port_closed":True,
                    "physical_response_verified":False,"link_loss_stop_verified":False,"tx":tx}
            (out/"result.json").write_text(json.dumps(report,indent=2)+"\n");(out/"received.bin").write_bytes(raw)
    print(json.dumps({k:v for k,v in report.items() if k!="tx"},indent=2))
    if error:raise SystemExit(1)


if __name__=="__main__":main()
