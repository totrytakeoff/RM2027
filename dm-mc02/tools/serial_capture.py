#!/usr/bin/env python3
"""Capture MC-02 UART text without sending data; requires pyserial."""
import argparse
import json
import time
from pathlib import Path
import serial


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device',default='/dev/serial/by-id/usb-Horco_Horco_CMSIS-DAP_305410062834-if02')
    parser.add_argument('--baud',type=int,default=115200)
    parser.add_argument('--duration',type=float,default=60)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.duration<=0: parser.error('duration must be positive')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    data=bytearray(); start=time.monotonic(); last_print=start
    with serial.Serial(args.device,args.baud,timeout=0.1,exclusive=True) as port:
        port.reset_input_buffer()
        print('Serial capture ready',args.device,args.baud,flush=True)
        while time.monotonic()-start<args.duration:
            data.extend(port.read(max(1,port.in_waiting)))
            if time.monotonic()-last_print>=10:
                print('Captured',len(data),'bytes',flush=True); last_print=time.monotonic()
    args.output.write_bytes(data)
    text=data.decode('utf-8',errors='replace')
    args.output.with_suffix('.json').write_text(json.dumps({'device':args.device,'baud':args.baud,
        'duration_s':time.monotonic()-start,'bytes':len(data),'lines':text.count('\n'),
        'decode_errors':text.count('\ufffd'),'chassis_rows':sum(x.startswith('CH ') for x in text.splitlines())},indent=2)+'\n')
    print('Saved',args.output,flush=True)


if __name__=='__main__': main()
