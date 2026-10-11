#!/usr/bin/env python3
"""Read running MC-02 diagnostics through OpenOCD Tcl; never halts the CPU.

The stm32h7x target config enables cacheable AP accesses. No UART cable is needed.
Reset is performed only with --reset; all other target operations are reads.
"""
import argparse
import hashlib
import json
import socket
import struct
import subprocess
import tempfile
import time
from pathlib import Path


GDB_METADATA = r'''
python
import gdb, json
def value(expr): return int(gdb.parse_and_eval(expr))
def addr(expr): return value('&' + expr)
def node(t):
    t=t.strip_typedefs()
    n={'size':int(t.sizeof)}
    if t.code==gdb.TYPE_CODE_STRUCT:
        n['fields']={f.name:{'offset':int(f.bitpos//8),'node':node(f.type)} for f in t.fields()}
    elif t.code==gdb.TYPE_CODE_ARRAY:
        lo,hi=t.range(); n['count']=hi-lo+1; n['element']=node(t.target())
    elif t.code==gdb.TYPE_CODE_FLT: n['float']=True
    elif t.code==gdb.TYPE_CODE_BOOL: n['bool']=True
    elif t.code==gdb.TYPE_CODE_INT: n['signed']=str(t).startswith(('int','signed','short','long','char'))
    return n
def typed(expr):
    v=gdb.parse_and_eval(expr)
    return {'addr':int(v.address),'node':node(v.type)}
m={}
for key,expr in {'board':"'board.c'::board_info",'tasks':"'runtime.c'::stats",
    'ins':"'bringup.c'::ins_snapshot",'raw':"'bringup.c'::imu_snapshot",
    'gate':"'output_gate.c'::gate",'fault':'rm_fault_record',
    'remote':"'bringup.c'::remote_snapshot",'remote_parser':"'bringup.c'::remote"}.items(): m[key]=typed(expr)
try: m['chassis']=typed("'chassis_app.c'::published")
except gdb.error: pass
m['can']=[typed("'can.c'::buses[%d].stats"%i) for i in range(3)]
m['serial']=[]
for i in range(4):
    expr="'serial.c'::endpoints[%d]"%i
    v=gdb.parse_and_eval(expr); n=node(v.type); start=n['fields']['stats']['offset']
    n={'size':n['size']-start,'fields':{k:{'offset':f['offset']-start,'node':f['node']}
        for k,f in n['fields'].items() if f['offset']>=start}}
    m['serial'].append({'addr':int(v.address)+start,'node':n})
m['logs']={'head':addr("'logging.c'::head"),'ring':addr("'logging.c'::ring"),
    'dropped':addr("'logging.c'::dropped"),'capacity':64}
m['injection']={'control_callback':addr("'runtime.c'::callbacks.tick[4]"),
    'delay_us':value('(unsigned int)RmTime_DelayUs') | 1,
    'serial_fault':[addr("'serial.c'::endpoints[%d].fault"%i) for i in range(4)],
    'serial_generation':[addr("'serial.c'::endpoints[%d].stats.generation"%i) for i in range(4)]}
print('RM_METADATA='+json.dumps(m))
end
'''


def metadata(elf):
    with tempfile.NamedTemporaryFile(mode='w', suffix='.gdb') as script:
        script.write(GDB_METADATA); script.flush()
        output=subprocess.check_output(['arm-none-eabi-gdb','-q','-batch',str(elf),
                                        '-x',script.name],text=True)
    meta=json.loads(next(line[12:] for line in output.splitlines()
                         if line.startswith('RM_METADATA=')))
    meta['firmware']={'elf':str(elf),'sha256':hashlib.sha256(Path(elf).read_bytes()).hexdigest()}
    return meta


class Probe:
    def __init__(self, host='localhost', port=6666):
        self.socket=socket.create_connection((host,port),timeout=5)
        self.socket.settimeout(20)

    def command(self, text):
        self.socket.sendall(text.encode()+b'\x1a')
        result=b''
        while not result.endswith(b'\x1a'):
            chunk=self.socket.recv(65536)
            if not chunk: raise RuntimeError('OpenOCD disconnected')
            result+=chunk
        return result[:-1].decode()

    def words(self, address, count):
        # AP access may briefly fail while a deliberate watchdog reset is in flight.
        for attempt in range(5):
            reply=self.command(f'read_memory {address:#x} 32 {count}')
            try:
                values=[int(x,0) for x in reply.split()]
                if len(values)==count: return values
            except ValueError:
                pass
            if attempt<4: time.sleep(0.02)
        raise RuntimeError(f'SWD read at {address:#x} failed: {reply}')

    def data(self, address, length):
        return b''.join(struct.pack('<I',x) for x in self.words(address,(length+3)//4))[:length]

    def typed(self, item):
        return decode(self.data(item['addr'],item['node']['size']),item['node'])

    def close(self): self.socket.close()


def decode(data,node):
    if 'fields' in node:
        return {key:decode(data[field['offset']:],field['node']) for key,field in node['fields'].items()}
    if 'element' in node:
        size=node['element']['size']
        return [decode(data[i*size:],node['element']) for i in range(node['count'])]
    if node.get('float'): return struct.unpack('<f' if node['size']==4 else '<d',data[:node['size']])[0]
    value=int.from_bytes(data[:node['size']],'little',signed=node.get('signed',False))
    return bool(value) if node.get('bool') else value


def snapshot(probe,meta):
    sample={key:probe.typed(meta[key]) for key in ('board','tasks','ins','raw','gate','fault','remote','remote_parser')}
    if 'chassis' in meta: sample['chassis']=probe.typed(meta['chassis'])
    sample['can']=[probe.typed(x) for x in meta['can']]
    sample['serial']=[probe.typed(x) for x in meta['serial']]
    for endpoint in sample['serial']:
        endpoint['stats']['online']=endpoint['started'] and not endpoint['fault'] and not endpoint['aborting'] and not endpoint['stats']['quarantined']
    sample['timer_us']=probe.words(0x40000c24,1)[0]
    sample['cfsr'],sample['hfsr']=probe.words(0xe000ed28,2)
    sample['ccr']=probe.words(0xe000ed14,1)[0]
    sample['mpu_ctrl']=probe.words(0xe000ed94,1)[0]
    sample['dhcsr']=probe.words(0xe000edf0,1)[0]
    sample['gpiob_odr']=probe.words(0x58020414,1)[0]
    sample['gpioc_odr']=probe.words(0x58020814,1)[0]
    sample['log_dropped']=probe.words(meta['logs']['dropped'],1)[0]
    return sample


def monitor(probe,meta,duration,output):
    start=time.monotonic(); samples=[]; events=[]; last_head=0; resets=0; last_timer=None
    while time.monotonic()-start<duration:
        sample=snapshot(probe,meta); sample['host_elapsed_s']=time.monotonic()-start
        if last_timer is not None and sample['timer_us']<last_timer:
            resets+=1; last_head=0
        last_timer=sample['timer_us']; samples.append(sample)
        head=probe.words(meta['logs']['head'],1)[0]
        if head<last_head: last_head=0
        if head!=last_head:
            raw=probe.words(meta['logs']['ring'],4*meta['logs']['capacity'])
            for index in range(max(last_head,head-meta['logs']['capacity']),head):
                offset=(index%meta['logs']['capacity'])*4
                stamp,event,a,b=raw[offset:offset+4]
                events.append({'host_elapsed_s':sample['host_elapsed_s'],'index':index,
                               'time_ms':stamp,'event':event,'a':a,'b':b})
            last_head=head
        if len(samples)==1 or len(samples)%10==0:
            print(json.dumps({'elapsed_s':round(sample['host_elapsed_s'],1),'timer_us':sample['timer_us'],
                'ins_state':sample['ins']['state'],'ins_sequence':sample['ins']['sequence'],
                'resets':resets,'task_misses':[t['deadline_misses'] for t in sample['tasks']]}),flush=True)
        delay=min(1.0,max(0,duration-(time.monotonic()-start)))
        if delay: time.sleep(delay)
    summary={'firmware':meta['firmware'],'duration_s':time.monotonic()-start,'samples':len(samples),'observed_resets':resets,
        'task_max_runtime_us':[max(s['tasks'][i]['max_runtime_us'] for s in samples) for i in range(6)],
        'task_stack_min_words':[min(s['tasks'][i]['stack_free_words'] for s in samples) for i in range(6)],
        'final':samples[-1] if samples else None}
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps({'summary':summary,'events':events,'samples':samples},indent=2)+'\n')
    print('Saved',output,flush=True)
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--elf',type=Path,default=Path('build/h7-debug/board_bringup.elf'))
    parser.add_argument('--duration',type=float,default=60)
    parser.add_argument('--reset',action='store_true')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.duration<=0: parser.error('duration must be positive')
    meta=metadata(args.elf); probe=Probe()
    try:
        if args.reset: probe.command('reset run')
        monitor(probe,meta,args.duration,args.output)
    finally: probe.close()


if __name__=='__main__': main()
