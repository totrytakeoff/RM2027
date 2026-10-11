#!/usr/bin/env python3
"""Debug-only early startup stall and real FPU HardFault board tests.

Temporarily redirects PC; the FPU test also disables MPU/D-Cache for RAM code.
Every exit requests reset run to restore the production configuration.
"""
import argparse
import json
import subprocess
import time
from pathlib import Path
from board_probe import Probe, metadata, monitor


def gdb_run(elf, output, script):
    path = output.with_suffix('.gdb')
    path.write_text('set pagination off\nset confirm off\ntarget remote :3333\n' + script)
    run = subprocess.run(['arm-none-eabi-gdb', '-q', '-batch', str(elf), '-x', str(path)],
                         capture_output=True, text=True, timeout=30)
    output.with_suffix('.gdb.txt').write_text(run.stdout + run.stderr)
    if run.returncode: raise RuntimeError(run.stdout + run.stderr)
    return run.stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=('early-stall', 'fpu-fault', 'serial-start'), required=True)
    parser.add_argument('--elf', type=Path, default=Path('build/h7-debug/board_bringup.elf'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    meta = metadata(args.elf)
    probe = Probe()
    result = {'case': args.case, 'firmware': meta['firmware']}
    try:
        if args.case == 'serial-start':
            gdb_run(args.elf, args.output, '''
monitor reset halt
thbreak HAL_UARTEx_ReceiveToIdle_DMA
continue
return (HAL_StatusTypeDef)1
detach
''')
            probe.command('resume')
            result['monitor'] = monitor(probe, meta, 3, args.output.with_suffix('.monitor.json'))
            final = result['monitor']['final']
            remote = final['serial'][0]['stats']
            result['passed'] = (remote['restart_failures'] == 1 and remote['restarts'] == 1 and
                                remote['online'] and final['ins']['valid'])
        elif args.case == 'early-stall':
            gdb_run(args.elf, args.output, '''
monitor reset halt
thbreak *ExitRun0Mode
continue
printf "EARLY_PC=%#x\\n", $pc
x/5wx 0x58004800
set $pc = ((unsigned int)&Default_Handler & ~1U)
detach
''')
            probe.command('resume')
            result['monitor'] = monitor(probe, meta, 24, args.output.with_suffix('.monitor.json'))
            flags = result['monitor']['final']['board']['reset_flags']
            result['passed'] = bool(flags & (1 << 26)) and result['monitor']['final']['ins']['valid']
            # TIM5 is reset throughout the early hang, then starts on the watchdog reboot.
        else:
            obj = args.output.with_suffix('.o')
            binary = args.output.with_suffix('.bin')
            subprocess.run(['arm-none-eabi-as', '-o', str(obj), 'tools/fpu_fault_probe.s'], check=True)
            subprocess.run(['arm-none-eabi-objcopy', '-O', 'binary', str(obj), str(binary)], check=True)
            disassembly = subprocess.check_output(['arm-none-eabi-objdump', '-d', str(obj)], text=True)
            args.output.with_suffix('.disassembly.txt').write_text(disassembly)
            fault_site = int(next(line.split()[0] for line in disassembly.splitlines()
                                 if '<RmTest_FpuFaultSite>:' in line), 16)
            result['expected_pc'] = 0x2404f000 + fault_site
            handler = subprocess.check_output(['arm-none-eabi-objdump', '-d', '--disassemble=record', str(args.elf)], text=True)
            wait_pc = int(next(line.split(':')[0].strip() for line in handler.splitlines()
                               if '\twfi' in line), 16)
            # Unallocated AXI tail, XN MPU override only for this test; FPU context on PSP.
            text = gdb_run(args.elf, args.output, f'''
monitor reset run
python import time; time.sleep(2)
monitor halt
set $primask = 1
set {{unsigned int}}0xe000ed94 = 0
set {{unsigned int}}0xe000ed14 = ({{unsigned int}}0xe000ed14 & ~0x10000U)
restore {binary} binary 0x2404f000
thbreak *RmFault_FromException
set $pc = 0x2404f000
continue
printf "EXC_RETURN=%#x\\n", $r1
printf "FRAME_SP=%#x\\n", $r0
x/26wx $r0
thbreak *{wait_pc:#x}
continue
python
import json
v=gdb.parse_and_eval('rm_fault_record')
fault={{f.name:int(v[f.name]) for f in v.type.fields() if f.name!='frame'}}
fault['frame']=[int(v['frame'][i]) for i in range(8)]
def word(a): return int(gdb.parse_and_eval('*(unsigned int*)%d'%a))
print('RM_CAPTURE='+json.dumps({{'fault':fault,'gpiob_odr':word(0x58020414),
 'gpioc_odr':word(0x58020814),'can_cccr':[word(a+0x18) for a in (0x4000a000,0x4000a400,0x4000d400)]}}))
end
detach
''')
            result['exc_return'] = int(next(line.split('=')[1] for line in text.splitlines()
                                           if line.startswith('EXC_RETURN=')), 16)
            result.update(json.loads(next(line[11:] for line in text.splitlines()
                                          if line.startswith('RM_CAPTURE='))))
            probe.command('resume')
            result['monitor'] = monitor(probe, meta, 3, args.output.with_suffix('.monitor.json'))
            fault = result['fault']
            result['passed'] = (result['exc_return'] & 0x10 == 0 and
                fault['magic'] == 0x524d4641 and fault['reason'] == 0xf003 and
                fault['frame'][6] == result['expected_pc'] and fault['cfsr'] & (1 << 16) != 0)
            result['passed'] = result['passed'] and result['monitor']['final']['ins']['valid']
            result['passed'] = result['passed'] and bool(result['monitor']['final']['board']['reset_flags'] & (1 << 26))
            result['passed'] = result['passed'] and all(cccr & 1 for cccr in result['can_cccr'])
    finally:
        probe.command('reset run')
        probe.close()
        args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'case': args.case, 'passed': result.get('passed'), 'output': str(args.output)}))
    if not result.get('passed'): raise SystemExit(1)


if __name__ == '__main__': main()
