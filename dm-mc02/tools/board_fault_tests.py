#!/usr/bin/env python3
"""Deliberate SWD fault injection on the bringup image, with reset cleanup.

Requires the Debug ELF and an OpenOCD Tcl server. No external devices needed.
These tests change live RAM/NVIC state; do not use on an operating robot.
"""
import argparse
import json
import time
from pathlib import Path
from board_probe import Probe, metadata, snapshot


def write(probe, address, value, width=32):
    probe.command(f'write_memory {address:#x} {width} {{{value:#x}}}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=('serial-recover', 'serial-timeout', 'task-stall'), required=True)
    parser.add_argument('--elf', type=Path, default=Path('build/h7-debug/board_bringup.elf'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    meta = metadata(args.elf)
    probe = Probe()
    result = {'case': args.case, 'firmware': meta['firmware'], 'injection': [], 'samples': []}
    try:
        probe.command('reset run')
        time.sleep(2)
        result['before'] = snapshot(probe, meta)
        start = time.monotonic()
        if args.case.startswith('serial-'):
            if args.case == 'serial-timeout':
                # STM32H723 DMA1 Stream4 IRQ 15, remote RX DMA abort callback.
                write(probe, 0xe000e180, 1 << 15)
                result['injection'].append('Disable DMA1_Stream4_IRQn (15) in NVIC')
            write(probe, meta['injection']['serial_fault'][0], 1, 8)
            result['injection'].append('Set remote endpoint fault=true; actual HAL DMA abort follows')
            duration = 1.5
        else:
            result['original_callback'] = probe.words(meta['injection']['control_callback'], 1)[0]
            # AAPCS r0 receives low 32 bits of now_us; the delay exceeds IWDG timeout.
            write(probe, meta['injection']['control_callback'], meta['injection']['delay_us'])
            result['injection'].append('Replace CONTROL callback with RmTime_DelayUs(now_us low32)')
            duration = 3.0
        while time.monotonic() - start < duration:
            sample = snapshot(probe, meta)
            sample['host_elapsed_s'] = time.monotonic() - start
            result['samples'].append(sample)
            time.sleep(0.025)
        result['after'] = result['samples'][-1]
        endpoint = result['after']['serial'][0]['stats']
        if args.case == 'serial-recover':
            passed = endpoint['restarts'] > result['before']['serial'][0]['stats']['restarts'] and endpoint['online'] and not endpoint['quarantined']
        elif args.case == 'serial-timeout':
            passed = endpoint['abort_timeouts'] == 1 and endpoint['quarantined'] and not endpoint['online']
            result['dma1_stream4_cr'] = probe.words(0x40020070, 1)[0]
            result['nvic_iser0'] = probe.words(0xe000e100, 1)[0]
            result['uart5_cr1_cr2_cr3'] = probe.words(0x40005000, 3)
            passed = passed and not result['dma1_stream4_cr'] & 1
            passed = passed and not result['nvic_iser0'] & (1 << 15)
            passed = passed and not result['uart5_cr1_cr2_cr3'][0] & 1
        else:
            samples = [result['before']] + result['samples']
            result['observed_resets'] = sum(b['timer_us'] < a['timer_us'] for a, b in zip(samples, samples[1:]))
            result['heartbeat_gate_observed'] = any(s['gate']['reasons'] & (1 << 11) for s in result['samples'])
            passed = result['observed_resets'] == 1 and result['heartbeat_gate_observed']
            passed = passed and bool(result['after']['board']['reset_flags'] & (1 << 26))
            # A moved board may still be in bounded static calibration after reboot.
            passed = passed and result['after']['ins']['state'] != 2
            passed = passed and all(t['run_count'] > 100 for t in result['after']['tasks'])
        result['outputs_disabled'] = all(not s['gate']['permitted'] and
            not s['gpiob_odr'] & (1 << 1) and not s['gpioc_odr'] & 0xe000 for s in result['samples'])
        result['passed'] = bool(passed and result['outputs_disabled'])
    finally:
        probe.command('reset run')
        probe.close()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'case': args.case, 'passed': result.get('passed'), 'output': str(args.output)}))
    if not result.get('passed'): raise SystemExit(1)


if __name__ == '__main__': main()
