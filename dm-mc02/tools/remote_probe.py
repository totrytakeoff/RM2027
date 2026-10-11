#!/usr/bin/env python3
"""Live ET08/DBUS observation, without halting or writing the MCU.

Host polling is not an atomic firmware API snapshot. Edge visibility depends
on the control task consuming its mailbox and host sampling latency.
"""
import argparse
import json
import time
from pathlib import Path
from board_probe import Probe, metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--elf', type=Path, default=Path('build/h7-debug/board_bringup.elf'))
    parser.add_argument('--duration', type=float, default=60)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.duration <= 0: parser.error('duration must be positive')
    meta = metadata(args.elf)
    probe = Probe()
    samples = []
    start = time.monotonic()
    last = None
    try:
        while time.monotonic() - start < args.duration:
            remote = probe.typed(meta['remote'])
            parser_state = probe.typed(meta['remote_parser'])
            uart = probe.typed(meta['serial'][0])
            sample = {'host_elapsed_s': time.monotonic() - start,
                      'timer_us': probe.words(0x40000c24, 1)[0],
                      'remote': remote, 'stats': parser_state['stats'],
                      'raw_channels': parser_state['raw_channels'], 'uart': uart}
            samples.append(sample)
            state = (remote['data_valid'], remote['failsafe'], tuple(remote['switches'][:4]))
            if state != last or len(samples) % 100 == 0:
                print(json.dumps({'elapsed_s': round(sample['host_elapsed_s'], 2),
                    'valid': remote['data_valid'], 'failsafe': remote['failsafe'],
                    'sequence': remote['sample_sequence'], 'axes': remote['axis_raw'][:7],
                    'switches': remote['switches'][:4], 'uart_errors': uart['stats']['rx_errors']}), flush=True)
                last = state
            time.sleep(0.01)
    finally:
        probe.close()
    valid = [s for s in samples if s['remote']['data_valid']]
    first, final = samples[0], samples[-1]
    keys = ('frames', 'good_frames', 'bad_frames', 'frame_lost', 'failsafe', 'discontinuities')
    summary = {'firmware': meta['firmware'], 'duration_s': time.monotonic() - start,
        'samples': len(samples), 'valid_samples': len(valid),
        'stats_delta': {k: final['stats'][k] - first['stats'][k] for k in keys},
        'uart_delta': {k: final['uart']['stats'][k] - first['uart']['stats'][k]
                       for k in ('rx_bytes', 'rx_errors', 'rx_dropped', 'restarts', 'generation')},
        'axis_ranges': [[min(s['remote']['axis_raw'][i] for s in valid),
                         max(s['remote']['axis_raw'][i] for s in valid)] for i in range(16)] if valid else [],
        'switch_positions': [sorted({s['remote']['switches'][i] for s in valid}) for i in range(4)],
        'link_transitions': [{'host_elapsed_s': s['host_elapsed_s'],
                             'valid': s['remote']['data_valid'], 'failsafe': s['remote']['failsafe']}
                            for i, s in enumerate(samples) if i == 0 or
                            (s['remote']['data_valid'], s['remote']['failsafe']) !=
                            (samples[i-1]['remote']['data_valid'], samples[i-1]['remote']['failsafe'])],
        'final': final}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({'summary': summary, 'samples': samples}, indent=2) + '\n')
    print('Saved', args.output, flush=True)


if __name__ == '__main__': main()
