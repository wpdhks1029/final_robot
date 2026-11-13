"""Replay scheduling gaps against the real motor core, with memory serial only.

Async cases MODEL removal of publication blocking; they do not execute DDS or
prove the deployed network's timing. Scheduler stalls remain in both modes.
"""
import hashlib
import json
from pathlib import Path
import random
import sys

ACTIVE = Path.home() / 'follower_diagnostics'
sys.path.insert(0, str(ACTIVE))
from follower_motor_candidate import MotorCore, Config
from follower_motor_mock import MemoryTransport
from test_motor_candidate import Clock


def replay(gap, timeout, reply=True):
    clock = Clock()
    serial = MemoryTransport()
    serial.feedback_enabled = reply
    core = MotorCore(serial, dry_run=False, allow_motion=False,
                     config=Config(response_timeout=timeout), clock=clock)
    core.tick(clock.ros)
    assert core.waiting_since is not None
    clock.now += gap
    core.tick(clock.ros)
    result = dict(fault=core.fault, valid_feedback=core.valid_count)
    return result


def loss_detection(timeout):
    clock = Clock()
    serial = MemoryTransport()
    serial.feedback_enabled = False
    core = MotorCore(serial, dry_run=False, allow_motion=False,
                     config=Config(response_timeout=timeout), clock=clock)
    start = clock.now
    for tick in range(100):
        clock.now = start + tick * .02
        core.tick(clock.ros)
        if core.fault:
            assert core.fault == 'RESPONSE_TIMEOUT'
            return round(clock.now-start, 6)
    raise AssertionError('response loss was not detected')


def main():
    recorded_gaps = (.303127479, .318044921, .389192208, .393733141)
    report = dict(kind='MEMORY_SERIAL_AND_SCHEDULING_MODEL_ONLY',
                  hardware_opened=False, seed=20261111,
                  source_sha256=hashlib.sha256((ACTIVE/'follower_motor_candidate.py').read_bytes()).hexdigest(),
                  assumptions=['Async removes modeled publication blocking only.',
                               'Serial reply is already queued, as in the supplied stall traces.',
                               'OS scheduler stalls are not removed by async publishing.'])
    variants = {}
    for mode, timeout in (('sync_0_3', .3), ('sync_0_4', .4),
                          ('async_model_0_3', .3), ('async_model_0_4', .4)):
        asynchronous = mode.startswith('async')
        rows = []
        for gap in recorded_gaps:
            actual_gap = .02 if asynchronous else gap
            result = replay(actual_gap, timeout)
            rows.append(dict(block_s=gap, modeled_tick_gap_s=actual_gap, **result))
        rng = random.Random(20261111)
        failures = 0
        for _ in range(10000):
            publish_block = rng.uniform(.001, 1.2)
            residual_gap = rng.uniform(.015, .08)
            gap = residual_gap if asynchronous else residual_gap+publish_block
            failures += replay(gap, timeout)['fault'] is not None
        scheduler = [replay(gap, timeout)['fault'] for gap in (.565, 1.116)]
        assert all(scheduler), 'scheduler stalls must still fail closed'
        variants[mode] = dict(recorded_replay=rows, random_cases=10000,
                              random_faults=failures,
                              disarmed_response_loss_detection_s=loss_detection(timeout),
                              scheduler_stall_faults=scheduler)
    assert variants['async_model_0_3']['random_faults'] == 0
    assert variants['sync_0_4']['random_faults'] > 0
    report['variants'] = variants
    report['recommendation'] = 'ASYNCHRONOUS publication with unchanged 0.3s watchdogs; hardware timing still requires observation.'
    out = Path(__file__).parents[2] / 'motor_publish_stall_comparison.json'
    out.write_text(json.dumps(report, indent=2)+'\n')
    print('REPORT='+str(out))
    for name, result in variants.items():
        print(name, 'random_faults=', result['random_faults'],
              'response_loss_detection_s=', result['disarmed_response_loss_detection_s'])


if __name__ == '__main__':
    main()
