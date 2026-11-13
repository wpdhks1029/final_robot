"""Sequential software-only EKF comparisons; does not launch hardware."""
import argparse
from datetime import datetime
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parent
CASES = ('finite_vx_covariance', 'vy_constraint', 'short_warmup')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-software-comparison', action='store_true')
    args = parser.parse_args()
    if not args.run_software_comparison:
        print('PLAN ONLY, no ROS processes. Cases:', ', '.join(CASES))
        return 0
    if not (ROOT / 'ekf_replay_variants_v1/manifest.json').is_file():
        raise SystemExit('Prepared diagnostic cases missing')
    out = ROOT / 'reports' / datetime.now().strftime('ekf_comparison_%Y%m%d_%H%M%S_%f')
    out.mkdir(parents=True, exist_ok=False)
    for case in CASES:
        print(f'CASE={case}: software replay, about 30 seconds...', flush=True)
        with (out / f'{case}.txt').open('w') as log:
            result = subprocess.run(
                [sys.executable, '-B', str(ROOT / 'leader_ekf_offline_replay.py'),
                 '--run-software-replay', '--variant', case],
                stdout=log, stderr=subprocess.STDOUT)
        text = (out / f'{case}.txt').read_text()
        for line in text.splitlines():
            if line.startswith(('REPLAY_INPUT_CHECK', 'REPLAY COMPARISON INVALID',
                                'REPLAY FAILED', 'RESULT_DIR=')):
                print(line, flush=True)
        print(f'CASE_EXIT_CODE={result.returncode}', flush=True)
        if result.returncode:
            print(text[-3000:], flush=True)
            print(f'COMPARISON_DIR={out}', flush=True)
            return result.returncode
    print(f'COMPARISON_DIR={out}', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
