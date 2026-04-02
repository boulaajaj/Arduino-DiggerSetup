"""Deploy Digger Control sketch via arduino-cli.

Usage:
  python scripts/deploy.py              # use defaults from board.cfg
  python scripts/deploy.py --port COM5  # override port
"""
import argparse
import subprocess
import sys

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

# Defaults — override via command line or edit here when switching boards
DEFAULT_FQBN = 'arduino:renesas_uno:nanor4'
DEFAULT_PORT = 'COM8'
DEFAULT_SKETCH = 'sketches/rc_test'

parser = argparse.ArgumentParser(description='Deploy sketch to Arduino board')
parser.add_argument('--fqbn', default=DEFAULT_FQBN, help='Board FQBN')
parser.add_argument('--port', default=DEFAULT_PORT, help='Serial port')
parser.add_argument('--sketch', default=DEFAULT_SKETCH, help='Sketch directory')
args = parser.parse_args()

print(f'Compiling {args.sketch} for {args.fqbn}...')
result = subprocess.run(
    ['arduino-cli', 'compile', '--fqbn', args.fqbn, args.sketch],
    capture_output=True, text=True
)
print(result.stdout)
if result.returncode != 0:
    print(f'COMPILE ERROR:\n{result.stderr}')
    sys.exit(1)

print(f'\nUploading to {args.port}...')
result = subprocess.run(
    ['arduino-cli', 'upload', '--fqbn', args.fqbn, '-p', args.port, args.sketch],
    capture_output=True, text=True
)
print(result.stdout)
if result.returncode != 0:
    print(f'UPLOAD ERROR:\n{result.stderr}')
    sys.exit(1)

print('\nDone. Open Serial Monitor at 115200 baud.')
