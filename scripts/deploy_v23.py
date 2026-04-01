"""Deploy sketch to Arduino Nano R4 via arduino-cli.

NOTE: This script was originally for UNO Q (SSH + App Lab). Now uses
arduino-cli for direct compile and upload to Nano R4 over USB.
"""
import subprocess
import sys

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

FQBN = 'arduino:renesas_uno:nanor4'
PORT = 'COM8'
SKETCH_DIR = 'sketches/rc_test'

print(f'Compiling {SKETCH_DIR} for {FQBN}...')
result = subprocess.run(
    ['arduino-cli', 'compile', '--fqbn', FQBN, SKETCH_DIR],
    capture_output=True, text=True
)
print(result.stdout)
if result.returncode != 0:
    print(f'COMPILE ERROR:\n{result.stderr}')
    sys.exit(1)

print(f'\nUploading to {PORT}...')
result = subprocess.run(
    ['arduino-cli', 'upload', '--fqbn', FQBN, '-p', PORT, SKETCH_DIR],
    capture_output=True, text=True
)
print(result.stdout)
if result.returncode != 0:
    print(f'UPLOAD ERROR:\n{result.stderr}')
    sys.exit(1)

print('\nDone. Open Serial Monitor at 115200 baud.')
print('Note: If X.BUS is connected to D0, Serial Monitor will not work.')
print('Disconnect X.BUS from D0 for debugging.')
