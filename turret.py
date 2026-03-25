#!/usr/bin/env python3
"""
turret.py — Interactive controller for the HackPack turret

Commands:
  L [n]  - pan left  (1 move ≈ 90°)
  R [n]  - pan right
  U [n]  - tilt up
  D [n]  - tilt down
  F      - fire one dart
  H      - home position
  q      - quit
"""

import serial
import time
import threading

PORT  = '/dev/cu.usbserial-110'
BAUD  = 9600
DELAY = 0.8  # seconds between moves

def reader(s):
    """Background thread: print Arduino responses as they arrive."""
    keywords = {b'LEFT', b'RIGHT', b'UP', b'DOWN', b'FIRING', b'HOMING'}
    buf = b''
    while True:
        try:
            byte = s.read(1)
            if not byte:
                continue
            if byte in (b'\n', b'\r'):
                line = buf.strip()
                if line in keywords:
                    print(f'  → {line.decode()}')
                buf = b''
            else:
                buf += byte
        except Exception:
            break

def send(s, cmd, moves):
    for _ in range(moves):
        s.write(cmd.encode())
        time.sleep(DELAY)

def main():
    print(f'Connecting to {PORT}...')
    s = serial.Serial(PORT, BAUD, timeout=1)
    print('Waiting for Arduino to boot...')
    time.sleep(2.5)
    s.reset_input_buffer()
    print('Ready. Type a command (L/R/U/D/F/H or q to quit):\n')

    t = threading.Thread(target=reader, args=(s,), daemon=True)
    t.start()

    while True:
        try:
            line = input('turret> ').strip().upper()
        except (EOFError, KeyboardInterrupt):
            break

        if not line:
            continue
        if line == 'Q':
            break

        parts = line.split()
        cmd   = parts[0]
        moves = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else 1

        if cmd in ('L', 'R', 'U', 'D', 'F', 'H'):
            send(s, cmd, moves)
        else:
            print('  Unknown command. Use L/R/U/D/F/H [count]')

    s.close()
    print('Disconnected.')

if __name__ == '__main__':
    main()
