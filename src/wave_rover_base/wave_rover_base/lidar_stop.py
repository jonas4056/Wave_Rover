"""Zet de motor van de RPLidar C1 stil.

Wordt de driver abrupt gestopt (crash, herstart van de container), dan blijft de motor
van de C1 draaien. Dit stuurt STOP (A5 25) en motorsnelheid 0 (A5 A8 02 00 00 + checksum).
De module-manager voert dit uit na het stoppen van de lidarmodule en bij het opstarten.
"""
import sys
import time

import serial

STOP = bytes([0xA5, 0x25])


def motor_speed(rpm):
    payload = bytes([0x02, rpm & 0xFF, rpm >> 8])
    checksum = 0
    for b in bytes([0xA5, 0xA8]) + payload:
        checksum ^= b
    return bytes([0xA5, 0xA8]) + payload + bytes([checksum])


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/lidar'
    try:
        with serial.Serial(port, 460800, timeout=0.2) as s:
            s.write(STOP)
            time.sleep(0.05)
            s.write(motor_speed(0))
            time.sleep(0.1)
    except serial.SerialException as e:
        print(f'Lidar niet bereikbaar op {port}: {e}', file=sys.stderr)
        return 1
    print('Lidarmotor gestopt')
    return 0


if __name__ == '__main__':
    sys.exit(main())
