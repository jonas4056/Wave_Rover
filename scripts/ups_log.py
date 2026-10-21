#!/usr/bin/env python3
"""Logt spanning en stroom van de Waveshare UPS Power Module C (INA219, I2C-bus 7, adres 0x41).

Op de Jetson zelf uitvoeren (niet in de container):

    nohup python3 ~/wave_rover/scripts/ups_log.py ~/ups_log.csv 10 &

Stroom > 0: de batterij wordt geladen. Stroom < 0: de batterij levert stroom.
De stroom wordt berekend uit de shuntspanning met een veronderstelde shunt van 0,01 ohm.
"""
import subprocess
import sys
import time

BUS = '7'
ADDR = '0x41'
SHUNT_OHM = 0.01


def read_word(register):
    out = subprocess.run(['i2cget', '-y', BUS, ADDR, register, 'w'],
                         capture_output=True, text=True, check=True).stdout
    raw = int(out, 16)
    return ((raw & 0xFF) << 8) | (raw >> 8)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else 'ups_log.csv'
    interval = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0

    with open(path, 'a', buffering=1) as f:
        f.write('tijd,spanning_V,stroom_A\n')
        while True:
            bus_v = (read_word('0x02') >> 3) * 0.004
            shunt = read_word('0x01')
            if shunt > 32767:
                shunt -= 65536
            current = shunt * 10e-6 / SHUNT_OHM
            f.write(f'{time.strftime("%H:%M:%S")},{bus_v:.3f},{current:.3f}\n')
            time.sleep(interval)


if __name__ == '__main__':
    main()
