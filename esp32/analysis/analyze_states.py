#!/usr/bin/env python3
"""Reads the Monsoon CSV files from the ESP32 power tests and writes the results
(current, power, wake-up and measurement energy for each state) to a JSON file."""
import argparse
import json
import os
import sys

import powerlib as pl

# csv files we expect in the data folder (names like we saved them in the lab)
SLEEP = {'S01': 15, 'S01_2': 15, 'S02': 15, 'S09a': 15, 'S09b_80': 15,
         'S02_3V3': 20, 'S01_voda': 15, 'S02_voda': 15}
FLAT = ['S03', 'S06', 'S07_80']
# name: (period s, threshold mA, window after onset s, threshold for the first onset)
BURSTS = {'S04': (2.999, 38.3, 0.4, None), 'S04_80': (2.999, 33.0, 0.4, None),
          'S05_80': (5.978, 33.0, 1.3, 60), 'S04_80_voda': (2.999, 37.3, 0.4, None),
          'S05_80_voda': (5.978, 33.5, 1.3, 60)}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--data', required=True, help='folder with the S*.csv captures')
    ap.add_argument('--out', default='results.json', help='output JSON (default: results.json)')
    ap.add_argument('--cache', default=None, help='folder for .npz copies that speed up re-runs')
    args = ap.parse_args()

    def load(name):
        return pl.load_capture(os.path.join(args.data, name + '.csv'), args.cache)

    def have(name):
        ok = os.path.exists(os.path.join(args.data, name + '.csv'))
        if not ok:
            print(f'skipping {name}: {name}.csv not found', file=sys.stderr)
        return ok

    r = {}
    for name, thr in SLEEP.items():
        if have(name):
            r[name] = pl.sleep_with_wakes(*load(name), thr)
    for name in FLAT:
        if have(name):
            r[name] = pl.flat(*load(name))
    for name, (period, thr, after, first) in BURSTS.items():
        if have(name):
            r[name] = pl.periodic_bursts(*load(name), period, thr, after, first)
    if 'S04_80_voda' in r and 'S04_80' in r:
        # in water the probe current is too noisy to find the burst,
        # so we take the burst from the dry run
        r['S04_80_voda']['burst_ms'] = r['S04_80']['burst_ms']
        r['S04_80_voda']['extra_mJ'] = r['S04_80']['extra_mJ']
    if have('S08'):
        r['S08'] = pl.buzzer(*load('S08'))
    if have('S10'):
        r['S10'] = pl.light_sleep(*load('S10'))

    with open(args.out, 'w') as f:
        json.dump(r, f, indent=1)
    for k, v in r.items():
        print(k, {a: round(b, 4) if isinstance(b, float) else b for a, b in v.items()})


if __name__ == '__main__':
    main()
