"""Helper functions for the Monsoon captures (used by analyze_states.py):
loading the csv, sleep level, wake-up energy, bursts."""
import os

import numpy as np


def load_capture(path, cache_dir=None):
    """Return time (s), current (mA) and power (mW) arrays of one PowerTool CSV export.
    A .npz copy is kept in cache_dir, because parsing a 20 MB CSV takes a while."""
    cache = None
    if cache_dir:
        os.makedirs(cache_dir, exist_ok=True)
        cache = os.path.join(cache_dir, os.path.splitext(os.path.basename(path))[0] + '.npz')
        if os.path.exists(cache) and os.path.getmtime(cache) >= os.path.getmtime(path):
            z = np.load(cache)
            return z['t'], z['i'], z['p']
    with open(path) as f:
        header = f.readline().strip().split(',')
    ci = header.index('Main Avg Current (mA)')
    pi = header.index('Main Avg Power (mW)')
    d = np.loadtxt(path, delimiter=',', skiprows=1, usecols=(0, ci, pi))
    t, i, p = d[:, 0], d[:, 1], d[:, 2]
    if cache:
        np.savez(cache, t=t, i=i, p=p)
    return t, i, p


def segments(t, i, thr, merge=0.05, minlen=0.0005):
    """Index ranges where the current is above thr; gaps shorter than merge (s) are joined."""
    dt = t[1] - t[0]
    hi = i > thr
    e = np.diff(hi.astype(int))
    st = list(np.where(e == 1)[0] + 1)
    en = list(np.where(e == -1)[0] + 1)
    if hi[0]:
        st = [0] + st
    if hi[-1]:
        en = en + [len(i)]
    m = []
    for s, x in zip(st, en):
        if m and (s - m[-1][1]) * dt < merge:
            m[-1] = (m[-1][0], x)
        else:
            m.append((s, x))
    return [(s, x) for s, x in m if (x - s) * dt >= minlen]


def volt(i, p):
    """Supply voltage from P / I (the export has no voltage column)."""
    return float(np.median(p / np.maximum(i, 1e-3)))


def sleep_with_wakes(t, i, p, thr):
    """Flat sleep level plus the energy of each wake-up above that level.
    Samples before the supply was switched on (current near 0) are ignored."""
    dt = t[1] - t[0]
    m = segments(t, i, thr, .05)
    full = [(s, x) for s, x in m if s > 250 and x < len(i) - 250]
    mask = np.ones(len(i), bool)
    for s, x in m:
        mask[max(0, s - 1000):x + 1000] = False
    mask &= i > 1
    pb = p[mask].mean()
    du = [(x - s) * dt * 1000 for s, x in full]
    ex = [p[s - 250:x + 250].sum() * dt - pb * (x - s + 500) * dt for s, x in full]
    return dict(U=volt(i, p), I_mA=float(i[mask].mean()), P_mW=float(pb),
                I_std=float(i[mask].std()), n=len(full),
                wake_ms=float(np.mean(du)), wake_ms_sd=float(np.std(du)),
                wake_extra_mJ=float(np.mean(ex)), wake_extra_sd=float(np.std(ex)))


def flat(t, i, p):
    """Average of a capture that holds one steady state."""
    blocks = [i[j:j + 5000].mean() for j in range(0, len(i) - 5000, 5000)]
    return dict(U=volt(i, p), I_mA=float(i.mean()), P_mW=float(p.mean()),
                I_std_1s=float(np.std(blocks)))


def periodic_bursts(t, i, p, period, thr, win_after, first_thr=None):
    """Energy of each periodic burst (one measurement) above the idle level between bursts.
    The first burst is found where the current first exceeds first_thr (default thr)."""
    dt = t[1] - t[0]
    s0 = t[np.argmax(i > (thr if first_thr is None else first_thr))]
    ons = []
    while True:
        a = int((s0 - 0.2) / dt)
        b = int((s0 + win_after) / dt)
        if b >= len(i):
            break
        hi = np.where(i[a:b] > thr)[0]
        if len(hi):
            ons.append((a + hi[0], a + hi[-1]))
        s0 += period
    mask = np.ones(len(i), bool)
    for on, off in ons:
        mask[max(0, on - 1500):off + 1500] = False
    pb = p[mask].mean()
    ex = [p[on - 250:off + 250].sum() * dt - pb * (off - on + 500) * dt for on, off in ons]
    du = [(off - on) * dt * 1000 for on, off in ons]
    return dict(U=volt(i, p), idle_I_mA=float(i[mask].mean()), idle_P_mW=float(pb),
                n=len(ons), burst_ms=float(np.mean(du)), burst_ms_sd=float(np.std(du)),
                extra_mJ=float(np.mean(ex)), extra_sd=float(np.std(ex)))


def buzzer(t, i, p):
    """Buzzer on/off levels of power test 08 and their difference."""
    on = i > 45
    off = (~on) & (i < 40)
    r = dict(U=volt(i, p), on_I_mA=float(i[on].mean()), off_I_mA=float(i[off].mean()),
             on_P_mW=float(p[on].mean()), off_P_mW=float(p[off].mean()))
    r['buzzer_I_mA'] = r['on_I_mA'] - r['off_I_mA']
    r['buzzer_P_mW'] = r['on_P_mW'] - r['off_P_mW']
    return r


def light_sleep(t, i, p):
    """Power test 10: mean of the 3 s light-sleep levels, sensors off (A) and on (C)."""
    dt = t[1] - t[0]
    low = i < 20
    e = np.diff(low.astype(int))
    st = list(np.where(e == 1)[0] + 1)
    en = list(np.where(e == -1)[0] + 1)
    if low[0]:
        st = [0] + st
    if low[-1]:
        en = en + [len(i)]
    lows = [(s, x) for s, x in zip(st, en) if (x - s) * dt > 2.5]
    lv = [(i[s + 500:x - 500].mean(), p[s + 500:x - 500].mean()) for s, x in lows]
    a = [v for v in lv if v[0] < 9.95]
    c = [v for v in lv if v[0] >= 9.95]
    return dict(U=volt(i, p), A_I_mA=float(np.mean([v[0] for v in a])),
                A_P_mW=float(np.mean([v[1] for v in a])),
                C_I_mA=float(np.mean([v[0] for v in c])),
                C_P_mW=float(np.mean([v[1] for v in c])), nA=len(a), nC=len(c))
