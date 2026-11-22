"""Code shared by the Pi service and the measurement states: rules, logs, the ESP32 protocol.

The photo analysis is in vision.py.
"""
import json
import operator
import os
import re
import time
from datetime import datetime

DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(DIR, "configuration.txt")
DATA = os.path.join(DIR, "data")  # everything the programs write: the logs and the last photo
os.makedirs(DATA, exist_ok=True)
LOG = os.path.join(DATA, "events_log.txt")
PHASES = os.path.join(DATA, "phases_log.txt")

OPS = {"<": operator.lt, ">": operator.gt, "<=": operator.le, ">=": operator.ge}


# ---------- configuration.txt ----------

def load_rules(path=CONFIG):
    """Parse lines like '3: soil_humidity < 350' into [(buzzes, name, op, limit)].

    A bad line is skipped and noted in phases_log.txt (as in Vid's pi_test.py): on a Pi without
    a screen, a crash would silently stop every session.
    """
    rules = []
    with open(path) as f:
        for line in f:
            m = re.fullmatch(r"\s*(\d+)\s*:\s*(\w+)\s*([<>]=?)\s*(-?\d+(?:\.\d+)?)\s*", line)
            if m:
                rules.append((int(m[1]), m[2], m[3], float(m[4])))
            elif line.strip():
                phase(f"configuration.txt: line ignored: {line.strip()!r}")
    return rules


def triggered(rules, values, active):
    """Return the buzz counts of rules that just BECAME true.

    `active` (a set, kept by the caller) remembers which rules are already true,
    so a condition buzzes once when it is crossed and not again on every sample.
    """
    out = []
    for i, (buzzes, name, op, limit) in enumerate(rules):
        if name not in values:
            continue
        if OPS[op](values[name], limit):
            if i not in active:
                active.add(i)
                out.append(buzzes)
        else:
            active.discard(i)
    return out


# ---------- events_log.txt and phases_log.txt ----------

def log(text, when=None):
    """Append 'Mon Oct 21 14:04:30 CEST 2024: text'. `when` = unix time it happened (default now)."""
    stamp = time.strftime("%a %b %d %H:%M:%S %Z %Y", time.localtime(when))
    with open(LOG, "a") as f:
        f.write(f"{stamp}: {text}\n")


def phase(text):
    """One line in phases_log.txt: wall time, seconds since boot, what happened.
    Used to cut the Monsoon trace into phases."""
    line = f"{datetime.now():%Y-%m-%d %H:%M:%S.%f}"[:-3] + f" up={time.monotonic():.3f} {text}"
    print(line, flush=True)
    with open(PHASES, "a") as f:
        f.write(line + "\n")


# ---------- JSON protocol of the ESP firmware ("Navodila za Pi") ----------

def open_uart(timeout):
    import serial  # imported here so the self-test runs on a laptop without pyserial
    return serial.Serial("/dev/serial0", 115200, timeout=timeout)


def line(msg):
    """One protocol message: compact JSON and a newline, exactly as Vid's tested pi_test.py sends it."""
    return json.dumps(msg, separators=(",", ":")).encode() + b"\n"


def real_time(t, batch):
    """ESP time -> unix time. Not synced: t counts from ESP power-on, the batch's `now` is its clock now."""
    return t if batch["synced"] else time.time() - (batch["now"] - t)


def log_batch(msg, seen):
    """Write the readings of one batch to events_log.txt and return the ack line to send.

    A batch sent again (same seq and readings, because our ack was lost) is acked but not
    logged twice. `seen` is a set kept by the caller.
    shortcut: `seen` lives in memory only, so a batch resent in the NEXT session (after a
    poweroff) is logged twice; store the last key in a file if that shows up in the log.
    """
    key = (msg["seq"], json.dumps(msg["readings"]))
    if key not in seen:
        seen.add(key)
        for reading in msg["readings"]:
            for name, value in reading.items():
                if name != "t":
                    log(f"{name} = {value:g}", real_time(reading["t"], msg))
    return line({"type": "ack", "seq": msg["seq"]})
