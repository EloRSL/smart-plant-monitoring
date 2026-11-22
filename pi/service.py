"""The Pi service: talks to the ESP32 as in Vid's "Navodila za Pi", built on his pi_test.py.

    python service.py 0    V0: always on; every session gets a full-resolution photo and the
                               expensive analysis (leafs-starter)
    python service.py 1    V1: the same photo and analysis, but powers off after the session
    python service.py 2    V2 and V3: 640x480 photo and the cheap analysis (leafs-advanced),
    python service.py 3        then powers off. The Pi side of V3 is V2; the ESP adds "change".

From V1 on the photo is taken only by day; a session at night only receives.
plant@.service starts it at boot. Every step goes to phases_log.txt, like in stanje.py.

One session (Pi -> ESP after the last batch): ack, time (only if NTP synced), rules,
photo + analysis, buzz (a CV rule from configuration.txt became true), bye.
"""
import json
import subprocess
import sys
import time

import common as c

SENSORS = ("air_temperature", "air_humidity", "soil_humidity")  # the only rule names the ESP knows
VERSIONS = {  # version: (analysis pipeline, photo size, power off after bye)
    0: ("starter", None, False),
    1: ("starter", None, True),
    2: ("advanced", (640, 480), True),
    3: ("advanced", (640, 480), True),
}
DAY = (8, 18)  # shortcut: fixed hours, not sunrise/sunset; enough for "photo only by day"
SILENCE_S = 30  # the ESP gives up after 4 tries of 5 s, so after this a session is over
NTP_WAIT_S = 15  # + photo and analysis (5 s at most) stays inside the ESP's 30 s wait for bye


def send(port, msg):
    port.write(c.line(msg))


def ntp_synced():
    """The Pi has no battery clock. Only trust its time once NTP has synced (Vid's check)."""
    try:
        out = subprocess.run(["timedatectl", "show", "-p", "NTPSynchronized", "--value"],
                             capture_output=True, text=True, timeout=5).stdout.strip()
        return out == "yes"
    except (OSError, subprocess.SubprocessError):
        return False


def by_day(batch, ntp):
    """Is it day? By the ESP clock if it is real, else by the Pi clock if NTP set it, else None."""
    if batch["synced"]:
        when = batch["now"]
    elif ntp:
        when = time.time()
    else:
        return None
    return DAY[0] <= time.localtime(when).tm_hour < DAY[1]


def look(pipeline, size, models):
    """Photo + analysis, as stanje.py slika/cv measure them. `models` is filled on first use."""
    import vision as v  # only here: loading OpenCV at start would delay the first ready at boot

    c.phase("photo start")
    v.wait_photo(v.start_photo(size))
    c.phase("photo end")
    if not models:
        models.update(v.load_models(pipeline))
        c.phase("model loaded")
    values = v.analyze_file(v.PHOTO, models)
    c.phase(f"cv end {values}")
    return values


def finish(port, version, batch, rules, active, models):
    """Everything after the last ack, up to and including bye."""
    pipeline, size, _ = VERSIONS[version]
    ntp = ntp_synced()
    # Right after a boot the Pi clock is the one saved at shutdown. If the ESP has no real time
    # either, wait for NTP so the ESP gets it; it keeps it, so this happens once, not every session.
    for _ in range(NTP_WAIT_S if not ntp and not batch["synced"] else 0):
        time.sleep(1)
        ntp = ntp_synced()
        if ntp:
            break
    if ntp:
        send(port, {"type": "time", "unix": int(time.time())})
    send(port, {"type": "rules", "rules": [
        {"buzz": n, "sensor": name, "op": op, "value": limit}
        for n, name, op, limit in rules if name in SENSORS]})

    day = by_day(batch, ntp)
    if version == 0 or day or (day is None and batch["reason"] == "daily"):
        try:
            values = look(pipeline, size, models)
        # SystemExit too: the lab's read_and_resize() calls sys.exit() on an unreadable photo.
        except (Exception, SystemExit) as e:  # a broken camera must not keep the Pi on
            c.phase(f"photo or analysis failed: {e!r}")
        else:
            when = c.real_time(batch["now"], batch)  # the ESP clock, if it has the real time
            for name, value in values.items():
                c.log(f"{name} = {value:g}", when)
            for n in c.triggered(rules, values, active):
                send(port, {"type": "buzz", "count": n})
                c.log(f"buzzer {n}", when)
            c.phase("events_log written")
    else:
        c.phase("night: no photo")

    send(port, {"type": "bye"})
    port.flush()  # bye must be on the wire before the Pi powers off
    c.phase("bye")


def poweroff():
    """From V1 on, after bye. Started at boot there is no terminal to type a sudo password into,
    so this needs the sudoers line from the README setup; without it the Pi stays on."""
    c.phase("poweroff")
    if subprocess.run(["sudo", "-n", "poweroff"]).returncode:
        c.phase("poweroff failed: sudo wants a password, add the sudoers line from the README")


def run(version, port):
    """Serve sessions: V0 forever, from V1 on return after the first bye."""
    rules = c.load_rules()  # V0: restart the service after editing configuration.txt
    seen, active, models, alarms = set(), set(), {}, []
    in_session, last_ready, last_rx, rx = False, None, 0.0, b""
    c.phase(f"service start V{version}")
    while True:
        if in_session and time.monotonic() - last_rx > SILENCE_S:
            c.phase("session abandoned by the ESP")
            in_session = False
        if not in_session and (last_ready is None or time.monotonic() - last_ready >= 2):
            send(port, {"type": "ready"})
            if last_ready is None:
                c.phase("first ready")
            last_ready = time.monotonic()

        rx += port.readline()
        if not rx.endswith(b"\n"):
            continue  # timeout or half a line: keep reading
        text, rx = rx, b""
        try:
            msg = json.loads(text)
        except ValueError:
            continue  # noise on the line, e.g. while the ESP boots
        kind = msg.get("type") if isinstance(msg, dict) else None
        if kind not in ("alarm", "batch"):
            continue
        in_session, last_rx = True, time.monotonic()

        if kind == "alarm":
            alarms.append(msg)  # its time can only be converted with the next batch's clock
            continue
        c.phase(f"batch {msg['seq']} received")
        for alarm in alarms:
            c.log(f"buzzer {alarm['buzz']}", c.real_time(alarm["t"], msg))
        alarms.clear()
        port.write(c.log_batch(msg, seen))
        if msg["last"]:
            c.phase(f"last ack, reason {msg['reason']}, stats {msg.get('stats')}")
            finish(port, version, msg, rules, active, models)
            in_session, last_ready = False, time.monotonic()  # next ready in 2 s
            if VERSIONS[version][2]:
                return


if __name__ == "__main__":
    if len(sys.argv) != 2 or sys.argv[1] not in ("0", "1", "2", "3"):
        sys.exit(__doc__)
    run(int(sys.argv[1]), c.open_uart(timeout=2))  # wakes every 2 s when idle
    poweroff()
