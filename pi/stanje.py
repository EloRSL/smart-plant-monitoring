"""Pi states for the Monsoon measurement (Vid's "Navodila za meritve porabe Raspberry Pi").

One command per recording; the README "Measuring" section says which file each one goes into.
Each state does its step three times, with 10 s of rest before each and after the last, and
writes every start and end to data/phases_log.txt:

  python stanje.py slika       P3  full-resolution photo                      (V0, V1)
  python stanje.py slika_mala  P3  640x480 photo                              (V2, V3)
  python stanje.py cv          P4  expensive analysis in a fresh Python       (V1)
  python stanje.py cv_v0       P4  expensive analysis, model kept loaded      (V0)
  python stanje.py cv_mali     P4  cheap analysis in a fresh Python           (V2, V3)
  python stanje.py seja        P5  one V2 session run by the real service code, with a stand-in
                                   for the ESP (72 readings, photo, analysis, bye), then poweroff

A V1+ session starts a fresh Python, so it pays all of it: importing the libraries, loading the
model, the analysis. The lines "imported", "model loaded" and "result" split those cv steps.
V0 loads the model once at start, so it pays only the analysis: that is cv_v0.
"""
import subprocess
import sys
import time

import common as c
from common import phase

REPEATS, PAUSE = 3, 10  # as in Vid's "Navodila za meritve porabe Raspberry Pi"
SMALL = (640, 480)  # same photo size as V2/V3 in service.py


def repeat(name, step):
    for i in range(1, REPEATS + 1):
        time.sleep(PAUSE)  # flat idle before each step, so every step is a separate peak
        phase(f"{name} {i} start")
        start = time.monotonic()
        step()
        phase(f"{name} {i} end {time.monotonic() - start:.3f} s")
    time.sleep(PAUSE)  # and after the last: the CPU stays at full speed ~5 s after work, record that too


def photos(size):
    import vision as v  # imported only here: loading OpenCV would slow the other states down

    repeat("slika" if size is None else "slika_mala", lambda: v.wait_photo(v.start_photo(size)))


def cv(pipeline, size):
    import vision as v

    phase("preparation: one photo to analyse")
    v.wait_photo(v.start_photo(size))
    code = ("import common as c; import vision as v; c.phase('  imported'); "
            f"m = v.load_models('{pipeline}'); c.phase('  model loaded'); "
            "c.phase(f'  result {v.analyze_file(v.PHOTO, m)}')")
    repeat("cv" if pipeline == "starter" else "cv_mali",
           lambda: subprocess.run([sys.executable, "-c", code], cwd=c.DIR, check=True))


def cv_v0():
    import vision as v

    phase("preparation: one full photo, the model loaded once (V0 does this at start)")
    v.wait_photo(v.start_photo(None))
    models = v.load_models("starter")
    repeat("cv_v0", lambda: phase(f"  result {v.analyze_file(v.PHOTO, models)}"))


def fake_session(now=None):
    """A daily V1/V2 session as the ESP sends it: 72 readings (one per 20 min), batches of 50."""
    now = int(now or time.time())
    readings = [{"t": now - (71 - k) * 1200, "air_temperature": 22, "air_humidity": 45,
                 "soil_humidity": 500} for k in range(72)]
    return [c.line({"type": "batch", "seq": i, "reason": "daily", "now": now, "synced": True,
                    "readings": part, "last": i == 1})
            for i, part in enumerate([readings[:50], readings[50:]])]


class FakeEsp:
    """Stands in for the ESP32 on the serial port: hands over its lines, ignores the Pi's replies."""

    def __init__(self, lines):
        self.lines = lines

    def readline(self):
        if not self.lines:
            raise EOFError("the stand-in ESP has nothing more to send")
        return self.lines.pop(0)

    def write(self, data):
        pass

    def flush(self):
        pass


def seja():
    import service  # vision is imported inside the session, as in the real service

    c.LOG = c.DATA + "/events_log_test.txt"  # the same work as the real log, without filling it
    noon = time.mktime(time.localtime()[:3] + (12, 0, 0, 0, 0, -1))  # "by day", so it takes the photo
    esp = FakeEsp(fake_session(noon))
    time.sleep(PAUSE)
    phase("seja start")
    service.run(2, esp)
    service.poweroff()


STATES = {
    "slika": lambda: photos(None),
    "slika_mala": lambda: photos(SMALL),
    "cv": lambda: cv("starter", None),
    "cv_v0": cv_v0,
    "cv_mali": lambda: cv("advanced", SMALL),
    "seja": seja,
}

if __name__ == "__main__":
    if len(sys.argv) != 2 or sys.argv[1] not in STATES:
        sys.exit(__doc__)
    STATES[sys.argv[1]]()
