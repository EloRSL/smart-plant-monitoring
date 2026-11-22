"""Self-check for common.py, service.py and vision.py. Run inside the venv: python test_common.py"""
import json
import os
import tempfile
import time

import common as c


def test_rules():
    with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as f:
        f.write("1: air_humidity > 50\n2: air_temperature < 15\n\n3: soil_humidity < 300\n")
    rules = c.load_rules(f.name)
    os.unlink(f.name)
    assert [(r[0], r[1], r[3]) for r in rules] == [
        (1, "air_humidity", 50), (2, "air_temperature", 15), (3, "soil_humidity", 300)]

    active = set()
    assert c.triggered(rules, {"air_humidity": 48, "air_temperature": 13}, active) == [2]
    assert c.triggered(rules, {"air_humidity": 48, "air_temperature": 13}, active) == []   # still true: no repeat
    assert c.triggered(rules, {"air_humidity": 51, "air_temperature": 16}, active) == [1]
    assert c.triggered(rules, {"air_humidity": 51, "air_temperature": 13}, active) == [2]  # crossed again


def test_bad_rule():
    """A bad line is skipped and noted in phases_log.txt; the good lines still count."""
    import contextlib
    import io
    c.PHASES = os.path.join(tempfile.gettempdir(), "phases_log_test.txt")
    open(c.PHASES, "w").close()
    with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as f:
        f.write("1: air_humidity is high\n3: soil_humidity < 350\n")
    with contextlib.redirect_stdout(io.StringIO()):  # phase() also prints
        rules = c.load_rules(f.name)
    os.unlink(f.name)
    assert [r[1] for r in rules] == ["soil_humidity"], rules
    assert "air_humidity is high" in open(c.PHASES).read()
    os.unlink(c.PHASES)


class FakePort:
    """Stands in for the UART: hands out the ESP's messages, keeps what the Pi sends."""

    def __init__(self, *msgs):
        self.lines = [b"\x00garbage while the ESP boots\n"] + [c.line(m) for m in msgs]
        self.sent = []

    def readline(self):
        if not self.lines:
            raise EOFError  # the ESP has nothing more to say: ends the endless V0 loop
        return self.lines.pop(0)

    def write(self, data):
        self.sent.append(json.loads(data))

    def flush(self):
        pass


def test_service():
    import service
    c.LOG = os.path.join(tempfile.gettempdir(), "events_log_service_test.txt")
    open(c.LOG, "w").close()
    c.phase = lambda text: None  # keep phases_log.txt and the test output clean
    service.ntp_synced = lambda: False  # no time message, whatever this machine's clock says
    looked = []

    def fake_look(pipeline, size, models):  # no camera here; a sick-looking result
        looked.append((pipeline, size))
        return {"unhealthy_percent": 80}

    service.look = fake_look
    noon = time.mktime((2026, 11, 18, 12, 0, 0, 0, 0, -1))
    night = noon - 10 * 3600

    def batch(seq, now, last, reason="daily"):
        return {"type": "batch", "seq": seq, "reason": reason, "now": now, "synced": True,
                "readings": [{"t": now - 1, "soil_humidity": 300 + seq}], "last": last}

    # V2 by day: an alarm, a batch whose ack got lost (so it comes twice), the last batch.
    port = FakePort({"type": "alarm", "t": noon - 5, "buzz": 3, "sensor": "soil_humidity", "value": 300},
                    batch(0, noon, False), batch(0, noon, False), batch(1, noon, True))
    service.run(2, port)  # returns after bye: from V1 on the Pi powers off
    assert [m["type"] for m in port.sent] == ["ready", "ack", "ack", "ack", "rules", "buzz", "bye"]
    assert [m["seq"] for m in port.sent if m["type"] == "ack"] == [0, 0, 1]
    sensor_rules = port.sent[4]["rules"]  # the 4 sensor lines of configuration.txt; CV rules stay on the Pi
    assert len(sensor_rules) == 4 and all(r["sensor"] in service.SENSORS for r in sensor_rules), sensor_rules
    assert sensor_rules[0] == {"buzz": 3, "sensor": "soil_humidity", "op": "<", "value": 350}
    assert port.sent[5] == {"type": "buzz", "count": 4} and looked == [("advanced", (640, 480))]
    log = [entry.split(": ", 1)[1] for entry in open(c.LOG).read().splitlines()]
    assert log == ["buzzer 3", "soil_humidity = 300", "soil_humidity = 301",
                   "unhealthy_percent = 80", "buzzer 4"], log
    stamp = time.strftime("%a %b %d %H:%M:%S %Z %Y", time.localtime(noon))
    assert f"{stamp}: unhealthy_percent = 80" in open(c.LOG).read()  # ESP clock, not the Pi's

    # The photo cannot be read (the lab helper calls sys.exit()): bye still goes out.
    def broken_look(pipeline, size, models):
        raise SystemExit(1)

    service.look = broken_look
    port = FakePort(batch(5, noon, True))
    service.run(2, port)
    assert [m["type"] for m in port.sent] == ["ready", "ack", "rules", "bye"]
    service.look = fake_look

    # V2 at night: only receives.
    looked.clear()
    port = FakePort(batch(2, night, True, "change"))
    service.run(2, port)
    assert [m["type"] for m in port.sent] == ["ready", "ack", "rules", "bye"] and looked == []

    # V0 photographs every session, also at night, and keeps running after bye.
    port = FakePort(batch(3, night, True))
    try:
        service.run(0, port)
        assert False, "V0 must not return"
    except EOFError:
        pass
    assert looked == [("starter", None)] and port.sent[-1] == {"type": "bye"}

    # Neither clock is real and NTP does not come: only the daily session gets a photo.
    service.NTP_WAIT_S = 0
    looked.clear()
    for reason in ("daily", "change"):
        service.run(2, FakePort(dict(batch(6, 1000, True, reason), synced=False)))
    assert len(looked) == 1

    # NTP comes during the wait (1 s here): the Pi passes the time on to the ESP.
    answers = iter([False, True])
    service.ntp_synced = lambda: next(answers)
    service.NTP_WAIT_S = 1
    port = FakePort(dict(batch(7, 1000, True, "change"), synced=False))
    service.run(2, port)
    assert [m["type"] for m in port.sent][2] == "time" and port.sent[-1] == {"type": "bye"}
    os.unlink(c.LOG)


def test_seja():
    """The stand-in ESP of stanje.py takes the real service code through a whole session."""
    import service
    import stanje
    c.LOG = os.path.join(tempfile.gettempdir(), "events_log_seja_test.txt")
    open(c.LOG, "w").close()
    service.ntp_synced = lambda: False
    service.look = lambda pipeline, size, models: {"unhealthy_percent": 10}
    noon = time.mktime((2026, 11, 18, 12, 0, 0, 0, 0, -1))
    service.run(2, stanje.FakeEsp(stanje.fake_session(noon)))  # returns after bye, like V2
    log = open(c.LOG).read().splitlines()
    os.unlink(c.LOG)
    assert len(log) == 72 * 3 + 1 and log[-1].endswith("unhealthy_percent = 10"), log[-3:]


def test_log():
    c.LOG = os.path.join(tempfile.gettempdir(), "events_log_test.txt")
    open(c.LOG, "w").close()
    c.log("air_humidity = 48")
    line = open(c.LOG).read()
    os.unlink(c.LOG)
    assert line.endswith(": air_humidity = 48\n") and len(line.split()) >= 8, line


def test_log_batch():
    import stanje
    fmt = "%a %b %d %H:%M:%S %Z %Y"
    c.LOG = os.path.join(tempfile.gettempdir(), "events_log_batch_test.txt")
    open(c.LOG, "w").close()

    # Synced: t is the real time. Sent twice (lost ack): acked twice, logged once.
    msg = {"type": "batch", "seq": 5, "now": 1794850983, "synced": True, "last": True,
           "readings": [{"t": 1794850982, "air_temperature": 23, "soil_humidity": 0}]}
    seen = set()
    assert c.log_batch(msg, seen) == b'{"type":"ack","seq":5}\n'
    assert c.log_batch(msg, seen) == b'{"type":"ack","seq":5}\n'
    stamp = time.strftime(fmt, time.localtime(1794850982))
    assert open(c.LOG).read().splitlines() == [
        f"{stamp}: air_temperature = 23", f"{stamp}: soil_humidity = 0"]

    # Not synced: the reading was taken now - t = 600 s before the ESP sent it.
    open(c.LOG, "w").close()
    c.log_batch(dict(msg, seq=6, synced=False, now=1000, readings=[{"t": 400, "air_humidity": 43}]), seen)
    line = open(c.LOG).read()
    assert any(line.startswith(time.strftime(fmt, time.localtime(time.time() - 600 - d))) for d in (0, 1)), line

    # The fake session of stanje.py: 72 readings in two batches, only the second one is last.
    batches = [json.loads(b) for b in stanje.fake_session()]
    assert [len(b["readings"]) for b in batches] == [50, 22] and [b["last"] for b in batches] == [False, True]
    os.unlink(c.LOG)


def test_vision():
    """The lab photographs must give the results the lab handouts expect."""
    import vision as v
    models = v.load_models("starter", "advanced")
    for name, sick in [("pepper_healthy.jpg", False), ("pepper_bacterial_spot.jpg", True),
                       ("potato_healthy.jpg", False), ("potato_early_blight.jpg", True)]:
        r = v.analyze_file(v.ADVANCED / "images" / name, models)
        assert (r["unhealthy_percent"] > 50) == sick, (name, r)
    r = v.analyze_file(v.STARTER / "images" / "pepper_healthy.jpg", models)
    assert r["leaf_count"] >= 1 and r["green_percent"] >= 90, r  # handout: G=0.99
    # Each version loads one pipeline and must get only that pipeline's values.
    photo = v.STARTER / "images" / "pepper_bacterial_spot.jpg"
    assert set(v.analyze_file(photo, v.load_models("advanced"))) == {"unhealthy_percent"}
    assert "unhealthy_percent" not in v.analyze_file(photo, v.load_models("starter"))
    try:
        v.load_models("advnced")
        assert False, "unknown pipeline name must be rejected"
    except ValueError:
        pass


if __name__ == "__main__":
    test_rules()
    test_bad_rule()
    test_log()
    test_log_batch()
    test_service()
    test_seja()
    try:
        test_vision()
    except ImportError as e:
        print("SKIPPED test_vision (activate the venv):", e)
    print("ok")
