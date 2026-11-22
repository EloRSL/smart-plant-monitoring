# Raspberry Pi side

The ESP32 measures, checks the sensor rules and buzzes. The Pi receives the measurements over
UART, writes `events_log.txt`, photographs the plant, analyses the photo, sends the ESP32 the time
and the rules from `configuration.txt`, and (from V1 on) powers itself off. The protocol is Vid's
("Navodila za Pi: komunikacija z ESP32"); `service.py` is built on his `pi_test.py`.

| File | What it is |
|---|---|
| `service.py` | The Pi program: `python service.py 0` to `3` for the version |
| `plant@.service` | Starts `service.py` at boot, for V1 to V3 |
| `common.py` | Shared logic: rules, logs, protocol messages |
| `vision.py` | Photo analysis: the pipelines of the two leaf labs |
| `configuration.txt` | Buzzer rules, format from the project instructions |
| `stanje.py` | Monsoon measurement: one command per recording (see Measuring) |
| `requirements.txt` | Python packages, installed in a venv as in the labs |
| `test_common.py` | Self-check: `python test_common.py` |
| `data/` | Everything the programs write: `events_log.txt` (the plant log), `phases_log.txt` (time of every step), `last_photo.jpg` (newest photo, overwritten each time) |
| `.venv/` (hidden) | The installed Python packages; never edit |

`../leafs-starter` and `../leafs-advanced` are the two class leaf labs, unchanged: `vision.py` loads their
models. They are course material, so they are not in this repository: put them next to `pi/`.

## What each version does on the Pi

| Version | Pi between sessions | Photo | Analysis |
|---|---|---|---|
| V0 | always on, service runs all the time | full resolution, every session | expensive (leafs-starter) |
| V1 | off; the ESP wakes it, it powers off after `bye` | full resolution, only by day | expensive (leafs-starter) |
| V2 | as V1 | 640x480, only by day | cheap (leafs-advanced) |
| V3 | as V2; the ESP also wakes it on a big change (`change`) | as V2 | as V2 |

"By day" is 8:00 to 18:00, judged by the ESP clock if it has the real time, otherwise by the
Pi clock if NTP set it. With neither, only the `daily` session (12:00 to 16:00) gets a photo.
A session at night (`full` or `change`) only receives.

One session, as the Pi sees it:

1. Sends `{"type":"ready"}` every 2 s until something arrives.
2. Receives `alarm` lines (the ESP buzzed: logged as `buzzer N`) and `batch` lines (measurements:
   logged one line per value), and acks every batch. A batch sent again is acked, not logged twice.
3. After the last batch: `time` (only if NTP is synced), `rules` (the sensor lines of
   `configuration.txt`), then the photo and the analysis, then `buzz` if a CV rule just became
   true, then `bye`.
4. From V1 on: `sudo poweroff`.

Photo + analysis take at most about 5 s on the Pi (measured with `stanje.py`); the ESP waits
30 s for `bye`. If the camera or the analysis fails, the Pi still sends `bye` (and powers off),
and the error is in `phases_log.txt`.

Every step goes to `phases_log.txt` with `up=` seconds since boot, as Vid asked, so a Monsoon
trace of a real session can be cut into phases. The `stats` the ESP sends in its last batch are on
the `last ack` line there.

## configuration.txt

    3: soil_humidity < 350
    2: air_temperature > 25
    2: air_temperature < 8
    1: soil_humidity > 900
    4: yellow_percent > 30
    4: unhealthy_percent > 50

- Rules about `air_temperature`, `air_humidity` and `soil_humidity` are sent to the ESP, which
  checks them on every measurement and buzzes itself.
- The other rules are about the photo analysis and are checked on the Pi. They use 4 beeps, a
  number no sensor rule uses, so you can tell them apart by ear (Vid's suggestion).
  `yellow_percent` acts in V0 and V1 (expensive analysis), `unhealthy_percent` in V2 and V3
  (cheap analysis); a rule about a value the version does not produce is ignored.
- A CV rule buzzes when it becomes true. From V1 on the Pi starts fresh at every session, so it
  buzzes again at every daily session while it stays true.
- The thresholds still have to be matched to our plant, with a source, for the report.

The file is read when the service starts: from V1 on that is every session, in V0 restart the
service after editing it. A line with a typo is skipped, not fatal: look for `line ignored` in
`phases_log.txt` after editing.

## Photo analysis (from the class labs)

`vision.py` uses the models and `helpers.py` directly from `../leafs-starter` and `../leafs-advanced`,
so those two folders must sit next to `pi/` on the Pi as well.

| Pipeline | Used by | Time per photo on the Pi | Values it gives |
|---|---|---|---|
| `"starter"`: U2-Net finds the leaves, HSV colour rules measure them | V0, V1 | about 2.2 s (+0.2-0.4 s to load) | `leaf_count`, `green_percent`, `yellow_percent`, `brown_percent` |
| `"advanced"`: MobileNetV3 classifier | V2, V3 | about 0.08 s (+0.1-0.5 s to load) | `unhealthy_percent` |

The starter lab looks simpler, but it is the expensive one: before any pixel is counted, U2-Net
has to find the leaf, and that network is far more work than the classifier.

- `leaf_count`: number of leaf-like regions. Touching leaves count as one.
- `green_percent`, `yellow_percent`, `brown_percent`: share of the leaf pixels with that colour.
  Yellow and brown overlap, so they can add up to more than 100. Missing when no region was found.
- `unhealthy_percent`: score for "diseased". Only meaningful for close-ups of pepper, potato and
  tomato leaves; on anything else it can be confidently wrong.

## Setup (once), the same way as in the leaf labs

Copy `pi/`, `leafs-starter/` and `leafs-advanced/` into `/home/admin/PEON` on the Pi, then in `pi/`:

    sudo raspi-config        # Interface Options > Serial Port: login shell No, hardware Yes
    python3 -m venv .venv
    source .venv/bin/activate
    python -m pip install --only-binary=:all: -r requirements.txt
    python test_common.py    # must print "ok" and nothing about SKIPPED

From V1 on the service powers the Pi off with `sudo -n poweroff`. Started at boot it has no
terminal to type a password into, so allow exactly that one command without a password:

    echo 'admin ALL=(root) NOPASSWD: /usr/sbin/poweroff' > /tmp/plant-poweroff
    sudo visudo -cf /tmp/plant-poweroff && sudo install -m 440 /tmp/plant-poweroff /etc/sudoers.d/plant-poweroff

(`visudo -cf` checks the line first: a broken file in `/etc/sudoers.d` would break `sudo` itself.)

Also from Vid's instructions: `sudo rpi-eeprom-config` must show `WAKE_ON_GPIO=1`, and
`/boot/firmware/config.txt` must not contain `dtoverlay=gpio-shutdown` (the ESP's pulse would
power the Pi off instead of waking it).

## Running the service

By hand, for testing with the ESP (venv active):

    python service.py 0      # V0: runs until Ctrl+C
    python service.py 2      # one session, then the Pi powers off

At every boot, for V1 to V3 (the number after `@` is the version):

    sudo cp plant@.service /etc/systemd/system/
    sudo systemctl enable plant@2
    sudo poweroff

From then on the ESP wakes the Pi, the service handles one session and powers it off. If the ESP
sends nothing, the Pi stays on and keeps sending `ready`, so you can still log in. Turn it off
with `sudo systemctl disable plant@2` (same number). Only one version may be enabled at a time:
they all use the serial port.

## Measuring the Pi with the Monsoon (Vid's "Navodila za meritve porabe Raspberry Pi")

Nine recordings, one CSV each. The commands write the start and end of every step to
`data/phases_log.txt`, which goes to Vid with the CSVs. There is no ESP: recording 8 runs one V2
session through the real service code with a stand-in for the ESP (`stanje.py seja`), and the Pi
then switches itself off, as in V2.

**At home, before the lab day**

1. Copy `pi/` to the Pi and do the setup above, including the sudoers line. `plant@2` enabled.
2. Try the two new commands: `python stanje.py cv_v0`, then `python stanje.py seja`. The second
   must end with the Pi switching itself off without asking for a password.
3. Ask Vid for a screenshot of his PowerTool export settings: the CSV needs Main current too.

**At the lab: setup**

- The Pi: only the camera plugged in (no ESP, HDMI, keyboard), power only from the Monsoon (USB-C).
- PowerTool: channel Main, USB passthrough Off, 5 kHz. Vout 5.10 V set, not enabled yet.
- Phone hotspot on, laptop on it. Keep the phone unlocked on the hotspot screen until the Pi shows
  up there. A stopwatch at hand.
- Never press Vout Disable while the Pi is on (green LED blinking), only once it is off.
- Export into a new folder: the file names are the same as last time.

**Every recording, the same way**

1. Type the command on the Pi, but do not press Enter yet.
2. Press RUN in PowerTool, then Enter.
3. Press STOP when the table says.
4. Export CSV: all samples, Main current and Main power, with the name from the table. Save the
   PowerTool recording too, under the same name. After the first export, check that the CSV's
   first line has a "Current" column.

| # | File | Command on the Pi | STOP when |
|---|---|---|---|
| 1 | `P1_boot.csv` | none: RUN, then Vout Enable, start the stopwatch | the stopwatch shows 1:30 |
| 2 | `P2_idle.csv` | none: RUN, touch nothing | 60 s later |
| 3 | `P3_photo_full.csv` | `python stanje.py slika` | the prompt is back (about 50 s) |
| 4 | `P3_photo_small.csv` | `python stanje.py slika_mala` | the prompt is back |
| 5 | `P4_cv_starter.csv` | `python stanje.py cv` | the prompt is back (about 1 min) |
| 6 | `P4_cv_starter_v0.csv` | `python stanje.py cv_v0` | the prompt is back |
| 7 | `P4_cv_advanced.csv` | `python stanje.py cv_mali` | the prompt is back |
| 8 | `P5_P6_session_shutdown.csv` | `python stanje.py seja` | the Pi switched itself off: green LED dark for 10 s |
| 9 | `P7_off.csv` | none: RUN, leave Vout on | 60 s later |

Between 1 and 2, log in and put the boot time in the log, then wait until the stopwatch shows at
least 2:00 (Vid: idle only 2 min after boot):

    ssh admin@peon-vr.local
    cd ~/PEON/pi && source .venv/bin/activate
    systemd-analyze | tee -a data/phases_log.txt

Between 7 and 8: `vcgencmd get_throttled | tee -a data/phases_log.txt` must print
`throttled=0x0`. If not, the voltage dropped too low: `sudo poweroff`, Vout to 5.20 V, and record
1-7 again (Vid's rule).

- In 3-7 the first small peak is the program starting (in 5-7 also the photo it analyses); the
  three measured steps follow, 10 s apart.
- A recording went wrong (you touched something, SSH dropped): STOP and record it again, same name.
- The Pi does not answer `ssh` 2 minutes after Vout Enable: check that the phone lists it as
  connected, then try `ssh admin@172.20.10.2` (or `.3`, `.4`).

**Afterwards**: Vout Disable, Vout Enable (the Pi is off, so this is safe; it boots again). A
minute later, on the laptop: `scp "admin@peon-vr.local:PEON/pi/data/phases_log.txt" .` Send Vid
the 9 CSVs and `phases_log.txt`.

Right after a boot, trust the `up=` numbers, not the clock times: the Pi has no battery clock
and starts from the time it saved at shutdown until it gets the time from the network.

## Known limits

- **Clock**: right after a boot the Pi's clock is the time it saved at shutdown, until NTP corrects
  it over WiFi. If the ESP has no real time either (it was just powered on), the Pi waits up to
  15 s for NTP and passes the time on; the ESP keeps it through deep sleep, so later sessions
  have correct times and no wait. The times of that first session can still be off. Without WiFi
  the Pi never gets the real time: then every such session waits the 15 s in vain (about 15 s
  of Pi power each), and "day" is judged only from the `daily` session.
- **A batch resent in the next session** (all 4 tries of an ack lost, then a poweroff) is logged
  twice: the Pi remembers batches only until it powers off.
- **The models are not quantized.** The assignment mentions quantization as a best practice; the
  class models are used unchanged.
- **Colour rules depend on lighting**, as the lab handout explains. Without a plant in view the
  expensive analysis still finds "leaves" and the cheap one says "unhealthy".
