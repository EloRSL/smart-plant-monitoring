# Smart Plant Monitoring

PECD 2026 IoT mini project. An ESP32 measures the plant all the time, checks the buzzer rules and
sleeps between readings. It wakes a Raspberry Pi 4 with a camera only when needed: the Pi receives
the readings, photographs and analyses the plant, writes the log and powers itself off.

| Folder | What | Who |
|---|---|---|
| [`pi/`](pi/) | Raspberry Pi side: the service, the photo analysis, the logs, the Monsoon measurement states. See [pi/README.md](pi/README.md) | Rok |
| ESP32 firmware V0–V3 | to be added | Vid |

The Pi loads the models of the two class leaf labs (`leafs-starter`, `leafs-advanced`). They are
course material, so they are not in this repository; `pi/README.md` says where they go.
