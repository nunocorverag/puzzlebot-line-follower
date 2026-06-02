# First-Time Setup

Getting a fresh laptop ready to drive the Puzzlebot. There are **two
independent tracks** — do whichever applies to you, in any order:

- **A. Libraries** — what every laptop needs to run the tools and view streams.
- **B. Network** — only for the person who *hosts* the robot WiFi (RoboNet).

A teammate who only writes/runs code needs **A**. Whoever brings the WiFi dongle
and shares internet to the robot also does **B**. They do not depend on each
other.

```bash
git clone <repo-url> && cd puzzlebot-line-follower
```

---

## A. Libraries (every laptop)

```bash
scripts/setup_laptop.sh
```

Installs the GStreamer H264 receiver, `ssh`/`rsync`, and the Python deps
(`numpy`, `opencv-python`, `pygame`) from `requirements-laptop.txt`. It also
seeds `scripts/local.env`. No ROS2 is needed on the laptop — the run-scripts SSH
into the Jetson, which has its own ROS stack.

- `NO_APT=1 scripts/setup_laptop.sh` installs only the pip packages (no sudo).
- WSL/X11 users: `scripts/set_local_video_sink.sh ximagesink` so preview windows
  work.

You also want passwordless SSH to the robot (needed by every `run_*` script):

```bash
ssh-copy-id puzzlebot@10.10.0.100
```

---

## B. Network — RoboNet (only the WiFi host)

The robot joins a WiFi AP called **RoboNet** that a laptop hosts. That laptop is
always `10.10.0.1`; the Jetson connects as a client at `10.10.0.100`.

```bash
scripts/setup_robonet.sh
```

This recreates the access point via `nmcli` (SSID `RoboNet`, password
`puzzlebot123`, internet-sharing, `10.10.0.1/24`). Defaults match what the robot
expects — **change the SSID/password only if you also reconfigure the Jetson**,
or it will not connect.

Notes:

- Needs a WiFi adapter that supports AP mode. The script prefers a USB dongle
  (`wlx*`); override with `WIFI_IFACE=<dev>`.
- Internet sharing requires a **second** interface (internal WiFi or ethernet)
  for the upstream connection; the AP runs on a separate adapter.
- Override defaults inline: `ROBONET_PSK=... ROBONET_SSID=... scripts/setup_robonet.sh`.

---

## Verify

```bash
scripts/check_setup.sh
```

Read-only checklist. It reports the **LIBRARIES** and **NETWORK** blocks
separately, with the exact fix command for anything missing. All green = ready
to run (`scripts/run_line_follower_jetson.sh`, `scripts/run_teleop_wasd_combo.sh`,
…). See [SCRIPTS.md](SCRIPTS.md) for the full catalog.
