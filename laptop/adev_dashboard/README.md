# ADEV Dashboard — Setup Guide

**Developed by ADEV** · WorldSkills India · Navigation & Localization

This guide tells you exactly what to do: copy files to your laptop, then run Pi + laptop together at competition.

---

## What runs where

| Machine | What runs | Port |
|---------|-----------|------|
| **Raspberry Pi** | Robot hardware, Nav2, lightweight API bridge | `8080` (API only — no web page) |
| **Your laptop** | Dashboard UI (map, waypoints, E-stop, mission) | `3000` |

You need **both** running and connected via **Pi hotspot** (or same WiFi).

---

## Step 1 — Copy dashboard to your laptop (one time, or after updates)

On your **laptop**, connected to the Pi hotspot:

```bash
mkdir -p ~/Music/adev_dashboard
scp -r vmx@192.168.4.1:~/skill23_ws/laptop/adev_dashboard/* ~/Music/adev_dashboard/
chmod +x ~/Music/adev_dashboard/start.sh
```

Replace `192.168.4.1` with your Pi’s IP if different (`hostname -I` on the Pi).

**Or** use the script included in the copy:

```bash
cd ~/Music
bash adev_dashboard/pull_dashboard_from_pi.sh 192.168.4.1
```

You should have:

```
~/Music/adev_dashboard/
  index.html
  css/
  js/
  start.sh
  README.md
```

---

## Step 2 — Build on the Pi (one time, or after code changes)

SSH into the Pi (or use its terminal):

```bash
cd ~/skill23_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select adev_dashboard studica_control xdrive allterrain mecanum
source install/setup.bash
```

Add to `~/.bashrc` if not already there:

```bash
source /opt/ros/humble/setup.bash
source ~/skill23_ws/install/setup.bash
alias ros2_pi='~/skill23_ws/src/studica_ws/scripts/ros2_pi.sh'
```

Uses the default ROS 2 middleware (Fast-DDS). You do **not** need CycloneDDS or `RMW_IMPLEMENTATION`.

Hardware permissions (once):

```bash
cd ~/skill23_ws/src/studica_ws
./scripts/setup_permissions.sh ~/skill23_ws
```

---

## Step 3 — Every time you compete / practice

### A. On the Raspberry Pi

1. Power on robot and Pi.
2. Turn on Pi hotspot (or connect Pi and laptop to same network).
3. Open a terminal on the Pi:

```bash
source ~/.bashrc

# Pick YOUR robot (only one) — use ros2_pi.sh (sudo + env) so battery/VMX topics work:

# All-terrain
~/skill23_ws/src/studica_ws/scripts/ros2_pi.sh launch adev_dashboard adev_dashboard.launch.py robot:=allterrain map:=~/skill23_ws/test_map10.yaml

# Or shortcut:
~/skill23_ws/src/adev_dashboard/scripts/launch_adev_dashboard.sh allterrain

# Mecanum
~/skill23_ws/src/studica_ws/scripts/ros2_pi.sh launch adev_dashboard adev_dashboard.launch.py robot:=mecanum map:=~/skill23_ws/test_map6.yaml

# X-Drive
~/skill23_ws/src/studica_ws/scripts/ros2_pi.sh launch adev_dashboard adev_dashboard.launch.py robot:=xdrive map:=~/skill23_ws/test_map10.yaml
```

**Navigation only** (no dashboard bridge) — same prefix you already use:

```bash
~/skill23_ws/src/studica_ws/scripts/ros2_pi.sh launch studica_control navigation_allterrain.py
```

For debugging topics: always prefix with `ros2_pi.sh` (e.g. `ros2_pi.sh topic echo /battery_state --once`).

4. Wait **15–20 seconds** for Nav2 and sensors to start.
5. You should see a log line like: `ADEV API bridge on :8080 (API only — UI runs on laptop)`.

**Do not** open `http://pi-ip:8080` in a browser expecting the dashboard — that URL is API only.

---

### B. On your laptop

1. Connect WiFi to the **Pi hotspot**.
2. Open a terminal:

```bash
cd ~/Music/adev_dashboard
./start.sh
```

3. Open a browser:

```
http://localhost:3000
```

4. In the dashboard left panel:
   - **Pi Bridge URL:** `http://192.168.4.1:8080` (use your Pi IP)
   - Click **Connect**
   - Status should show **CONNECTED** (green dot)

5. Select the **same robot type** you launched on the Pi (allterrain / mecanum / xdrive).

6. Click **Load Map from Pi** (or pick map from dropdown).

---

## Step 4 — Run a mission

1. On the map toolbar, choose **+ Waypoint**.
2. **Click** on the map for each goal position.
3. Set **yaw** (degrees) for the last point if needed → click **Set**.
4. Check the waypoint list on the right.
5. Click **▶ Mission Start**.

The robot will go to each waypoint in order. If it fails, it **retries** until that point is reached, then continues.

- **■ Stop Mission** — cancel navigation  
- **EMERGENCY STOP** — stop robot immediately (release when safe)

---

## Step 5 — Other useful tools

| Tool | How |
|------|-----|
| **Measure distance** | Map toolbar → **Measure** → click two points |
| **Set robot pose** | Map toolbar → **Set Pose** → click on map (AMCL) |
| **Simulation** | Toggle **Simulation Mode** on laptop (no Pi needed for UI test) |
| **E-stop status** | Right panel shows hardware E-stop if wired |

---

## Quick checklist

```
[ ] Copied ~/Music/adev_dashboard from Pi to laptop
[ ] Built skill23_ws on Pi (colcon build)
[ ] Pi: ~/skill23_ws/src/studica_ws/scripts/ros2_pi.sh launch adev_dashboard ... robot:=YOUR_ROBOT
[ ] Laptop: ./start.sh → http://localhost:3000
[ ] Dashboard: Connect to http://<pi-ip>:8080
[ ] Map loaded, waypoints added, mission started
```

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| Battery **N/A** / topics missing | Launch and debug with `ros2_pi.sh` (sudo + env prefix) |
| Laptop shows **OFFLINE** | Check Pi IP, hotspot, firewall; Pi bridge must be running |
| **Nav2 not ready** | Wait 20 s after launch; check Pi terminal for errors |
| Mission does not start | Add waypoints first; ensure Nav2 is up |
| Wrong robot behaviour | Same `robot:=` on Pi launch as in dashboard dropdown |
| Map won’t load | Confirm `map:=` path exists on Pi (`~/skill23_ws/test_map10.yaml` etc.) |
| After code update on Pi | Re-run `scp` to laptop + `colcon build` on Pi |

---

## Maps on the Pi

Default maps in `~/skill23_ws/`:

| Robot | Suggested map |
|-------|----------------|
| allterrain | `test_map10.yaml` |
| mecanum | `test_map6.yaml` |
| xdrive | `test_map10.yaml` |

---

## File locations summary

| What | Path |
|------|------|
| Dashboard on laptop (you run this) | `~/Music/adev_dashboard/` |
| Dashboard source on Pi (for copying) | `~/skill23_ws/laptop/adev_dashboard/` |
| Pi bridge + launch files | `~/skill23_ws/src/adev_dashboard/` |
| Nav2 / robot launches | `~/skill23_ws/src/studica_ws/.../launch/` |

---

**Representing India · WorldSkills · Developed by ADEV**
