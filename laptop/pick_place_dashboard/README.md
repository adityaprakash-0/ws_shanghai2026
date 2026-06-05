# Pick & Place Dashboard (laptop)

Lightweight UI for the **skill23_pick_place** stack on the Pi.

- Talks to Pi bridge on **port 8081** (not ADEV 8080)
- Uses separate `localStorage` keys (`pp_dashboard_*`)
- Map stickers → mission steps → align / pick / place

## Run

```bash
chmod +x start.sh
./start.sh        # http://localhost:3001
```

Set **Pi Bridge URL** to `http://<pi-hotspot-ip>:8081`.

## Pi launch

```bash
ros2_pi.sh launch skill23_pick_place pick_place.launch.py robot:=mecanum
```

See `src/skill23_pick_place/README.md` for calibration.
