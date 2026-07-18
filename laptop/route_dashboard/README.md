# Route Graph Builder (laptop only)

**Build / edit graphs here.** Run **missions** with **ADEV Dashboard**.

## Copy to laptop

```bash
scp -r vmx@<pi-ip>:~/skill23_ws/laptop/route_dashboard/* ~/Music/laptop/route_dashboard/
chmod +x ~/Music/laptop/route_dashboard/start.sh
```

## Build a new graph

```bash
cd ~/Music/laptop/route_dashboard
./start.sh
```

1. Open http://localhost:3001  
2. Select **map .yaml** and **map .pgm**  
3. Click **Load Local Map**  
4. **+ Node** on corridors → **Autoconnect** → **Export GeoJSON** or **Save Graph**  
5. `scp *_graph.geojson vmx@<pi-ip>:~/skill23_ws/`

## Load & edit an existing GeoJSON

1. Load **yaml + pgm** (same map the graph was built for)  
2. Select **Graph GeoJSON** → **Load Graph**  
   - Nodes, edges, and node numbers appear on the map and in the Nodes list  
3. Add / edit with **+ Node**, Autoconnect, Undo, etc.  
4. **Save Graph (overwrite)**  
   - Chrome/Edge: can overwrite the file in place (file picker)  
   - Otherwise downloads with the **same filename** (choose overwrite in the dialog)

**Export GeoJSON** still works as a separate “download a copy” action.

## Run robot + missions (Pi)

```bash
~/skill23_ws/src/adev_dashboard/scripts/launch_adev_dashboard.sh nav2_secret_mode mecanum \
  ~/skill23_ws/test_map11.yaml \
  ~/skill23_ws/test_map11_graph.geojson
```

Laptop ADEV dashboard → connect to `http://<pi-ip>:8080`.
