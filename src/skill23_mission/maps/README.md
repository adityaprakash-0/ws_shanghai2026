# Put one folder per arena here, then rebuild (or use absolute map:=).
#
#   maps/
#     default/
#       map.yaml
#       map.pgm
#       waypoints.json     ← from Studio (Save to Pi / copy)
#       sequences.json     ← optional, from Studio "Save sequences to Pi"
#
# Launch:
#   ros2 launch skill23_mission bringup.launch.py robot:=mecanum map:=default
#
# Rename "default" to whatever you like (e.g. maps/arena1/) and pass map:=arena1
