#!/usr/bin/env bash
# Install Orbbec depth camera ROS2 packages into workspace local_sysroot (not apt).
#
# Usage:
#   ./scripts/install_depth_camera.sh
#   DEPTH_CAMERA_DIR=/path/to/debs ./scripts/install_depth_camera.sh
#
# Requires: sudo (udev rules + ldconfig). Does NOT use apt install.

set -eo pipefail

WS="${SKILL23_WS:-${HOME}/skill23_ws}"
DEPTH_CAMERA_DIR="${DEPTH_CAMERA_DIR:-${HOME}/depth_camera}"
LOCAL_SYSROOT="${WS}/local_sysroot"
ORBBEC_PREFIX="${LOCAL_SYSROOT}/opt/ros/humble"

MSGS_DEB="${DEPTH_CAMERA_DIR}/ros-humble-orbbec-camera-msgs_jammy_arm64.deb"
CAMERA_DEB="${DEPTH_CAMERA_DIR}/ros-humble-orbbec-camera_jammy_arm64.deb"

for deb in "$MSGS_DEB" "$CAMERA_DEB"; do
  if [ ! -f "$deb" ]; then
    echo "ERROR: missing package: $deb"
    echo "       Put Orbbec .deb files in ${DEPTH_CAMERA_DIR}/"
    exit 1
  fi
done

echo "==> Extracting Orbbec packages into ${LOCAL_SYSROOT}"
mkdir -p "${LOCAL_SYSROOT}"
dpkg-deb -x "$MSGS_DEB" "${LOCAL_SYSROOT}"
dpkg-deb -x "$CAMERA_DEB" "${LOCAL_SYSROOT}"

# Keep a copy of udev rules in the workspace (system install below).
mkdir -p "${LOCAL_SYSROOT}/etc/udev/rules.d"
if [ -f "${LOCAL_SYSROOT}/etc/udev/rules.d/99-obsensor-libusb.rules" ]; then
  echo "OK  udev rules extracted to local_sysroot"
fi

echo "==> Fixing /dev/shm permissions (Orbbec SDK needs writable shared memory)"
# Some Pi images ship /dev/shm as 0755 — Orbbec cannot create orbbec_device_lock there.
if [ "$(stat -c '%a' /dev/shm)" != "1777" ]; then
  sudo chmod 1777 /dev/shm
  echo "OK  /dev/shm set to 1777"
else
  echo "OK  /dev/shm already 1777"
fi

echo "==> Installing USB udev rules (requires sudo)"
sudo cp "${LOCAL_SYSROOT}/etc/udev/rules.d/99-obsensor-libusb.rules" \
  /etc/udev/rules.d/99-obsensor-libusb.rules
sudo chmod 644 /etc/udev/rules.d/99-obsensor-libusb.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "OK  udev rules installed"

echo "==> Adding user to video group (camera USB access)"
if id -nG "$USER" | tr ' ' '\n' | grep -qx video; then
  echo "OK  $USER already in group video"
else
  sudo usermod -aG video "$USER"
  echo "OK  added $USER to video — log out/in or run: newgrp video"
fi

echo "==> Registering local_sysroot libraries with ldconfig"
LDCONF_FILE="/etc/ld.so.conf.d/skill23_local_sysroot.conf"
sudo tee "$LDCONF_FILE" > /dev/null <<EOF
# skill23_ws local_sysroot (studica + orbbec)
${LOCAL_SYSROOT}/lib
${ORBBEC_PREFIX}/lib
EOF
sudo ldconfig
echo "OK  ldconfig updated"

HOOK_MARKER="# skill23 local_sysroot env"
SETUP_BASH="${WS}/install/setup.bash"
if [ -f "$SETUP_BASH" ] && ! grep -qF "$HOOK_MARKER" "$SETUP_BASH"; then
  cat >> "$SETUP_BASH" <<EOF

$HOOK_MARKER
if [ -f "\${SKILL23_WS:-\$HOME/skill23_ws}/local_sysroot/env.bash" ]; then
  . "\${SKILL23_WS:-\$HOME/skill23_ws}/local_sysroot/env.bash"
fi
EOF
  echo "OK  hooked local_sysroot into install/setup.bash"
elif [ -f "$SETUP_BASH" ]; then
  echo "OK  install/setup.bash hook already present"
else
  echo "WARN install/setup.bash not found — source local_sysroot/env.bash manually"
fi

echo ""
echo "Install complete."
echo ""
echo "Verify (plug in Gemini E camera first):"
echo "  source ${WS}/install/setup.bash"
echo "  ros2 pkg prefix orbbec_camera    # should show .../local_sysroot/opt/ros/humble"
echo "  lsusb | grep -i 2bc5             # Orbbec USB"
echo "  ros2 launch orbbec_camera gemini_e.launch.py"
echo ""
echo "Optional — remove global apt packages so only local_sysroot is used:"
echo "  sudo apt remove ros-humble-orbbec-camera ros-humble-orbbec-camera-msgs"
