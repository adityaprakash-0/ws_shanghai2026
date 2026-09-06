#!/usr/bin/env bash
# Reconfigure Pi: hostname vmx1, client WiFi (Sakar Robotics), disable hotspot AP.
# Safe to run over SSH while eth0 is connected — wlan0 changes do not affect eth0.
#
# Usage:
#   sudo ./tools/configure_vmx1.sh '<new-login-password>' '<sakar-wifi-password>'
#
# After reboot (or when Sakar Robotics is in range):
#   ssh vmx1@vmx1.local

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run with sudo: sudo $0 '<new-login-password>' '<sakar-wifi-password>'"
  exit 1
fi

if [[ $# -lt 2 ]]; then
  echo "Usage: sudo $0 '<new-login-password>' '<sakar-wifi-password>'"
  exit 1
fi

NEW_PASS="$1"
SAKAR_PASS="$2"
OLD_USER="vmx"
NEW_USER="vmx1"
NEW_HOST="vmx1"
SAKAR_SSID="Sakar Robotics"

log() { echo "[configure_vmx1] $*"; }

log "Step 1/7: Disable hotspot autoconnect (wlan0 stays up until step 6)"
nmcli connection modify Hotspot connection.autoconnect no connection.autoconnect-priority -999 2>/dev/null || true

log "Step 2/7: Add/update Sakar Robotics WiFi (high autoconnect priority)"
if nmcli -t -f NAME connection show | grep -qxF "${SAKAR_SSID}"; then
  nmcli connection modify "${SAKAR_SSID}" \
    wifi.ssid "${SAKAR_SSID}" \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "${SAKAR_PASS}" \
    connection.autoconnect yes \
    connection.autoconnect-priority 200
else
  nmcli connection add type wifi con-name "${SAKAR_SSID}" ifname wlan0 \
    ssid "${SAKAR_SSID}" \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "${SAKAR_PASS}"
  nmcli connection modify "${SAKAR_SSID}" \
    connection.autoconnect yes \
    connection.autoconnect-priority 200
fi

log "Step 3/7: Set hostname to ${NEW_HOST}"
hostnamectl set-hostname "${NEW_HOST}"
if grep -q '^127.0.1.1' /etc/hosts; then
  sed -i "s/^127.0.1.1.*/127.0.1.1\t${NEW_HOST}/" /etc/hosts
else
  echo -e "127.0.1.1\t${NEW_HOST}" >> /etc/hosts
fi

log "Step 4/7: Rename user ${OLD_USER} -> ${NEW_USER} (current SSH session stays alive via UID)"
if id "${OLD_USER}" &>/dev/null && ! id "${NEW_USER}" &>/dev/null; then
  usermod -l "${NEW_USER}" "${OLD_USER}"
  usermod -d "/home/${NEW_USER}" -m "${NEW_USER}"
  groupmod -n "${NEW_USER}" "${OLD_USER}" 2>/dev/null || true
  TARGET_USER="${NEW_USER}"
elif id "${NEW_USER}" &>/dev/null; then
  TARGET_USER="${NEW_USER}"
else
  TARGET_USER="${OLD_USER}"
  log "WARN: could not find ${OLD_USER}; skipping user rename"
fi

log "Step 5/7: Update sudoers entries for renamed user"
for f in /etc/sudoers.d/skill23_studio /etc/sudoers.d/skill23_studio_kill /etc/sudoers.d/studica_control; do
  if [[ -f "$f" ]]; then
    sed -i "s/^${OLD_USER} /${NEW_USER} /g" "$f"
    visudo -cf "$f" >/dev/null
  fi
done

log "Step 6/7: Set login password for ${TARGET_USER}"
echo "${TARGET_USER}:${NEW_PASS}" | chpasswd

log "Step 7/7: Enable SSH + mDNS, switch wlan0 from hotspot to client WiFi"
systemctl enable ssh avahi-daemon
systemctl restart avahi-daemon

# Bring down hotspot AP; connect to Sakar Robotics if in range (eth0 SSH unaffected)
nmcli connection down Hotspot 2>/dev/null || true
if nmcli connection up "${SAKAR_SSID}" 2>/dev/null; then
  log "Connected wlan0 to ${SAKAR_SSID}"
else
  log "Sakar Robotics not in range now — will autoconnect on next boot"
fi

log "Done."
log "  Hostname:  $(hostname)"
log "  User:      ${TARGET_USER}"
log "  mDNS:      ${NEW_HOST}.local (via avahi)"
log "  SSH:       ssh ${TARGET_USER}@${NEW_HOST}.local"
log "  Hotspot:   disabled (autoconnect off)"
log ""
log "Current eth0 SSH session is unchanged. Reboot when convenient for a clean boot test."
