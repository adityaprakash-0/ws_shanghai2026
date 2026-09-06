#!/usr/bin/env bash
# Replace two 4GB swap files with one 16GB swap file.
# Safe to run over SSH. Requires sudo.
#
# Usage: sudo ./tools/reconfigure_swap.sh

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run with sudo: sudo $0"
  exit 1
fi

NEW_SWAP="/swapfile"
NEW_SIZE_GB=16
OLD_SWAPS=("/swapfile" "/swapfile2")

log() { echo "[reconfigure_swap] $*"; }

log "Current swap:"
swapon --show || true
free -h

log "Disabling old swap"
for f in "${OLD_SWAPS[@]}"; do
  swapoff "$f" 2>/dev/null || true
done

log "Removing old swap entries from /etc/fstab"
cp -a /etc/fstab "/etc/fstab.bak.$(date +%Y%m%d%H%M%S)"
sed -i '\|^/swapfile[[:space:]]|d' /etc/fstab
sed -i '\|^/swapfile2[[:space:]]|d' /etc/fstab

log "Removing old swap files"
rm -f /swapfile /swapfile2

log "Creating ${NEW_SIZE_GB}GB swap file at ${NEW_SWAP}"
fallocate -l "${NEW_SIZE_GB}G" "${NEW_SWAP}"
chmod 600 "${NEW_SWAP}"
mkswap "${NEW_SWAP}"

log "Enabling new swap"
if ! grep -q "^${NEW_SWAP}[[:space:]]" /etc/fstab; then
  echo "${NEW_SWAP}      none   swap    sw  0    0" >> /etc/fstab
fi
swapon "${NEW_SWAP}"

log "Done. New swap layout:"
swapon --show
free -h
