#!/usr/bin/env bash
# Builds Icarus and installs it for this account: `icarus` (command line) and `icarus-app` (the window) in ~/.local/bin, and
# Icarus in the app menu with its icon (hicolor). Icarus is private, so it is installed from this checkout, not from a
# package repository.
#   bash scripts/install-local.sh            build and install
#   bash scripts/install-local.sh --remove   take it out again
#   bash scripts/install-local.sh --udev     also let the person at the computer open tuners with no Linux driver (needs sudo once:
#                                            packaging/70-icarus-tuners.rules into /etc/udev/rules.d)
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ ${1:-} == --udev ]]; then
  sudo install -Dm644 packaging/70-icarus-tuners.rules /etc/udev/rules.d/70-icarus-tuners.rules
  sudo udevadm control --reload
  sudo udevadm trigger --subsystem-match=usb --action=change
  echo "Tuner access rules installed (/etc/udev/rules.d/70-icarus-tuners.rules)."
  shift
fi
bin=$HOME/.local/bin
apps=${XDG_DATA_HOME:-$HOME/.local/share}/applications
icons=${XDG_DATA_HOME:-$HOME/.local/share}/icons/hicolor
sizes=(16 24 32 48 64 128 256 512)
if [[ ${1:-} == --remove ]]; then
  rm -f "$bin/icarus" "$bin/icarus-app" "$apps/icarus.desktop" "$icons/scalable/apps/icarus.svg"
  for n in "${sizes[@]}"; do rm -f "$icons/${n}x${n}/apps/icarus.png"; done
  echo "Icarus removed from $bin and the app menu."
  exit 0
fi
cargo build --release
install -Dm755 target/release/icarus "$bin/icarus"
install -Dm755 target/release/icarus-app "$bin/icarus-app"
install -Dm644 packaging/icarus.desktop "$apps/icarus.desktop"
install -Dm644 packaging/icons/icarus.svg "$icons/scalable/apps/icarus.svg"
for n in "${sizes[@]}"; do install -Dm644 "packaging/icons/${n}x${n}/icarus.png" "$icons/${n}x${n}/apps/icarus.png"; done
echo "Installed: icarus and icarus-app in $bin; Icarus is in the app menu."
