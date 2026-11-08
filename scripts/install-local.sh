#!/usr/bin/env bash
# Builds Icarus and installs it for this account: `icarus` (command line) and `icarus-app` (the window) in ~/.local/bin, and
# Icarus in the app menu. Icarus is private, so it is installed from this checkout, not from a package repository.
#   bash scripts/install-local.sh            build and install
#   bash scripts/install-local.sh --remove   take it out again
set -euo pipefail
cd "$(dirname "$0")/.."
bin=$HOME/.local/bin
apps=${XDG_DATA_HOME:-$HOME/.local/share}/applications
if [[ ${1:-} == --remove ]]; then
  rm -f "$bin/icarus" "$bin/icarus-app" "$apps/icarus.desktop"
  echo "Icarus removed from $bin and the app menu."
  exit 0
fi
cargo build --release
install -Dm755 target/release/icarus "$bin/icarus"
install -Dm755 target/release/icarus-app "$bin/icarus-app"
install -Dm644 packaging/icarus.desktop "$apps/icarus.desktop"
echo "Installed: icarus and icarus-app in $bin; Icarus is in the app menu."
