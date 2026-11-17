#!/usr/bin/env bash
# Install g1_link_watch on the G1's Jetson as the systemd service g1-link-watch; docs/estop.md.
#
#   rsync -a robot_stacks/g1/g1_link_watch/ unitree@192.168.123.164:~/g1_link_watch/
#   ssh -t unitree@192.168.123.164 'sudo ~/g1_link_watch/install.sh'
#
# --dry-run: print the steps, change nothing. Safe to re-run: it copies the code and restarts.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST=/opt/g1_link_watch  # the unit's PYTHONPATH
UNIT=g1-link-watch.service

DRY_RUN=0
case "${1:-}" in
    --dry-run) DRY_RUN=1 ;;
    "") ;;
    *) echo "usage: $0 [--dry-run]" >&2; exit 2 ;;
esac
run() { if [ "$DRY_RUN" = 1 ]; then printf '%q ' "$@"; printf '\n'; else "$@"; fi; }

run install -d "$DEST"
run rsync -a --delete --exclude __pycache__ "$HERE/g1_link_watch/" "$DEST/g1_link_watch/"
run install -m 644 "$HERE/$UNIT" "/etc/systemd/system/$UNIT"
run systemctl daemon-reload
run systemctl enable "$UNIT"
run systemctl restart "$UNIT"
