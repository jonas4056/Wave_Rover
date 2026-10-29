#!/bin/bash
# Toont de laatste opstarts van de Jetson en hoe elke sessie eindigde.
# Vanaf de pc:  scripts/boot_history.sh [aantal]
#
#   netjes    = normaal afgesloten of herstart
#   ABRUPT    = geen afsluitmeldingen: bevriezing, crash, watchdog of stroomverlies
#   (actief)  = huidige sessie
set -e

COUNT="${1:-10}"

ssh jetson "COUNT=$COUNT bash -s" <<'EOF'
journalctl --list-boots --no-pager | tail -n "$COUNT" | while read -r idx id rest; do
    [[ "$idx" =~ ^-?[0-9]+$ ]] || continue
    last=$(sudo journalctl -b "$id" --no-pager -o short-iso -n 1 | tail -n 1 | cut -c1-19)
    if [ "$idx" = "0" ]; then
        result="(actief)"
    # Bij een normale afsluiting stopt systemd op het einde tientallen services; bij een
    # bevriezing of stroomverlies geen enkele. Alleen het einde bekijken: tijdens de sessie
    # stopt systemd ook af en toe iets (ssh-sessies, herstarte diensten).
    elif [ "$(sudo journalctl -b "$id" --no-pager -q -n 300 | grep -cE 'systemd\[1\]: Stopping ')" -gt 10 ]; then
        result="netjes"
    else
        result="ABRUPT"
    fi
    printf '%4s  einde %s  %s\n' "$idx" "$last" "$result"
done
EOF
