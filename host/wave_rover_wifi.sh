#!/bin/bash
# Draait op de Jetson (buiten de container) als wave-rover-wifi.service.
# Wifi-terugval: is er geen bekend wifinetwerk (thuis TP-Link_9374 of De Clercq), dan maakt
# de Jetson zelf de hotspot "WaveRover". Het dashboard blijft bereikbaar op
# http://jetson.local:8080 (of http://10.42.0.1:8080).
# - Na het opstarten of na het wegvallen van de wifi: eerst WAIT s wachten op een bekend
#   netwerk (NetworkManager verbindt daar zelf mee), pas daarna de hotspot.
# - Hotspot aan en al RETRY s geen enkel toestel verbonden: hotspot uit, zodat
#   NetworkManager een bekend netwerk kan zoeken (bv. de router thuis was even weg).
#   Niets gevonden: na WAIT s opnieuw de hotspot.
# Kost bijna niets: om de CHECK s een iw-opvraging.
HOTSPOT=WaveRover-hotspot
WAIT=60
RETRY=300
CHECK=10

IFACE=$(nmcli -t -f DEVICE,TYPE device | awk -F: '$2 == "wifi" {print $1; exit}')
if [ -z "$IFACE" ]; then
    echo "Geen wifikaart gevonden"
    exit 1
fi

# "hotspot", "wifi" (verbonden met een netwerk) of leeg. Met iw i.p.v. nmcli: iw vraagt het
# rechtstreeks aan de wifikaart, nmcli start telkens een D-Bus-client (~40 ms CPU).
connected() {
    local info
    info=$(iw dev "$IFACE" info 2>/dev/null)
    case "$info" in
        *"type AP"*) echo hotspot ;;
        *ssid*) echo wifi ;;
    esac
}

clients() {
    iw dev "$IFACE" station dump 2>/dev/null | grep -c '^Station'
}

lost=$(date +%s)  # laatste keer verbonden (wifi of hotspot)
idle=$lost        # laatste keer een toestel op de hotspot
while true; do
    now=$(date +%s)
    conn=$(connected)
    if [ -n "$conn" ]; then
        # Ook bij de hotspot: gaat die uit (hieronder of met de hand), dan krijgt een
        # bekend netwerk eerst WAIT s.
        lost=$now
    fi
    if [ "$conn" = hotspot ]; then
        if [ "$(clients)" -gt 0 ]; then
            idle=$now
        elif [ $((now - idle)) -ge $RETRY ]; then
            echo "Al $RETRY s niemand op de hotspot: bekend wifinetwerk zoeken"
            nmcli connection down "$HOTSPOT" >/dev/null
        fi
    elif [ -z "$conn" ] && [ $((now - lost)) -ge $WAIT ]; then
        echo "Geen bekend wifinetwerk na $WAIT s: hotspot aan"
        if nmcli connection up "$HOTSPOT" >/dev/null; then
            idle=$now
        else
            lost=$now  # mislukt: na WAIT s opnieuw proberen
        fi
    fi
    sleep $CHECK
done
