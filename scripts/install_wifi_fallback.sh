#!/bin/bash
# Installeert op de Jetson de wifi-terugval (vanaf de pc uitvoeren):
#   geen bekend wifinetwerk -> hotspot "WaveRover" (zie host/wave_rover_wifi.sh).
#   scripts/install_wifi_fallback.sh [wachtwoord]
# Plus een vaste interface ros0 (10.99.99.1) voor ROS: elke ROS-node geeft bij het starten
# zijn adressen door. Was dat enkel het wifi-adres, dan vonden nodes die na de omschakeling
# naar de hotspot starten (bv. een module vanuit de app) de rest niet meer (getest
# 28-10-2026). ros0 verandert nooit, dus daarover vinden ze elkaar altijd.
# Zonder wachtwoord wordt er een gemaakt (alleen de eerste keer). Het staat niet in git;
# opvragen op de Jetson met:
#   sudo nmcli -s -g 802-11-wireless-security.psk connection show WaveRover-hotspot
set -e

ssh jetson "bash -s -- '$1'" <<'REMOTE'
set -e
chmod +x ~/wave_rover/host/wave_rover_wifi.sh
IFACE=$(nmcli -t -f DEVICE,TYPE device | awk -F: '$2 == "wifi" {print $1; exit}')

if ! nmcli -t -f NAME connection show | grep -qx WaveRover-hotspot; then
    PASSWORD=${1:-$(tr -dc 'abcdefghjkmnpqrstuvwxyz23456789' </dev/urandom | head -c 10)}
    # 2,4 GHz: verder bereik en alle kanalen 1-11 toegelaten (5 GHz enkel kanaal 44).
    # WPA2 met AES (iOS waarschuwt bij TKIP). "shared": de Jetson is 10.42.0.1 en geeft
    # zelf adressen uit (dnsmasq van NetworkManager). Alleen wave_rover_wifi.sh zet hem aan.
    sudo nmcli connection add type wifi ifname "$IFACE" con-name WaveRover-hotspot \
        ssid WaveRover autoconnect no \
        802-11-wireless.mode ap 802-11-wireless.band bg \
        wifi-sec.key-mgmt wpa-psk wifi-sec.proto rsn wifi-sec.pairwise ccmp \
        wifi-sec.group ccmp wifi-sec.psk "$PASSWORD" \
        ipv4.method shared ipv6.method disabled >/dev/null
    echo "Hotspot WaveRover aangemaakt, wachtwoord: $PASSWORD"
elif [ -n "$1" ]; then
    sudo nmcli connection modify WaveRover-hotspot wifi-sec.psk "$1"
    echo "Wachtwoord van de hotspot aangepast"
fi

# ros0: vóór Docker, zodat de ROS-nodes in de container hem van bij het begin kennen.
sudo tee /etc/NetworkManager/conf.d/99-ros0-unmanaged.conf >/dev/null <<'CONF'
[keyfile]
unmanaged-devices=interface-name:ros0
CONF
sudo tee /etc/systemd/system/wave-rover-ros0.service >/dev/null <<'UNIT'
[Unit]
Description=Wave Rover: vaste interface ros0 (10.99.99.1) voor ROS, los van de wifi
Before=docker.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/sh -c 'ip link show ros0 >/dev/null 2>&1 || ip link add ros0 type dummy; ip addr replace 10.99.99.1/32 dev ros0; ip link set ros0 multicast on up'

[Install]
WantedBy=multi-user.target
UNIT

sudo tee /etc/systemd/system/wave-rover-wifi.service >/dev/null <<'UNIT'
[Unit]
Description=Wave Rover: hotspot WaveRover als er geen bekend wifinetwerk is
After=NetworkManager.service
Wants=NetworkManager.service

[Service]
ExecStart=/home/jonas/wave_rover/host/wave_rover_wifi.sh
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
UNIT

sudo systemctl daemon-reload
sudo systemctl reload NetworkManager
sudo systemctl enable --now wave-rover-ros0.service wave-rover-wifi.service
sudo systemctl restart wave-rover-wifi.service
systemctl is-active wave-rover-ros0.service wave-rover-wifi.service
ip -br addr show ros0
REMOTE
echo "Herstart de container (docker compose restart ros) als ros0 nieuw is, zodat ROS hem kent."
