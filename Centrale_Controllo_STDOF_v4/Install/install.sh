#!/bin/bash
# Script per installare il programma centrale di controllo

# Mi assicuro che esista la cartella /opt/spinitalia dove andrà l'eseguibile
if [ ! -d "/opt/spintalia" ]; then
	echo "creo la directory /opt/spinitalia"
	mkdir -p /opt/spinitalia
else
	echo "la cartella /opt/spinitalia esiste"
fi

# Copio l'eseguibile nella directory
echo "copio l'eseguibile"
cp "../centrale_controllo_stdof.o" "/opt/spinitalia/"
cp "power_detect.o" "/opt/spinitalia/"
chmod 777 "/opt/spinitalia/power_detect.o"

# Copio il servizio per systemd
echo "copio il servizio systemd per centrale_controllo_stdof. . ."
cp "stdof.service" "/lib/systemd/system"

echo "copio il servizio systemd per il power_detect"
cp "power_detect.service" "/lib/systemd/system"

echo "abilito il servizio"
systemctl enable stdof.service

echo "[PASSI DA ESEGUIRE A MANO]"
echo " 1. Abilitare il servizio power_detect tramite il comando"
echo "        systemctl enable power_detect.service"
echo " 2. Impostare l'indirizzo statico a 192.168.1.60"
echo "   2a. Entrare nella cartella di conman"
echo "        cd /usr/lib/conman/test"
echo "   2b. Segnarsi il nome della periferica"
echo "        ./get-services"
echo "   2c. Impostare l'indirizzo ip statico"
echo "        ./set-ipv4-method ethernet_78c5e5d9f1b7_cable manual 192.168.1.60 255.255.255.0 192.168.1.254"
