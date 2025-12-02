#!/bin/bash
# Script de redémarrage du conteneur MakersPet Loki Web

echo "Restarting MakersPet Loki Web Navigation..."
./scripts/stop.sh
sleep 2
./scripts/start.sh
