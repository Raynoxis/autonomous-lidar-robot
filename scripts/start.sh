#!/bin/bash
# Script de démarrage du conteneur MakersPet Loki Web

CONTAINER_NAME="makerspet-loki-web"
IMAGE_NAME="makerspet-loki-web:latest"

echo "Starting MakersPet Loki Web Navigation..."

# Vérifier si le conteneur existe déjà
if podman ps -a --format "{{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
    echo "Container ${CONTAINER_NAME} exists. Starting..."
    podman start ${CONTAINER_NAME}
else
    echo "Creating and starting new container..."
    podman run -d \
        --name ${CONTAINER_NAME} \
        --network host \
        --privileged \
        -v ./maps:/app/maps:Z \
        -v ./logs:/app/logs:Z \
        -v ./config:/app/config:Z \
        -v ./web:/app/web:Z \
        ${IMAGE_NAME}
fi

echo "Container started!"
echo ""
echo "Access web interface at: http://$(hostname -I | awk '{print $1}'):8080/web/index.html"
echo "RosBridge WebSocket: ws://$(hostname -I | awk '{print $1}'):9092"
echo ""
echo "View logs with: podman logs -f ${CONTAINER_NAME}"
