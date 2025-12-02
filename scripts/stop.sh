#!/bin/bash
# Script d'arrêt du conteneur MakersPet Loki Web

CONTAINER_NAME="makerspet-loki-web"

echo "Stopping MakersPet Loki Web Navigation..."
podman stop ${CONTAINER_NAME}
echo "Container stopped!"
