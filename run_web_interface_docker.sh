#!/bin/bash

# Démarrer le conteneur s'il n'est pas en cours d'exécution
if ! docker ps --format '{{.Names}}' | grep -q "ros2_humble"; then
    echo "Démarrage du conteneur ROS 2 Humble..."
    ./start_ros_docker.sh
fi

# Installer Flask et les dépendances si nécessaire
docker exec ros2_humble bash -c "pip install flask flask_socketio numpy"

# Lancer l'interface web
echo "Démarrage de l'interface web..."
docker exec -it ros2_humble python3 /picarx/simple_web_interface.py

# Instructions
echo "L'interface web est accessible à l'adresse: http://localhost:8080"
