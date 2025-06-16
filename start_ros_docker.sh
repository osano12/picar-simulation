#!/bin/bash
set -e

# Fonction pour exécuter des commandes avec ou sans sudo
run_docker_cmd() {
    if docker ps &>/dev/null; then
        "$@"
    else
        sudo "$@"
    fi
}

# Vérifier si le conteneur existe déjà
if run_docker_cmd docker ps -a --format '{{.Names}}' | grep -q "ros2_humble"; then
    echo "Le conteneur ROS 2 Humble existe déjà."
    
    # Vérifier s'il est en cours d'exécution
    if ! run_docker_cmd docker ps --format '{{.Names}}' | grep -q "ros2_humble"; then
        echo "Démarrage du conteneur..."
        run_docker_cmd docker start ros2_humble
    else
        echo "Le conteneur est déjà en cours d'exécution."
    fi
else
    # Démarrer le conteneur avec docker-compose
    echo "Création et démarrage du conteneur ROS 2 Humble..."
    if command -v docker-compose &>/dev/null; then
        run_docker_cmd docker-compose up -d ros_gazebo
    else
        run_docker_cmd docker compose up -d ros_gazebo
    fi
fi

# Installer les dépendances dans le conteneur
echo "Installation des dépendances dans le conteneur..."
run_docker_cmd docker exec ros2_humble bash -c "apt-get update && apt-get install -y python3-pip && pip install flask flask_socketio numpy"

echo "==================================================="
echo "Conteneur ROS 2 Humble prêt à l'emploi!"
echo "==================================================="
echo "Pour entrer dans le conteneur:"
echo "  docker exec -it ros2_humble bash"
echo ""
echo "Pour lancer l'interface web du PiCar X:"
echo "  docker exec -it ros2_humble python3 /picarx/simple_web_interface.py"
echo ""
echo "Pour arrêter le conteneur:"
echo "  docker stop ros2_humble"
