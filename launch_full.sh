#!/bin/bash

# Script de lancement pour la version complète du simulateur PiCar X avec ROS 2

echo "=== Lancement du PiCar X Simulator - Version Complète ==="
echo

# Vérifier si Docker est installé et en cours d'exécution
if ! command -v docker &> /dev/null; then
    echo "Erreur: Docker n'est pas installé"
    echo "Veuillez installer Docker et réessayer"
    exit 1
fi

if ! docker info &> /dev/null; then
    echo "Erreur: Docker n'est pas en cours d'exécution"
    echo "Veuillez démarrer Docker et réessayer"
    exit 1
fi

# Vérifier si docker-compose est installé
if ! command -v docker-compose &> /dev/null; then
    echo "Installation de docker-compose..."
    sudo apt update && sudo apt install -y docker-compose
fi

echo "Démarrage du simulateur complet avec ROS 2 et Gazebo..."
echo "Interface web disponible sur:"
echo "  - Local: http://localhost:8080"
echo "  - Réseau: http://$(hostname -I | awk '{print $1}'):8080"
echo
echo "Appuyez sur Ctrl+C pour arrêter"
echo

# Construire et lancer avec Docker Compose
docker-compose up --build ros_gazebo