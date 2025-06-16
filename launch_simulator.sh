#!/bin/bash

# Script de lancement pour le simulateur PiCar X

echo "=== Lancement du simulateur PiCar X ==="

# Vérifier si ROS 2 est installé
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "ROS 2 Humble n'est pas installé. Veuillez exécuter le script d'installation."
    exit 1
fi

# Vérifier si l'espace de travail existe
WORKSPACE_DIR="$HOME/ros2_ws"
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "L'espace de travail ROS 2 n'existe pas. Veuillez exécuter le script d'installation."
    exit 1
fi

# Sourcer ROS 2
source /opt/ros/humble/setup.bash

# Sourcer l'espace de travail
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
else
    echo "L'espace de travail n'est pas compilé. Compilation en cours..."
    cd "$WORKSPACE_DIR"
    colcon build
    source "$WORKSPACE_DIR/install/setup.bash"
fi

# Lancer le simulateur
echo "Lancement du simulateur..."
ros2 launch picar_simulator picar_simulator.launch.py

echo "Simulateur terminé."