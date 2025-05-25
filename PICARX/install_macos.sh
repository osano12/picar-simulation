#!/bin/bash

# Script d'installation pour macOS pour le PiCar X Simulator

# Fonction pour afficher les messages d'information
info() {
    echo -e "\033[0;34m[INFO]\033[0m $1"
}

# Fonction pour afficher les messages d'erreur
error() {
    echo -e "\033[0;31m[ERROR]\033[0m $1"
}

# Fonction pour afficher les messages de succès
success() {
    echo -e "\033[0;32m[SUCCESS]\033[0m $1"
}

echo "=== Installation du PiCar X Simulator pour macOS ==="
echo "Ce script va installer toutes les dépendances nécessaires pour le simulateur."
echo "L'installation peut prendre plusieurs minutes."
echo ""

# Vérifier si Homebrew est installé
if ! command -v brew &> /dev/null; then
    info "Installation de Homebrew..."
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
fi

# Installer les dépendances de base
info "Installation des dépendances de base..."
brew update
brew install \
    git \
    python3 \
    cmake \
    wget

# Installer Docker pour exécuter ROS 2
info "Installation de Docker pour exécuter ROS 2..."
brew install --cask docker

# Vérifier si Docker est en cours d'exécution
if ! docker info &> /dev/null; then
    info "Veuillez démarrer Docker Desktop et réessayer."
    info "Une fois Docker démarré, exécutez à nouveau ce script."
    exit 1
fi

# Installer les dépendances Python
info "Installation des dépendances Python..."
pip3 install -U \
    setuptools \
    flask \
    flask-socketio \
    eventlet \
    numpy \
    opencv-python \
    matplotlib \
    pyyaml \
    transforms3d

# Configurer l'espace de travail
info "Configuration de l'espace de travail..."

# Définir le répertoire de l'espace de travail
WORKSPACE_DIR="$HOME/ros2_ws"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Créer les répertoires nécessaires
mkdir -p "$WORKSPACE_DIR/src/picar_simulator/picar_description"
mkdir -p "$WORKSPACE_DIR/src/picar_simulator/picar_gazebo"
mkdir -p "$WORKSPACE_DIR/src/picar_simulator/picar_control"
mkdir -p "$WORKSPACE_DIR/src/picar_simulator/picar_bringup"
mkdir -p "$WORKSPACE_DIR/src/picar_simulator/web_interface/templates"

# Copier les fichiers du projet
info "Copie des fichiers du projet..."

# Copier les fichiers URDF et Gazebo
cp "$SCRIPT_DIR/picar_description_detailed.urdf" "$WORKSPACE_DIR/src/picar_simulator/picar_description/"
cp "$SCRIPT_DIR/picar_gazebo_detailed.gazebo" "$WORKSPACE_DIR/src/picar_simulator/picar_gazebo/"

# Copier les nœuds ROS
cp "$SCRIPT_DIR/controller_node.py" "$WORKSPACE_DIR/src/picar_simulator/picar_control/"
cp "$SCRIPT_DIR/sensor_sim_node.py" "$WORKSPACE_DIR/src/picar_simulator/picar_control/"
cp "$SCRIPT_DIR/world_manager_node.py" "$WORKSPACE_DIR/src/picar_simulator/picar_control/"

# Copier les fichiers de lancement
cp "$SCRIPT_DIR/picar_simulator.launch.py" "$WORKSPACE_DIR/src/picar_simulator/picar_bringup/"

# Copier la configuration du bridge ROS-Unreal
cp "$SCRIPT_DIR/ros_unreal_bridge_config.yaml" "$WORKSPACE_DIR/src/picar_simulator/"

# Copier les templates web
cp -r "$SCRIPT_DIR/web_interface/templates" "$WORKSPACE_DIR/src/picar_simulator/web_interface/"

# Créer un package.xml pour le package ROS
cat > "$WORKSPACE_DIR/src/picar_simulator/package.xml" << EOF
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>picar_simulator</name>
  <version>0.1.0</version>
  <description>Simulateur interactif pour le PiCar X</description>
  <maintainer email="user@example.com">User</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>gazebo_ros</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

# Créer un script de lancement
cat > "$HOME/launch_picar_simulator.sh" << EOF
#!/bin/bash
docker run -it --rm \
    -p 8080:8080 \
    -v $WORKSPACE_DIR:/root/ros2_ws \
    osrf/ros:humble-desktop \
    bash -c "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 launch picar_simulator picar_simulator.launch.py"
EOF
chmod +x "$HOME/launch_picar_simulator.sh"

# Créer le fichier docker-compose.yml
cat > "$SCRIPT_DIR/docker-compose.yml" << EOF
version: '3'

services:
  ros_gazebo:
    image: osrf/ros:humble-desktop
    container_name: picar_simulator
    ports:
      - "8080:8080"
    volumes:
      - ./:/root/picar_simulator
      - $HOME/ros2_ws:/root/ros2_ws
    command: >
      bash -c "cd /root/picar_simulator &&
               source /opt/ros/humble/setup.bash &&
               source /root/ros2_ws/install/setup.bash &&
               ros2 launch picar_simulator picar_simulator.launch.py"

  dev:
    image: osrf/ros:humble-desktop
    container_name: picar_simulator_dev
    ports:
      - "8080:8080"
    volumes:
      - ./:/root/picar_simulator
      - $HOME/ros2_ws:/root/ros2_ws
    command: bash
    stdin_open: true
    tty: true
EOF

success "=== Installation terminée ==="
echo ""
echo "Pour utiliser le simulateur, ouvrez un terminal et exécutez:"
echo "  $HOME/launch_picar_simulator.sh"
echo ""
echo "Pour accéder à l'interface web, ouvrez un navigateur et allez à:"
echo "  http://localhost:8080"
echo ""
echo "Pour utiliser Docker directement:"
echo "  cd $SCRIPT_DIR && docker-compose up ros_gazebo"
echo ""
echo "Profitez de votre PiCar X Simulator!"