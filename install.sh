#!/bin/bash

# Script d'installation pour le PiCar X Simulator
# Ce script installe toutes les dépendances nécessaires et configure l'environnement

set -e  # Arrêter le script en cas d'erreur

echo "=== Installation du PiCar X Simulator ==="
echo "Ce script va installer toutes les dépendances nécessaires pour le simulateur."
echo "L'installation peut prendre plusieurs minutes."
echo ""

# Vérifier si le script est exécuté sur un Raspberry Pi
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [[ "$ID" == "raspbian" ]] || [[ "$ID_LIKE" == *"debian"* && $(uname -m) == "aarch64" ]]; then
        IS_RASPBERRY_PI=true
        echo "Système détecté: Raspberry Pi ($PRETTY_NAME)"
    else
        IS_RASPBERRY_PI=false
        echo "Système détecté: $PRETTY_NAME (non Raspberry Pi)"
    fi
else
    IS_RASPBERRY_PI=false
    echo "Système non identifié"
fi

# Vérifier si l'utilisateur est root
if [ "$EUID" -ne 0 ]; then
    echo "Ce script doit être exécuté en tant que root (utilisez sudo)."
    exit 1
fi

# Créer un répertoire temporaire
TEMP_DIR=$(mktemp -d)
cd "$TEMP_DIR"

# Mettre à jour les paquets
echo "Mise à jour des paquets..."
apt-get update
apt-get upgrade -y

# Installer les dépendances de base
echo "Installation des dépendances de base..."
apt-get install -y \
    git \
    python3-pip \
    python3-dev \
    python3-venv \
    build-essential \
    cmake \
    curl \
    wget \
    lsb-release \
    gnupg2

# Installer ROS 2
echo "Installation de ROS 2..."

# Ajouter le dépôt ROS 2
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Mettre à jour les paquets
apt-get update

# Installer ROS 2 Humble (version minimale)
if [ "$IS_RASPBERRY_PI" = true ]; then
    # Version légère pour Raspberry Pi
    apt-get install -y \
        ros-humble-ros-base \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-cv-bridge \
        python3-colcon-common-extensions
else
    # Version complète pour les autres systèmes
    apt-get install -y \
        ros-humble-desktop \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-cv-bridge \
        python3-colcon-common-extensions
fi

# Installer les dépendances Python
echo "Installation des dépendances Python..."
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

# Créer l'espace de travail ROS 2
echo "Configuration de l'espace de travail ROS 2..."
WORKSPACE_DIR="$HOME/ros2_ws"
mkdir -p "$WORKSPACE_DIR/src/picar_simulator"

# Copier les fichiers du projet
echo "Copie des fichiers du projet..."
PROJECT_DIR="/home/osano/PycharmProjects/PythonProject"

# Créer les répertoires nécessaires
mkdir -p "$WORKSPACE_DIR/src/picar_simulator/picar_description"
mkdir -p "$WORKSPACE_DIR/src/picar_simulator/picar_gazebo"
mkdir -p "$WORKSPACE_DIR/src/picar_simulator/picar_control"
mkdir -p "$WORKSPACE_DIR/src/picar_simulator/picar_bringup"
mkdir -p "$WORKSPACE_DIR/src/picar_simulator/web_interface/templates"

# Copier les fichiers URDF et Gazebo
cp "$PROJECT_DIR/picar_description_detailed.urdf" "$WORKSPACE_DIR/src/picar_simulator/picar_description/"
cp "$PROJECT_DIR/picar_gazebo_detailed.gazebo" "$WORKSPACE_DIR/src/picar_simulator/picar_gazebo/"

# Copier les nœuds ROS
cp "$PROJECT_DIR/controller_node.py" "$WORKSPACE_DIR/src/picar_simulator/picar_control/"
cp "$PROJECT_DIR/sensor_sim_node.py" "$WORKSPACE_DIR/src/picar_simulator/picar_control/"
cp "$PROJECT_DIR/world_manager_node.py" "$WORKSPACE_DIR/src/picar_simulator/picar_control/"

# Copier les fichiers de lancement
cp "$PROJECT_DIR/picar_simulator.launch.py" "$WORKSPACE_DIR/src/picar_simulator/picar_bringup/"

# Copier la configuration du bridge ROS-Unreal
cp "$PROJECT_DIR/ros_unreal_bridge_config.yaml" "$WORKSPACE_DIR/src/picar_simulator/"

# Rendre les scripts exécutables
find "$WORKSPACE_DIR/src" -name "*.py" -exec chmod +x {} \;

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

# Créer un CMakeLists.txt pour le package ROS
cat > "$WORKSPACE_DIR/src/picar_simulator/CMakeLists.txt" << EOF
cmake_minimum_required(VERSION 3.8)
project(picar_simulator)

# Trouver les dépendances
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo_ros REQUIRED)

# Installer les scripts Python
ament_python_install_package(\${PROJECT_NAME})

# Installer les scripts Python exécutables
install(PROGRAMS
  picar_control/controller_node.py
  picar_control/sensor_sim_node.py
  picar_control/world_manager_node.py
  DESTINATION lib/\${PROJECT_NAME}
)

# Installer les fichiers de lancement
install(DIRECTORY
  picar_bringup/
  DESTINATION share/\${PROJECT_NAME}
)

# Installer les fichiers URDF et Gazebo
install(DIRECTORY
  picar_description/
  picar_gazebo/
  DESTINATION share/\${PROJECT_NAME}
)

# Installer les fichiers de configuration
install(FILES
  ros_unreal_bridge_config.yaml
  DESTINATION share/\${PROJECT_NAME}
)

ament_package()
EOF

# Compiler l'espace de travail
echo "Compilation de l'espace de travail ROS 2..."
cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash
colcon build

# Ajouter le setup.bash au .bashrc
if ! grep -q "source $WORKSPACE_DIR/install/setup.bash" "$HOME/.bashrc"; then
    echo "# Source PiCar X Simulator workspace" >> "$HOME/.bashrc"
    echo "source $WORKSPACE_DIR/install/setup.bash" >> "$HOME/.bashrc"
fi

# Créer un script de lancement
cat > "$HOME/launch_picar_simulator.sh" << EOF
#!/bin/bash
source /opt/ros/humble/setup.bash
source $WORKSPACE_DIR/install/setup.bash
ros2 launch picar_simulator picar_simulator.launch.py
EOF

chmod +x "$HOME/launch_picar_simulator.sh"

# Nettoyer
cd "$HOME"
rm -rf "$TEMP_DIR"

echo ""
echo "=== Installation terminée ==="
echo "Pour utiliser le simulateur, ouvrez un nouveau terminal et exécutez:"
echo "$HOME/launch_picar_simulator.sh"
echo ""
echo "Pour accéder à l'interface web, ouvrez un navigateur et allez à:"
echo "http://localhost:8080"
echo ""
echo "Profitez de votre PiCar X Simulator!"