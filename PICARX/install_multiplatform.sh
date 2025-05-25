#!/usr/bin/env bash

# Script d'installation multiplateforme pour le PiCar X Simulator
# Ce script détecte le système d'exploitation et installe les dépendances appropriées

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

# Fonction pour détecter le système d'exploitation
detect_os() {
    case "$(uname -s)" in
        Linux*)     OS="linux";;
        Darwin*)    OS="macos";;
        CYGWIN*|MINGW*|MSYS*) OS="windows";;
        *)          OS="unknown";;
    esac
    echo $OS
}

# Fonction pour installer les dépendances sur Linux
install_linux_deps() {
    info "Installation des dépendances pour Linux..."
    
    # Vérifier si l'utilisateur est root
    if [ "$EUID" -ne 0 ]; then
        error "Ce script doit être exécuté en tant que root sur Linux (utilisez sudo)."
        exit 1
    fi
    
    # Détecter la distribution Linux
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        DISTRO=$ID
    else
        DISTRO="unknown"
    fi
    
    # Vérifier si c'est un Raspberry Pi
    if [[ "$DISTRO" == "raspbian" ]] || [[ "$ID_LIKE" == *"debian"* && $(uname -m) == "aarch64" ]]; then
        IS_RASPBERRY_PI=true
        info "Système détecté: Raspberry Pi ($PRETTY_NAME)"
    else
        IS_RASPBERRY_PI=false
        info "Système détecté: $PRETTY_NAME"
    fi
    
    # Mettre à jour les paquets
    info "Mise à jour des paquets..."
    apt-get update
    apt-get upgrade -y
    
    # Installer les dépendances de base
    info "Installation des dépendances de base..."
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
    info "Installation de ROS 2..."
    
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
}

# Fonction pour installer les dépendances sur macOS
install_macos_deps() {
    info "Installation des dépendances pour macOS..."
    
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
}

# Fonction pour installer les dépendances sur Windows
install_windows_deps() {
    info "Installation des dépendances pour Windows..."
    
    # Vérifier si Chocolatey est installé
    if ! command -v choco &> /dev/null; then
        info "Installation de Chocolatey..."
        powershell -Command "Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://chocolatey.org/install.ps1'))"
    fi
    
    # Installer les dépendances de base
    info "Installation des dépendances de base..."
    choco install -y git python3 cmake wget
    
    # Installer Docker pour exécuter ROS 2
    info "Installation de Docker pour exécuter ROS 2..."
    choco install -y docker-desktop
    
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
}

# Fonction pour configurer l'espace de travail
setup_workspace() {
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
}

# Fonction pour créer les scripts de lancement
create_launch_scripts() {
    info "Création des scripts de lancement..."
    
    WORKSPACE_DIR="$HOME/ros2_ws"
    
    # Script de lancement pour Linux
    if [ "$OS" == "linux" ]; then
        cat > "$HOME/launch_picar_simulator.sh" << EOF
#!/bin/bash
source /opt/ros/humble/setup.bash
source $WORKSPACE_DIR/install/setup.bash
ros2 launch picar_simulator picar_simulator.launch.py
EOF
        chmod +x "$HOME/launch_picar_simulator.sh"
    
    # Script de lancement pour macOS et Windows (utilisant Docker)
    else
        cat > "$HOME/launch_picar_simulator.sh" << EOF
#!/bin/bash
docker run -it --rm \
    -p 8080:8080 \
    -v $WORKSPACE_DIR:/root/ros2_ws \
    osrf/ros:humble-desktop \
    bash -c "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 launch picar_simulator picar_simulator.launch.py"
EOF
        chmod +x "$HOME/launch_picar_simulator.sh"
        
        # Pour Windows, créer également un fichier .bat
        if [ "$OS" == "windows" ]; then
            cat > "$HOME/launch_picar_simulator.bat" << EOF
@echo off
docker run -it --rm ^
    -p 8080:8080 ^
    -v %USERPROFILE%/ros2_ws:/root/ros2_ws ^
    osrf/ros:humble-desktop ^
    bash -c "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 launch picar_simulator picar_simulator.launch.py"
EOF
        fi
    fi
}

# Fonction pour compiler l'espace de travail (Linux uniquement)
build_workspace() {
    if [ "$OS" == "linux" ]; then
        info "Compilation de l'espace de travail ROS 2..."
        WORKSPACE_DIR="$HOME/ros2_ws"
        cd "$WORKSPACE_DIR"
        source /opt/ros/humble/setup.bash
        colcon build
        
        # Ajouter le setup.bash au .bashrc
        if ! grep -q "source $WORKSPACE_DIR/install/setup.bash" "$HOME/.bashrc"; then
            echo "# Source PiCar X Simulator workspace" >> "$HOME/.bashrc"
            echo "source $WORKSPACE_DIR/install/setup.bash" >> "$HOME/.bashrc"
        fi
    else
        info "Compilation ignorée pour les systèmes non-Linux. Docker sera utilisé à la place."
    fi
}

# Fonction pour créer le fichier docker-compose.yml
create_docker_compose() {
    info "Création du fichier docker-compose.yml..."
    
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    
    cat > "$SCRIPT_DIR/docker-compose.yml" << EOF
version: '3'

services:
  ros_gazebo:
    image: osrf/ros:humble-desktop
    container_name: picar_simulator
    network_mode: host
    privileged: true
    environment:
      - DISPLAY=\${DISPLAY}
      - QT_X11_NO_MITSHM=1
    volumes:
      - ./:/root/picar_simulator
      - $HOME/ros2_ws:/root/ros2_ws
      - /tmp/.X11-unix:/tmp/.X11-unix
    command: >
      bash -c "cd /root/picar_simulator &&
               source /opt/ros/humble/setup.bash &&
               source /root/ros2_ws/install/setup.bash &&
               ros2 launch picar_simulator picar_simulator.launch.py"

  dev:
    image: osrf/ros:humble-desktop
    container_name: picar_simulator_dev
    network_mode: host
    privileged: true
    environment:
      - DISPLAY=\${DISPLAY}
      - QT_X11_NO_MITSHM=1
    volumes:
      - ./:/root/picar_simulator
      - $HOME/ros2_ws:/root/ros2_ws
      - /tmp/.X11-unix:/tmp/.X11-unix
    command: bash
    stdin_open: true
    tty: true
EOF
}

# Fonction pour créer le fichier .gitignore
create_gitignore() {
    info "Création du fichier .gitignore..."
    
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    
    cat > "$SCRIPT_DIR/.gitignore" << EOF
# Fichiers générés par Python
__pycache__/
*.py[cod]
*$py.class
*.so
.Python
env/
build/
develop-eggs/
dist/
downloads/
eggs/
.eggs/
lib/
lib64/
parts/
sdist/
var/
*.egg-info/
.installed.cfg
*.egg

# Fichiers générés par ROS
install/
log/
build/
.catkin_workspace

# Fichiers générés par l'IDE
.idea/
.vscode/
*.swp
*.swo

# Fichiers système
.DS_Store
Thumbs.db

# Fichiers Docker
.docker/

# Fichiers temporaires
*.tmp
*.temp
*.log
EOF
}

# Fonction principale
main() {
    echo "=== Installation du PiCar X Simulator ==="
    echo "Ce script va installer toutes les dépendances nécessaires pour le simulateur."
    echo "L'installation peut prendre plusieurs minutes."
    echo ""
    
    # Détecter le système d'exploitation
    OS=$(detect_os)
    info "Système d'exploitation détecté: $OS"
    
    # Installer les dépendances selon le système d'exploitation
    case $OS in
        linux)
            install_linux_deps
            ;;
        macos)
            install_macos_deps
            ;;
        windows)
            install_windows_deps
            ;;
        *)
            error "Système d'exploitation non pris en charge: $OS"
            exit 1
            ;;
    esac
    
    # Configurer l'espace de travail
    setup_workspace
    
    # Créer les scripts de lancement
    create_launch_scripts
    
    # Compiler l'espace de travail (Linux uniquement)
    build_workspace
    
    # Créer le fichier docker-compose.yml
    create_docker_compose
    
    # Créer le fichier .gitignore
    create_gitignore
    
    success "=== Installation terminée ==="
    echo ""
    echo "Pour utiliser le simulateur, ouvrez un nouveau terminal et exécutez:"
    echo "  $HOME/launch_picar_simulator.sh"
    if [ "$OS" == "windows" ]; then
        echo "  ou double-cliquez sur $HOME/launch_picar_simulator.bat"
    fi
    echo ""
    echo "Pour accéder à l'interface web, ouvrez un navigateur et allez à:"
    echo "  http://localhost:8080"
    echo ""
    echo "Pour utiliser Docker directement:"
    echo "  docker-compose up ros_gazebo"
    echo ""
    echo "Profitez de votre PiCar X Simulator!"
}

# Exécuter la fonction principale
main