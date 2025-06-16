#!/bin/bash
# Script d'installation de ROS 2 Humble et des dépendances du PiCar X
# Usage: ./install_ros_humble.sh [--docker]

set -e

print_header() {
    echo ""
    echo "======================================================"
    echo "$1"
    echo "======================================================"
    echo ""
}

print_info() {
    echo -e "\033[1;34m[INFO] $1\033[0m"
}

print_warning() {
    echo -e "\033[1;33m[ATTENTION] $1\033[0m"
}

print_error() {
    echo -e "\033[1;31m[ERREUR] $1\033[0m"
}

print_success() {
    echo -e "\033[1;32m[SUCCÈS] $1\033[0m"
}

install_ros_native() {
    print_header "Installation de ROS 2 Humble sur Ubuntu"
    
    # Vérifier la version d'Ubuntu
    if [ "$(lsb_release -sc)" != "jammy" ] && [ "$(lsb_release -sc)" != "focal" ]; then
        print_warning "Ce script est conçu pour Ubuntu 20.04 ou 22.04."
        read -p "Voulez-vous continuer quand même? (o/N): " response
        if [[ ! "$response" =~ ^([oO][uU][iI]|[oO])$ ]]; then
            print_info "Installation annulée. Essayez l'option --docker à la place."
            exit 0
        fi
    fi
    
    # Vérifier si nous sommes sur Ubuntu 24.04 (Noble)
    if [ "$(lsb_release -sc)" = "noble" ]; then
        print_warning "Vous utilisez Ubuntu 24.04 (Noble). ROS 2 Humble est conçu pour Ubuntu 22.04."
        print_warning "Sur Ubuntu 24.04, il est recommandé d'utiliser ROS 2 Iron ou Docker."
        read -p "Voulez-vous continuer avec l'installation Docker à la place? (O/n): " use_docker
        if [[ "$use_docker" =~ ^([oO][uU][iI]|[oO]|)$ ]]; then
            setup_docker
            return
        fi
        print_warning "Tentative d'installation native, mais des problèmes de compatibilité peuvent survenir."
    fi

    # Nettoyer complètement les configurations ROS existantes
    print_info "Nettoyage des configurations ROS existantes..."
    sudo rm -f /etc/apt/sources.list.d/ros*.list
    sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg
    
    # Installer les prérequis
    print_info "Installation des prérequis..."
    sudo apt update
    sudo apt install -y software-properties-common gnupg lsb-release curl
    
    # Demander à l'utilisateur de choisir entre nettoyage complet ou installation minimale
    print_info "Détection d'une installation ROS préexistante..."
    read -p "Souhaitez-vous (1) nettoyer complètement les paquets ROS existants ou (2) installer uniquement les dépendances web? (1/2): " ros_clean
    
    if [ "$ros_clean" = "1" ]; then
        print_info "Suppression des paquets ROS existants..."
        sudo apt purge -y ros-* || true
        sudo apt autoremove -y
    else
        print_info "Installation des dépendances web uniquement..."
        install_web_interface
        return
    fi
    
    # Ajouter le dépôt ROS 2
    print_info "Ajout du dépôt ROS 2..."
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    # Utiliser le bon nom de code Ubuntu
    UBUNTU_CODENAME=$(lsb_release -sc)
    if [ "$UBUNTU_CODENAME" = "noble" ]; then
        # Sur Noble (24.04), nous utilisons le dépôt jammy (22.04) pour Humble
        UBUNTU_CODENAME="jammy"
        print_info "Utilisation du dépôt Ubuntu 22.04 (Jammy) pour ROS 2 Humble sur Ubuntu 24.04"
    fi
    
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" | sudo tee /etc/apt/sources.list.d/ros2-humble.list > /dev/null
    
    # Installer ROS 2 Humble
    print_info "Installation de ROS 2 Humble..."
    sudo apt update || true
    sudo apt install -y ros-humble-desktop || {
        print_error "L'installation de ROS 2 Humble a échoué."
        print_info "Tentative d'installation des dépendances minimales..."
        install_web_interface
        print_info "Utilisation de Docker recommandée. Exécutez ce script avec l'option 2."
        return 1
    }
    
    # Installer les outils de développement
    print_info "Installation des outils de développement..."
    sudo apt install -y python3-rosdep python3-colcon-common-extensions
    sudo rosdep init || true
    rosdep update
    
    # Installer les dépendances pour l'interface web
    print_info "Installation des dépendances pour l'interface web..."
    pip install flask flask_socketio numpy
    
    # Configurer l'environnement
    print_info "Configuration de l'environnement..."
    if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    fi
    
    # Sourcer l'environnement ROS 2
    source /opt/ros/humble/setup.bash
    
    print_success "ROS 2 Humble a été installé avec succès!"
    print_info "Pour commencer à utiliser ROS 2, ouvrez un nouveau terminal ou exécutez:"
    echo "  source ~/.bashrc"
    echo ""
    print_info "Pour tester l'installation, exécutez:"
    echo "  ros2 topic list"
    echo ""
}

setup_docker() {
    print_header "Configuration de ROS 2 Humble avec Docker"
    
    # Vérifier si Docker est installé
    if ! command -v docker &> /dev/null; then
        print_info "Installation de Docker..."
        sudo apt update
        sudo apt install -y docker.io || sudo apt install -y docker-ce
        
        # Installer docker-compose
        if ! command -v docker-compose &> /dev/null; then
            if command -v pip &> /dev/null; then
                print_info "Installation de docker-compose via pip..."
                pip install docker-compose
            else
                print_info "Installation de python3-pip et docker-compose..."
                sudo apt install -y python3-pip
                pip install docker-compose
            fi
        fi
        
        # Activer et démarrer le service Docker
        sudo systemctl enable --now docker || true
        
        # Ajouter l'utilisateur au groupe docker
        sudo usermod -aG docker $USER
        print_warning "Vous devrez vous déconnecter et vous reconnecter pour utiliser Docker sans sudo."
        print_info "Pour éviter de vous déconnecter maintenant, vous pouvez utiliser 'sudo docker' pour la suite."
    else
        print_info "Docker est déjà installé."
    fi
    
    # Créer le fichier docker-compose.yml
    print_info "Création du fichier docker-compose.yml..."
    
    # Obtenir le chemin absolu du répertoire actuel
    CURRENT_DIR="$(pwd)"
    
    cat > docker-compose.yml << EOF
version: '3'
services:
  ros_gazebo:
    image: osrf/ros:humble-desktop-full
    container_name: ros2_humble
    network_mode: "host"
    privileged: true
    environment:
      - DISPLAY=\$DISPLAY
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - ${CURRENT_DIR}:/picarx
    working_dir: /picarx
    command: bash -c "source /opt/ros/humble/setup.bash && sleep infinity"
EOF
    
    print_info "Téléchargement de l'image Docker ROS 2 Humble..."
    docker pull osrf/ros:humble-desktop-full || sudo docker pull osrf/ros:humble-desktop-full
    
    # Créer le script de démarrage
    print_info "Création du script de démarrage..."
    
    cat > start_ros_docker.sh << 'EOF'
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
EOF
    
    chmod +x start_ros_docker.sh
    
    # Créer un script pour l'interface web dans Docker
    print_info "Création d'un script pour lancer l'interface web dans Docker..."
    
    cat > run_web_interface_docker.sh << 'EOF'
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
EOF
    
    chmod +x run_web_interface_docker.sh
    
    # Installer les dépendances locales pour l'interface web
    print_info "Installation des dépendances locales pour l'interface web..."
    pip install flask flask_socketio numpy || sudo pip install flask flask_socketio numpy
    
    print_success "Configuration Docker terminée!"
    print_info "Pour démarrer le conteneur ROS 2 Humble et l'interface web:"
    echo "  ./run_web_interface_docker.sh"
    echo ""
    print_info "Pour uniquement démarrer le conteneur:"
    echo "  ./start_ros_docker.sh"
    echo ""
    print_info "Pour entrer dans le conteneur et utiliser ROS 2:"
    echo "  docker exec -it ros2_humble bash"
    echo ""
}

# Installer l'interface web
install_web_interface() {
    print_header "Installation des dépendances pour l'interface web"
    
    # Vérifier si pip est installé
    if ! command -v pip &> /dev/null && ! command -v pip3 &> /dev/null; then
        print_info "Installation de pip..."
        sudo apt update
        sudo apt install -y python3-pip
    fi
    
    # Déterminer quelle commande pip utiliser
    PIP_CMD="pip"
    if ! command -v pip &> /dev/null && command -v pip3 &> /dev/null; then
        PIP_CMD="pip3"
    fi
    
    # Installer les dépendances Python
    print_info "Installation des dépendances Python..."
    
    # Essayer d'installer avec différentes méthodes si nécessaire
    if ! $PIP_CMD install flask flask_socketio numpy; then
        print_warning "L'installation avec $PIP_CMD a échoué, tentative avec sudo..."
        if ! sudo $PIP_CMD install flask flask_socketio numpy; then
            print_warning "Installation directe avec apt..."
            sudo apt install -y python3-flask python3-socketio python3-numpy || {
                print_error "L'installation des dépendances a échoué."
                print_info "Vous pouvez essayer d'installer manuellement avec:"
                echo "  sudo pip install flask flask_socketio numpy"
                return 1
            }
        fi
    fi
    
    # Créer un script pour lancer facilement l'interface web
    print_info "Création d'un script de lancement pour l'interface web..."
    
    cat > run_web_interface.sh << 'EOF'
#!/bin/bash

echo "Démarrage de l'interface web du PiCar X..."
python3 simple_web_interface.py

# Si l'exécution échoue, proposer d'installer les dépendances
if [ $? -ne 0 ]; then
    echo "Erreur au démarrage de l'interface web."
    echo "Voulez-vous installer les dépendances manquantes? (o/N)"
    read -r response
    if [[ "$response" =~ ^([oO][uU][iI]|[oO])$ ]]; then
        pip install flask flask_socketio numpy || sudo pip install flask flask_socketio numpy
        echo "Tentative de redémarrage de l'interface web..."
        python3 simple_web_interface.py
    fi
fi
EOF
    
    chmod +x run_web_interface.sh
    
    print_success "Installation des dépendances pour l'interface web terminée!"
    print_info "Pour lancer l'interface web, exécutez:"
    echo "  ./run_web_interface.sh"
}

# Menu principal
print_header "Installation de ROS 2 Humble pour PiCar X"
echo "Ce script vous aidera à installer ROS 2 Humble et les dépendances nécessaires"
echo "pour utiliser l'interface web du PiCar X."
echo ""
echo "Options disponibles:"
echo "1. Installation native de ROS 2 Humble (recommandé pour Ubuntu 22.04)"
echo "2. Installation via Docker (recommandé pour les autres distributions)"
echo "3. Installer uniquement les dépendances pour l'interface web"
echo "4. Quitter"
echo ""

# Gérer les arguments en ligne de commande
if [ "$1" == "--docker" ]; then
    choice=2
else
    read -p "Votre choix (1-4): " choice
fi

case $choice in
    1)
        install_ros_native
        ;;
    2)
        setup_docker
        ;;
    3)
        install_web_interface
        ;;
    4)
        print_info "Installation annulée."
        exit 0
        ;;
    *)
        print_error "Choix invalide."
        exit 1
        ;;
esac

print_header "Installation terminée"
echo "Vous pouvez maintenant tester l'interface web du PiCar X:"
echo "  cd /home/osano/Documents/PICARX"
echo "  python3 test_interface.py"
echo ""
print_info "N'oubliez pas de vérifier le README pour plus d'informations."