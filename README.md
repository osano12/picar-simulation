<<<<<<< HEAD
# picar-simulation
picar-simulation'
=======
# PiCar X Simulator

Simulateur interactif pour le PiCar X, permettant de prototyper et d'apprendre avant de déployer sur le matériel réel. Compatible avec Windows, macOS et Linux.

## Description

Le PiCar X Simulator est une plateforme éducative interactive qui simule le comportement d'un robot PiCar X dans un environnement 3D. Le système est conçu pour fonctionner sur PC (Windows/Linux/Mac) et utilise ROS 2 Humble, Gazebo pour la simulation physique, et optionnellement Unreal Engine pour le rendu visuel photo-réaliste.

## Fonctionnalités

- Simulation précise du PiCar X avec ses caractéristiques physiques
- Capteurs simulés (ultrason, IR, caméra, IMU) avec bruit et latence réalistes
- Environnements variés (atelier, urbain, tout-terrain, défi)
- Conditions météorologiques et cycle jour/nuit
- Interface web pour le contrôle et la visualisation
- Scénarios pédagogiques progressifs
- **Compatibilité multiplateforme**: Windows, macOS et Linux

### Nouvelles fonctionnalités (Mises à jour récentes)

- Vue du dessus du robot avec orientation et grille pour meilleure visualisation
- Paramètres configurables pour le suivi de ligne (seuil, vitesse, facteur P)
- Bouton de remise à zéro complète pour réinitialiser les paramètres
- Gestion améliorée des erreurs de modules manquants
- Mode de suivi de ligne intégré avec contrôle proportionnel
- Interface repensée pour une meilleure expérience utilisateur

## Installation

### Prérequis

- **Pour tous les systèmes**: Git, Python 3.8+
- **Pour Windows et macOS**: Docker Desktop
- **Pour Linux**: ROS 2 Humble, Gazebo 11 (installés automatiquement par le script)

### Installation automatique (recommandée)

1. Clonez ce dépôt:
   ```bash
   git clone https://github.com/votre-username/picar-simulator.git
   cd picar-simulator
   ```

2. Exécutez le script d'installation approprié pour votre système d'exploitation:

   **Linux**:
   ```bash
   # Script d'installation multiplateforme (ancien)
   sudo ./install_multiplatform.sh
   
   # OU, pour ROS 2 Humble spécifiquement (nouveau script)
   chmod +x install_ros_humble.sh
   ./install_ros_humble.sh
   ```

   **macOS**:
   ```bash
   ./install_macos.sh
   ```

   **Windows** (dans PowerShell ou CMD):
   ```
   .\install_windows.bat
   ```

### Installation facile de ROS 2 Humble (Nouveau)

Pour installer uniquement ROS 2 Humble et les dépendances nécessaires pour l'interface web:

```bash
# Rendez le script exécutable
chmod +x install_ros_humble.sh

# Exécutez le script et suivez les instructions
./install_ros_humble.sh
```

Le script offre plusieurs options:
1. Installation native de ROS 2 Humble (recommandé pour Ubuntu 22.04)
2. Installation via Docker (recommandé pour les autres distributions)
3. Installation uniquement des dépendances pour l'interface web

### Installation avec Docker (toutes plateformes)

Si vous avez déjà Docker installé, vous pouvez simplement utiliser:

```bash
docker-compose up ros_gazebo
```

## Utilisation

### Lancement du simulateur

**Linux**:
```bash
~/launch_picar_simulator.sh
```

**macOS**:
```bash
~/launch_picar_simulator.sh
```

**Windows**:
Double-cliquez sur `%USERPROFILE%\launch_picar_simulator.bat`

### Accès à l'interface web

Ouvrez un navigateur et accédez à:
```
http://localhost:8080
```

### Modes de fonctionnement

- **Mode standard**: Simulation complète avec tous les capteurs et effets
- **Mode light**: Rendu simplifié pour PC modestes
- **Avec Unreal Engine**: Rendu photo-réaliste (nécessite Unreal Engine)

### Utilisation du suivi de ligne (Nouvelle fonctionnalité)

L'interface web inclut désormais un système de suivi de ligne configurable:

1. Accédez à la section "Suivi de ligne" dans l'interface web
2. Ajustez les paramètres selon vos besoins:
   - **Seuil de détection**: Détermine la sensibilité pour détecter la ligne (0-255)
   - **Vitesse de base**: Configure la vitesse globale du robot (5-50)
   - **Facteur P**: Contrôle l'intensité de la correction proportionnelle (0-2)
3. Cliquez sur le bouton "Suivi de ligne" pour démarrer
4. Le code Python correspondant sera généré et exécuté automatiquement
5. Utilisez le bouton "Arrêter" pour interrompre le suivi

Pour réinitialiser tous les paramètres et l'état du robot, utilisez le bouton "Remise à zéro complète".

## Structure du projet

```
picar_simulator/
├── controller_node.py            # Contrôleur PID pour le PiCar X
├── sensor_sim_node.py            # Simulation des capteurs
├── world_manager_node.py         # Gestion de l'environnement
├── web_interface.py              # Interface web
├── picar_simulator.launch.py     # Fichier de lancement ROS 2
├── picar_description_detailed.urdf # Description URDF du robot
├── picar_gazebo_detailed.gazebo  # Configuration Gazebo
├── install_multiplatform.sh      # Script d'installation multiplateforme
├── install_windows.bat           # Script d'installation Windows
├── install_macos.sh              # Script d'installation macOS
├── docker-compose.yml            # Configuration Docker
└── web_interface/                # Fichiers de l'interface web
    └── templates/                # Templates HTML
```

## Développement

Pour le développement, vous pouvez utiliser l'environnement Docker:

```bash
docker-compose up dev
```

Cela vous donnera accès à un terminal interactif dans le conteneur avec toutes les dépendances installées.

## Compatibilité multiplateforme

Le simulateur utilise Docker pour garantir une expérience cohérente sur tous les systèmes d'exploitation:

- **Linux**: Exécution native de ROS 2 et Gazebo
- **Windows et macOS**: Exécution via Docker avec accès à l'interface web

## Contribution

Les contributions sont les bienvenues! Veuillez consulter le fichier CONTRIBUTING.md pour plus d'informations.

## Licence

Ce projet est sous licence MIT - voir le fichier LICENSE pour plus de détails.

## Résolution des problèmes

### Modules Python manquants

Si vous rencontrez des erreurs liées à des modules Python manquants lors de l'exécution de l'interface web:

```bash
# Installez les modules requis
pip install flask flask_socketio numpy
```

L'interface web a été améliorée pour afficher des messages d'erreur clairs et des instructions d'installation lorsque des modules sont manquants.

### Problèmes avec ROS 2

Si vous utilisez Docker et rencontrez des problèmes avec ROS 2:

```bash
# Démarrez ou redémarrez le conteneur
./start_ros_docker.sh

# Accédez au terminal du conteneur
docker exec -it ros2_humble bash
```

## Contact

Pour toute question ou suggestion, veuillez ouvrir une issue sur GitHub ou contacter l'auteur à user@example.com.
>>>>>>> ce3c94c (Description de tes modifications)
