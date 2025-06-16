# Architecture du Projet PiCar X Simulator

## Vue d'ensemble

Le PiCar X Simulator est une plateforme éducative interactive qui simule le comportement d'un robot PiCar X dans un environnement 3D. Le système est conçu pour fonctionner sur un Raspberry Pi 4/400 et utilise ROS 2, Gazebo pour la simulation physique, et Unreal Engine pour le rendu visuel.

```
┌─────────────────────────────────────────────────────────────────┐
│                      PiCar X Simulator                          │
│                                                                 │
│  ┌───────────────┐    ┌───────────────┐    ┌───────────────┐   │
│  │  Simulation   │    │  Contrôleurs  │    │  Interface    │   │
│  │  Physique     │◄──►│  & API        │◄──►│  Utilisateur  │   │
│  │  (Gazebo)     │    │  (ROS 2)      │    │  (Web/Flask)  │   │
│  └───────┬───────┘    └───────────────┘    └───────────────┘   │
│          │                                                      │
│          ▼                                                      │
│  ┌───────────────┐    ┌───────────────┐    ┌───────────────┐   │
│  │  Rendu 3D     │    │  Scénarios    │    │  Système      │   │
│  │  (Unreal      │    │  Pédagogiques │    │  d'Évaluation │   │
│  │  Engine)      │    │  & Missions   │    │  & Logs       │   │
│  └───────────────┘    └───────────────┘    └───────────────┘   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## Modules principaux

### 1. Simulation Physique (Gazebo)

Responsable de la simulation précise des aspects physiques du PiCar X:
- Dynamique du véhicule (masse, friction, suspension)
- Comportement des moteurs et servomoteurs
- Simulation des capteurs (ultrasons, IR, caméra, IMU)
- Environnement physique (collision, gravité, surfaces)

**Topics ROS:**
- `/picar/physics/state` - État physique du robot
- `/picar/sensors/ultrasonic` - Données du capteur ultrason
- `/picar/sensors/ir_left`, `/picar/sensors/ir_right` - Données des capteurs IR
- `/picar/sensors/camera` - Flux vidéo de la caméra
- `/picar/sensors/imu` - Données de l'IMU
- `/picar/actuators/motors` - Commandes des moteurs
- `/picar/actuators/steering` - Commandes de direction
- `/picar/actuators/camera_pan_tilt` - Commandes de la caméra rotative

### 2. Rendu 3D (Unreal Engine)

Responsable de la visualisation réaliste de l'environnement:
- Rendu des différentes zones (atelier, circuit urbain, parcours tout-terrain, zone défi)
- Effets visuels (conditions météo, cycle jour/nuit)
- Animations des éléments interactifs (feux tricolores, piétons, obstacles mobiles)
- Rendu de la caméra embarquée avec effets (distorsion, bruit)

**Communication:**
- Interface avec Gazebo via un plugin dédié
- Synchronisation des états physiques pour le rendu visuel

### 3. Contrôleurs & API (ROS 2)

Couche intermédiaire qui fournit:
- API Python de haut niveau pour le contrôle du robot
- Contrôleurs PID pour la vitesse et la direction
- Gestion des événements et callbacks
- Algorithmes de navigation et de perception

**Nodes ROS:**
- `picar_controller_node` - Gestion des contrôleurs PID
- `picar_api_node` - Exposition de l'API Python
- `picar_perception_node` - Traitement des données capteurs
- `picar_navigation_node` - Algorithmes de navigation
- `picar_battery_node` - Simulation de la batterie

### 4. Interface Utilisateur (Web/Flask + React)

Interface web responsive qui offre:
- Éditeur de code Python intégré
- Terminal pour l'exécution des commandes
- Visualisation du flux caméra
- Graphiques en temps réel (PID, vitesse, batterie)
- Tableau de bord des missions et badges

**Communication:**
- WebSocket pour les données en temps réel
- API REST pour les interactions non temps-réel
- Streaming vidéo via WebRTC ou MJPEG

### 5. Scénarios Pédagogiques & Missions

Système de progression qui comprend:
- Tutoriels pas à pas (atelier)
- Missions progressives (circuit urbain, tout-terrain)
- Défis chronométrés (zone défi)
- Système de badges et points

**Structure:**
- Définition des missions en YAML/JSON
- Conditions de réussite/échec
- Scripts d'initialisation des environnements
- Système de progression et déverrouillage

### 6. Système d'Évaluation & Logs

Responsable du suivi de la progression:
- Enregistrement des sessions (rosbag)
- Métriques de performance (temps, précision, efficacité)
- Replay des sessions
- Export des données pour analyse

## Flux de données

1. L'utilisateur écrit du code Python via l'interface web
2. Le code est envoyé au `picar_api_node` qui l'interprète
3. Les commandes sont traduites en messages ROS pour les actionneurs
4. Gazebo simule la physique et met à jour l'état du robot
5. Les données des capteurs simulés sont publiées sur les topics ROS
6. Le rendu visuel est mis à jour dans Unreal Engine
7. Les données sont renvoyées à l'interface utilisateur
8. Les logs et métriques sont enregistrés pour évaluation

## Modes de fonctionnement

### Mode Standard
- Simulation complète avec Gazebo et Unreal Engine
- Tous les capteurs et effets activés
- Performances optimisées pour Pi 4 (4GB+)

### Mode Light
- Rendu 3D simplifié ou désactivé
- Fréquence de simulation réduite
- Compatible avec Pi 3 ou Pi 4 (2GB)

### Mode Développeur
- Outils de débogage avancés
- Visualisation des topics ROS
- Métriques de performance système

## Structure des répertoires

```
picar_simulator/
├── ros2_ws/                      # Espace de travail ROS 2
│   ├── src/
│   │   ├── picar_description/    # Description URDF du robot
│   │   ├── picar_gazebo/         # Plugins et mondes Gazebo
│   │   ├── picar_control/        # Contrôleurs et API
│   │   ├── picar_msgs/           # Messages ROS personnalisés
│   │   └── picar_bringup/        # Launch files
│   └── ...
├── unreal_project/               # Projet Unreal Engine
│   ├── Content/
│   │   ├── Maps/                 # Environnements 3D
│   │   ├── PiCar/                # Modèles et matériaux
│   │   └── ...
│   └── ...
├── web_interface/                # Interface utilisateur web
│   ├── frontend/                 # React frontend
│   ├── backend/                  # Flask backend
│   └── ...
├── scenarios/                    # Scénarios pédagogiques
│   ├── tutorials/
│   ├── missions/
│   └── challenges/
├── docs/                         # Documentation
└── scripts/                      # Scripts utilitaires
```

## Étapes de développement

### Phase 1: Fondations
1. Configuration de l'environnement ROS 2 sur Raspberry Pi
2. Modélisation URDF du PiCar X
3. Intégration basique Gazebo
4. API Python fondamentale

### Phase 2: Simulation physique
1. Implémentation des capteurs simulés
2. Modélisation précise des moteurs et servos
3. Simulation de la batterie
4. Calibration des paramètres physiques

### Phase 3: Environnement 3D
1. Création des environnements dans Unreal Engine
2. Intégration Gazebo-Unreal
3. Effets visuels (météo, éclairage)
4. Optimisation des performances

### Phase 4: Interface et expérience utilisateur
1. Développement de l'interface web
2. Système de missions et progression
3. Visualisation des données
4. Système de logs et replay

### Phase 5: Finalisation
1. Tests d'intégration
2. Optimisation des performances
3. Documentation complète
4. Packaging et déploiement

## Considérations techniques

### Performance sur Raspberry Pi
- Utilisation de conteneurs pour isolation
- Profiling et optimisation CPU/RAM
- Gestion thermique
- Options de configuration pour différents modèles de Pi

### Extensibilité
- Architecture modulaire
- API bien documentée
- Possibilité d'ajouter de nouveaux capteurs/actionneurs
- Support pour extensions personnalisées

### Robustesse
- Gestion des erreurs et exceptions
- Mécanismes de récupération
- Sauvegarde automatique des progrès
- Diagnostics système