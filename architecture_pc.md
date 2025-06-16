# Architecture du Projet PiCar X Simulator pour PC

## Vue d'ensemble

Le PiCar X Simulator est une plateforme éducative interactive qui simule le comportement d'un robot PiCar X dans un environnement 3D. Le système est conçu pour fonctionner sur PC (Windows/Linux/Mac) et utilise ROS 2 Humble, Gazebo pour la simulation physique, et Unreal Engine pour le rendu visuel photo-réaliste.

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         PiCar X Simulator                               │
│                                                                         │
│  ┌───────────────┐    ┌───────────────┐    ┌───────────────────────┐   │
│  │  Simulation   │    │  Contrôleurs  │    │  Interface            │   │
│  │  Physique     │◄──►│  & API        │◄──►│  Utilisateur          │   │
│  │  (Gazebo)     │    │  (ROS 2)      │    │  (React + FastAPI)    │   │
│  └───────┬───────┘    └───────────────┘    └───────────────────────┘   │
│          │                                                              │
│          ▼                                                              │
│  ┌───────────────┐    ┌───────────────┐    ┌───────────────────────┐   │
│  │  Rendu 3D     │    │  Scénarios    │    │  Système              │   │
│  │  (Unreal      │◄──►│  Pédagogiques │    │  d'Évaluation         │   │
│  │  Engine)      │    │  & Missions   │    │  & Logs (rosbag)      │   │
│  └───────────────┘    └───────────────┘    └───────────────────────┘   │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

## Modules principaux

### 1. Simulation Physique (Gazebo)

Responsable de la simulation précise des aspects physiques du PiCar X:
- Dynamique du véhicule (masse, friction, suspension)
- Comportement des moteurs et servomoteurs
- Simulation des capteurs (ultrasons, IR, caméra, IMU)
- Environnement physique (collision, gravité, surfaces)
- Simulation de la batterie (décharge, chute de tension)

**Caractéristiques physiques:**
- Châssis: 450g, garde au sol 2cm
- Roues: Ø 6cm, largeur 2cm, μ variable par surface (bitume 0.8, carrelage 0.6, herbe 0.4)
- Moteurs: Kv 1200 RPM/V, couple 0.1 Nm, τ électrique ≈ 50ms, pertes 10%, jeu mécanique deadband ±2°
- Dynamique: inertie, Cd aérodynamique, frottements internes, slippage si accélération > 1m/s² ou pente > 15°

**Capteurs simulés:**
- IMU: gyro dérive 0.02°/s, accéléro bruit ±0.02g, 200Hz
- Ultrasons: 2cm–4m, bruit ±1cm, latence 50–70ms
- IR: détecteur de ligne, fausse détection 5%
- Caméra: 640×480@30FPS, flux MJPEG, latence 100ms, bruit σ=10%, distortion lens
- Batterie: chute de tension sous charge (<6V), décharge continue simulée

**Aléas pédagogiques:**
- Pannes capteurs (1 drop-out/10min)
- Reboot automatique pour inviter à gérer l'erreur

### 2. Rendu 3D (Unreal Engine via Ignition-Unreal)

Responsable de la visualisation photo-réaliste de l'environnement:
- Rendu des différentes zones (atelier, circuit urbain, parcours tout-terrain, zone défi)
- Effets visuels avancés (conditions météo, cycle jour/nuit)
- Éclairage dynamique et ombres
- Matériaux réalistes (réflexion, réfraction)
- Animations des éléments interactifs (feux tricolores, piétons, obstacles mobiles)
- Rendu de la caméra embarquée avec effets (distorsion, bruit)

**Caractéristiques météorologiques:**
- Pluie: réduction de l'adhérence (μ), augmentation de la latence des capteurs (+20ms)
- Brouillard: réduction de la portée de la caméra (≤30%)
- Cycle jour/nuit avec éclairage dynamique
- LED frontales simulées pour la vision nocturne

**Communication:**
- Bridge ROS↔Unreal pour synchroniser états physiques et visuels
- Transfert bidirectionnel des données de pose et d'environnement

### 3. Architecture ROS 2 & Flux de Topics

#### 3.1 controller_node
- Souscrit à `/cmd_vel` (Twist linéaire+angulaire)
- PID vitesse et braquage (50Hz), anti-windup, auto-tune
- Callbacks: `on_line_lost()`, `on_ultrasonic_close()`
- Publie sur `/picar/motors/cmd` et `/picar/steering/cmd`

**Topics publiés:**
- `/picar/motors/cmd` - Commandes des moteurs
- `/picar/steering/cmd` - Commandes de direction
- `/picar/camera/pan_tilt_cmd` - Commandes de la caméra rotative

**Topics souscrits:**
- `/cmd_vel` - Commandes de vitesse
- `/picar/sensors/ultrasonic` - Données du capteur ultrason
- `/picar/sensors/ir_left`, `/picar/sensors/ir_right` - Données des capteurs IR

#### 3.2 sim_picar_node
- Consomme `/cmd_vel`, calcule pose, publie `/odom`, `/joint_states`
- Simulation de la dynamique du véhicule
- Gestion des collisions et interactions avec l'environnement

**Topics publiés:**
- `/picar/odom` - Odométrie du robot
- `/picar/joint_states` - États des articulations
- `/tf` - Transformations du robot

**Topics souscrits:**
- `/picar/motors/cmd` - Commandes des moteurs
- `/picar/steering/cmd` - Commandes de direction
- `/picar/world/friction` - Coefficient de friction du sol

#### 3.3 sensor_sim_node
- Écoute `/odom`, génère `/imu`, `/scan` (ultrasons), `/camera/image_raw`, `/battery`
- Ajout de bruit et latence réalistes
- Simulation des pannes aléatoires des capteurs

**Topics publiés:**
- `/picar/sensors/imu` - Données de l'IMU
- `/picar/sensors/ultrasonic` - Données du capteur ultrason
- `/picar/sensors/ir_left`, `/picar/sensors/ir_right` - Données des capteurs IR
- `/picar/camera/image_raw` - Flux vidéo de la caméra
- `/picar/power/battery` - État de la batterie

**Topics souscrits:**
- `/picar/odom` - Odométrie du robot
- `/picar/world/weather` - Conditions météorologiques

#### 3.4 world_manager_node
- Change friction, pente, météo, cycle jour/nuit selon zone
- Gestion des événements environnementaux
- Contrôle des objets dynamiques (piétons, obstacles mobiles)

**Topics publiés:**
- `/picar/world/friction` - Coefficient de friction du sol
- `/picar/world/weather` - Conditions météorologiques
- `/picar/world/time` - Heure simulée (jour/nuit)
- `/picar/world/events` - Événements environnementaux

#### 3.5 ui_bridge_node
- Relaye `/camera/image_raw`, `/odom`, `/battery` vers front-end
- Communication via WebSocket/REST
- Compression et optimisation des données pour le transfert

**Topics souscrits:**
- `/picar/camera/image_raw` - Flux vidéo de la caméra
- `/picar/odom` - Odométrie du robot
- `/picar/power/battery` - État de la batterie
- `/picar/sensors/imu` - Données de l'IMU
- `/picar/sensors/ultrasonic` - Données du capteur ultrason
- `/picar/sensors/ir_left`, `/picar/sensors/ir_right` - Données des capteurs IR

### 4. Interface Utilisateur (React + FastAPI)

Interface web responsive qui offre:
- Éditeur de code Python intégré
- Console ROS
- Mini-map de l'environnement
- Streaming de la caméra
- Graphiques en temps réel (PID, vitesse, batterie)
- Tableau de bord des missions et badges

**Fonctionnalités de gamification:**
- Points et système de score
- Badges (Line Follower, Obstacle Dodger, etc.)
- Classement local
- Missions avec feedback instantané

**Communication:**
- WebSocket pour les données en temps réel
- API REST pour les interactions non temps-réel
- Streaming vidéo via WebRTC ou MJPEG

### 5. Scénarios Pédagogiques & Missions

Système de progression qui comprend:
- Atelier tutoriel: espace clos, marquages au sol, `drive()`, `turn()`, `stop()`
- Circuit urbain: feux tricolores, passages piétons, détection signalisation (vision)
- Parcours tout-terrain: bosses, rampes, flaques, drift, ponts à bascule (équilibre)
- Zone défi: labyrinthe, piétons/obstacles mobiles, challenges chronométrés, scoring, badges

**Structure:**
- Définition des missions en YAML/JSON
- Conditions de réussite/échec
- Scripts d'initialisation des environnements
- Système de progression et déverrouillage

### 6. Système d'Évaluation & Logs

Responsable du suivi de la progression:
- Enregistrement des sessions (rosbag)
- Métriques de performance (temps, précision, efficacité)
- Replay des sessions en mode spectateur
- Export des données pour analyse (JSON/rosbag)

## Flux de données

1. L'utilisateur écrit du code Python via l'interface web
2. Le code est envoyé au `picar_api_node` qui l'interprète
3. Les commandes sont traduites en messages ROS pour les actionneurs
4. Gazebo simule la physique et met à jour l'état du robot
5. Les données des capteurs simulés sont publiées sur les topics ROS
6. Le bridge ROS↔Unreal synchronise l'état physique avec le rendu visuel
7. Unreal Engine génère le rendu visuel photo-réaliste
8. Les données sont renvoyées à l'interface utilisateur via WebSocket/REST
9. Les logs et métriques sont enregistrés pour évaluation

## Modes de fonctionnement

### Mode Standard
- Simulation complète avec Gazebo et Unreal Engine
- Tous les capteurs et effets activés
- Performances optimisées pour PC modernes

### Mode Light
- Rendu 3D simplifié ou désactivé
- Fréquence de simulation réduite
- Compatible avec PC modestes

### Mode Développeur
- Outils de débogage avancés
- Visualisation des topics ROS
- Métriques de performance système
- Profiling CPU et mémoire

## Structure des répertoires

```
picar_simulator/
├── ros2_ws/                      # Espace de travail ROS 2
│   ├── src/
│   │   ├── picar_description/    # Description URDF du robot
│   │   ├── picar_gazebo/         # Plugins et mondes Gazebo
│   │   ├── picar_control/        # Contrôleurs et API
│   │   ├── picar_msgs/           # Messages ROS personnalisés
│   │   ├── picar_bringup/        # Launch files
│   │   ├── picar_sensors/        # Simulation des capteurs
│   │   ├── picar_world/          # Gestion de l'environnement
│   │   └── picar_bridge/         # Bridge ROS↔Unreal et UI
│   └── ...
├── unreal_project/               # Projet Unreal Engine
│   ├── Content/
│   │   ├── Maps/                 # Environnements 3D
│   │   ├── PiCar/                # Modèles et matériaux
│   │   ├── Weather/              # Effets météorologiques
│   │   ├── Lighting/             # Configurations d'éclairage
│   │   └── ...
│   └── ...
├── ui/                           # Interface utilisateur
│   ├── frontend/                 # React frontend
│   ├── backend/                  # FastAPI backend
│   └── ...
├── scenarios/                    # Scénarios pédagogiques
│   ├── tutorials/
│   ├── missions/
│   └── challenges/
├── docs/                         # Documentation
├── tests/                        # Tests unitaires et d'intégration
│   ├── unit/
│   ├── integration/
│   └── performance/
├── docker/                       # Configuration Docker
│   ├── Dockerfile
│   ├── docker-compose.yml
│   └── ...
└── scripts/                      # Scripts utilitaires
```

## Étapes de développement

### Phase 1: Fondations
1. Configuration de l'environnement ROS 2 sur PC
2. Modélisation URDF du PiCar X avec paramètres physiques précis
3. Intégration basique Gazebo
4. API Python fondamentale

### Phase 2: Simulation physique avancée
1. Implémentation des capteurs simulés avec bruit et latence
2. Modélisation précise des moteurs et servos (Kv, couple, τ)
3. Simulation de la batterie avec chute de tension
4. Calibration des paramètres physiques
5. Implémentation des aléas pédagogiques

### Phase 3: Intégration Unreal Engine
1. Configuration du bridge ROS↔Unreal
2. Création des environnements dans Unreal Engine
3. Implémentation des effets météorologiques
4. Cycle jour/nuit et éclairage dynamique
5. Optimisation des performances

### Phase 4: Architecture ROS 2 complète
1. Développement des nœuds principaux (controller, sim_picar, sensor_sim, world_manager, ui_bridge)
2. Implémentation des flux de topics
3. Gestion des erreurs et récupération
4. Tests unitaires et d'intégration

### Phase 5: Interface et expérience utilisateur
1. Développement du frontend React
2. Implémentation du backend FastAPI
3. Communication WebSocket/REST
4. Système de missions et progression
5. Gamification (points, badges, classement)

### Phase 6: Finalisation
1. Tests d'intégration complets
2. Optimisation des performances
3. Documentation complète
4. Packaging Docker
5. CI/CD avec GitHub Actions

## Considérations techniques

### Performance
- Profilage CPU < 40% par nœud
- RAM < 1 Go
- Latence end-to-end < 200ms
- RT-preempt ou priorities Linux pour boucles critiques

### Extensibilité
- Architecture modulaire
- API bien documentée
- Possibilité d'ajouter de nouveaux capteurs/actionneurs
- Support pour extensions personnalisées

### Robustesse
- Gestion des erreurs et exceptions
- Stratégie de reprise auto après crash ROS
- Mécanismes de récupération
- Diagnostics système

### Calibration de la caméra
- Utilisation d'OpenCV pour la calibration virtuelle
- Paramètres de distorsion configurables
- Génération de matrices de calibration
- Possibilité de charger des profils de caméra réels