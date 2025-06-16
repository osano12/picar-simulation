# PiCarX Studio

**Simulateur et Environnement de Programmation pour Robot PiCarX**

PiCarX Studio est une application desktop moderne qui permet de simuler, programmer et contrôler le robot PiCarX dans un environnement 2D interactif.

## 🌟 Fonctionnalités

### 🎮 Simulation Interactive
- **Simulation 2D en temps réel** avec physique réaliste
- **Capteurs simulés** : ultrasonique, infrarouges, IMU, batterie
- **Environnement configurable** avec obstacles et lignes à suivre
- **Visualisation des données** capteurs en temps réel

### 🐍 Programmation Python
- **Éditeur de code intégré** avec coloration syntaxique
- **Exemples prêts à l'emploi** pour démarrer rapidement
- **Exécution de code** en temps réel avec console de sortie
- **Bibliothèque d'algorithmes** robotiques

### 🎯 Scénarios d'Apprentissage
- **Progression guidée** du débutant à l'expert
- **Défis et missions** avec objectifs clairs
- **Système de badges** et suivi de progression
- **Tutoriels interactifs** étape par étape

### 🎛️ Interface Moderne
- **Interface sombre** optimisée pour le développement
- **Panneaux modulaires** et redimensionnables
- **Contrôles intuitifs** clavier et souris
- **Thème professionnel** avec icônes

## 🚀 Installation

### Prérequis
- Python 3.8 ou supérieur
- Système d'exploitation : Windows, macOS, ou Linux

### Installation rapide

1. **Cloner le projet** (si applicable) ou naviguer vers le dossier :
   ```bash
   cd /home/osano/Documents/PICARX/picarx_studio
   ```

2. **Installer les dépendances** :
   ```bash
   pip install -r requirements.txt
   ```

3. **Lancer l'application** :
   ```bash
   python launch.py
   ```

### Installation des dépendances individuelles

Si vous préférez installer manuellement :
```bash
pip install PyQt6 numpy flask flask-socketio
```

## 🎮 Utilisation

### Démarrage Rapide

1. **Lancez PiCarX Studio** avec `python launch.py`
2. **Explorez l'interface** :
   - **Panneau gauche** : Contrôles et capteurs
   - **Centre** : Simulateur 2D et éditeur de code
   - **Panneau droit** : Scénarios et missions

3. **Premier test** :
   - Cliquez sur "▶️ Démarrer" pour activer la simulation
   - Utilisez les flèches du clavier ou les boutons pour contrôler le robot
   - Observez les données des capteurs en temps réel

### Programmation

1. **Sélectionnez un exemple** dans l'éditeur de code
2. **Modifiez le code** selon vos besoins
3. **Exécutez** avec Ctrl+F5 ou le bouton "🐍 Exécuter"
4. **Observez les résultats** dans la console

### Scénarios d'Apprentissage

1. **Choisissez un scénario** dans le panneau de droite
2. **Lisez les objectifs** et la description
3. **Chargez le code template** avec "📂 Charger"
4. **Complétez la mission** en programmant le robot

## 🎯 Scénarios Disponibles

### 🟢 Débutant
- **🚀 Introduction aux bases** - Commandes de mouvement
- **📡 Lecture des capteurs** - Comprendre les données

### 🟡 Intermédiaire  
- **🛤️ Suivi de ligne** - Algorithme de suivi avec capteurs IR
- **🚧 Évitement d'obstacles** - Navigation intelligente

### 🔴 Avancé
- **🌀 Navigation en labyrinthe** - Algorithmes de pathfinding
- **🏁 Défis de vitesse** - Optimisation de performance

## 🎛️ Contrôles

### Clavier
- **Flèches directionnelles** : Déplacer le robot
- **Espace** : Arrêter le robot
- **F5** : Démarrer/Arrêter la simulation
- **Ctrl+F5** : Exécuter le code
- **Ctrl+S** : Sauvegarder le projet

### Interface
- **Boutons directionnels** : Contrôle manuel
- **Sliders** : Ajuster vitesse et paramètres
- **Checkboxes** : Affichage des éléments (capteurs, trajectoire, grille)

## 🔧 Configuration

### Paramètres de Simulation
- **Vitesse du robot** : 10-100%
- **Sensibilité des capteurs** : Ajustable
- **Qualité de rendu** : Optimisable selon le matériel

### Personnalisation
- **Thème d'interface** : Sombre (par défaut)
- **Taille de police** : Configurable
- **Raccourcis clavier** : Personnalisables

## 📁 Structure du Projet

```
picarx_studio/
├── main.py                 # Point d'entrée principal
├── launch.py              # Script de lancement
├── requirements.txt       # Dépendances Python
├── core/                  # Modules de base
│   ├── config.py         # Gestion de la configuration
│   └── logger.py         # Système de logging
├── ui/                    # Interface utilisateur
│   ├── main_window.py    # Fenêtre principale
│   ├── simulator_widget.py # Widget de simulation
│   ├── control_panel.py  # Panneau de contrôle
│   ├── sensor_panel.py   # Affichage des capteurs
│   ├── code_editor.py    # Éditeur de code Python
│   └── scenario_panel.py # Gestion des scénarios
└── README.md             # Documentation
```

## 🐛 Résolution de Problèmes

### Erreurs Communes

**"No module named 'PyQt6'"**
```bash
pip install PyQt6
```

**"Application ne démarre pas"**
- Vérifiez la version de Python (3.8+)
- Réinstallez les dépendances
- Consultez les logs dans `~/.picarx_studio/logs/`

**"Simulation lente"**
- Réduisez la qualité de rendu dans les paramètres
- Fermez les autres applications
- Désactivez l'affichage de la trajectoire

### Support

Pour obtenir de l'aide :
1. Consultez les logs dans `~/.picarx_studio/logs/`
2. Vérifiez la configuration dans `~/.picarx_studio/config.json`
3. Redémarrez l'application

## 🚀 Développement Futur

### Fonctionnalités Prévues
- **🌐 Connexion au robot réel** via WiFi/Bluetooth
- **🎨 Éditeur visuel par blocs** (type Scratch)
- **🏆 Système de classement** en ligne
- **📹 Enregistrement de sessions** pour analyse
- **🤖 IA et apprentissage automatique** intégrés

### Contributions
Les contributions sont les bienvenues ! Consultez le fichier CONTRIBUTING.md pour plus d'informations.

## 📄 Licence

Ce projet est sous licence MIT. Voir le fichier LICENSE pour plus de détails.

## 👥 Équipe

Développé par l'équipe PiCarX pour l'apprentissage de la robotique et de la programmation.

---

**🎉 Amusez-vous bien avec PiCarX Studio !**