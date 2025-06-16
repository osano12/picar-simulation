#!/bin/bash

# Script de lancement pour le mode démo du simulateur PiCar X

echo "=== Lancement du PiCar X Simulator - Mode Démo ==="
echo

# Vérifier si nous sommes dans le bon répertoire
if [ ! -f "simple_web_interface.py" ]; then
    echo "Erreur: Veuillez exécuter ce script depuis le répertoire du projet"
    exit 1
fi

# Activer l'environnement virtuel
if [ ! -d "venv" ]; then
    echo "Création de l'environnement virtuel..."
    python3 -m venv venv
    source venv/bin/activate
    pip install flask flask-socketio opencv-python numpy
else
    source venv/bin/activate
fi

echo "Démarrage du simulateur en mode démo..."
echo "Interface web disponible sur:"
echo "  - Local: http://localhost:8080"
echo "  - Réseau: http://$(hostname -I | awk '{print $1}'):8080"
echo
echo "Appuyez sur Ctrl+C pour arrêter"
echo

# Lancer l'interface web
python simple_web_interface.py