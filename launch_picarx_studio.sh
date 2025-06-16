#!/bin/bash

# Script de lancement pour PiCarX Studio
# Application desktop de simulation et programmation robotique

echo "🚀 Lancement de PiCarX Studio..."
echo "Application desktop de simulation et programmation robotique"
echo

# Vérifier si nous sommes dans le bon répertoire
if [ ! -d "picarx_studio" ]; then
    echo "❌ Erreur: Répertoire picarx_studio non trouvé"
    echo "Veuillez exécuter ce script depuis le répertoire racine du projet"
    exit 1
fi

# Aller dans le répertoire picarx_studio
cd picarx_studio

# Vérifier si l'environnement virtuel existe
if [ ! -d "../venv" ]; then
    echo "Création de l'environnement virtuel..."
    cd ..
    python3 -m venv venv
    source venv/bin/activate
    cd picarx_studio
    pip install -r requirements.txt
else
    # Activer l'environnement virtuel
    source ../venv/bin/activate
fi

echo "✅ Environnement virtuel activé"
echo "🎮 Lancement de PiCarX Studio..."
echo

# Lancer l'application
python3 launch.py