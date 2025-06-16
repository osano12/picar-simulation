#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Script de lancement pour PiCarX Studio
"""

import sys
import os

# Ajouter le répertoire du projet au path
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

def check_dependencies():
    """Vérifier les dépendances critiques"""
    missing_deps = []
    
    try:
        import PyQt6
    except ImportError:
        missing_deps.append("PyQt6")
    
    try:
        import numpy
    except ImportError:
        missing_deps.append("numpy")
    
    if missing_deps:
        print("❌ Dépendances manquantes:")
        for dep in missing_deps:
            print(f"   - {dep}")
        print("\n💡 Installez les dépendances avec:")
        print("   pip install -r requirements.txt")
        return False
    
    return True

def main():
    """Fonction principale de lancement"""
    print("🚀 Lancement de PiCarX Studio...")
    
    # Vérifier les dépendances
    if not check_dependencies():
        return 1
    
    # Importer et lancer l'application
    try:
        from main import main as app_main
        return app_main()
    except Exception as e:
        print(f"❌ Erreur lors du lancement: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())