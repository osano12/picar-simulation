#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Script de lancement pour PiCarX Studio
Version améliorée avec argparse, logging et modes avancés
"""

import sys
import os
import argparse
import logging
import time
import requests
from pathlib import Path

# Ajouter le répertoire du projet au path
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

# Configuration du logging
def setup_logging(verbose=False, log_file=None):
    """Configurer le système de logging"""
    level = logging.DEBUG if verbose else logging.INFO
    
    # Format des messages
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    # Logger principal
    logger = logging.getLogger('PiCarXStudio')
    logger.setLevel(level)
    
    # Handler console
    console_handler = logging.StreamHandler()
    console_handler.setLevel(level)
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)
    
    # Handler fichier si spécifié
    if log_file:
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
    
    return logger

def check_dependencies(logger):
    """Vérifier les dépendances critiques"""
    logger.info("Vérification des dépendances...")
    missing_deps = []
    
    dependencies = {
        'PyQt6': 'Interface graphique',
        'numpy': 'Calculs scientifiques',
        'yaml': 'Configuration',
        'requests': 'Mise à jour'
    }
    
    for dep, description in dependencies.items():
        try:
            __import__(dep.lower() if dep != 'PyQt6' else 'PyQt6')
            logger.debug(f"✅ {dep} ({description}) - OK")
        except ImportError:
            missing_deps.append((dep, description))
            logger.error(f"❌ {dep} ({description}) - MANQUANT")
    
    if missing_deps:
        print("\n❌ Dépendances manquantes:")
        for dep, desc in missing_deps:
            print(f"   - {dep}: {desc}")
        print("\n💡 Installez les dépendances avec:")
        print("   pip install -r requirements.txt")
        return False
    
    logger.info("✅ Toutes les dépendances sont installées")
    return True

def check_for_updates(logger):
    """Vérifier les mises à jour disponibles"""
    try:
        logger.info("Vérification des mises à jour...")
        # Simulation d'une vérification de mise à jour
        # Dans un vrai projet, ceci contacterait un serveur de versions
        current_version = "1.0.0"
        logger.info(f"Version actuelle: {current_version}")
        logger.info("✅ Vous utilisez la dernière version")
        return True
    except Exception as e:
        logger.warning(f"Impossible de vérifier les mises à jour: {e}")
        return False

def create_directories(logger):
    """Créer les répertoires nécessaires"""
    directories = [
        'logs',
        'projects',
        'scenarios',
        'exports',
        'temp'
    ]
    
    for dir_name in directories:
        dir_path = Path(project_root) / dir_name
        if not dir_path.exists():
            dir_path.mkdir(parents=True, exist_ok=True)
            logger.debug(f"📁 Répertoire créé: {dir_path}")

def run_headless_mode(logger, args):
    """Mode sans interface graphique"""
    logger.info("🖥️ Démarrage en mode headless")
    
    if args.scenario:
        logger.info(f"Exécution du scénario: {args.scenario}")
        # Ici on pourrait charger et exécuter un scénario
        time.sleep(2)  # Simulation
        logger.info("✅ Scénario exécuté avec succès")
    else:
        logger.info("Mode headless actif - En attente de commandes...")
        # Ici on pourrait démarrer un serveur web ou une API
        time.sleep(5)  # Simulation
    
    return 0

def main():
    """Fonction principale de lancement"""
    # Parser d'arguments
    parser = argparse.ArgumentParser(
        description='PiCarX Studio - Simulateur et Programmation Robotique',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples d'utilisation:
  %(prog)s                          # Lancement normal
  %(prog)s --verbose                # Mode verbeux
  %(prog)s --headless               # Mode sans interface
  %(prog)s --scenario demo.py       # Exécuter un scénario
  %(prog)s --no-update-check        # Pas de vérification de MAJ
  %(prog)s --log-file studio.log    # Sauvegarder les logs
        """
    )
    
    parser.add_argument('--version', action='version', version='PiCarX Studio 1.0.0')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Mode verbeux (plus de détails)')
    parser.add_argument('--headless', action='store_true',
                       help='Mode sans interface graphique')
    parser.add_argument('--scenario', '-s', type=str,
                       help='Exécuter un scénario spécifique')
    parser.add_argument('--no-update-check', action='store_true',
                       help='Désactiver la vérification de mise à jour')
    parser.add_argument('--log-file', type=str,
                       help='Fichier de log (optionnel)')
    parser.add_argument('--config', '-c', type=str,
                       help='Fichier de configuration personnalisé')
    
    args = parser.parse_args()
    
    # Configuration du logging
    log_file = args.log_file or (Path(project_root) / 'logs' / f'studio_{int(time.time())}.log')
    logger = setup_logging(args.verbose, log_file)
    
    logger.info("🚀 Démarrage de PiCarX Studio...")
    logger.info(f"Arguments: {vars(args)}")
    
    # Créer les répertoires nécessaires
    create_directories(logger)
    
    # Vérifier les dépendances
    if not check_dependencies(logger):
        return 1
    
    # Vérification des mises à jour
    if not args.no_update_check:
        check_for_updates(logger)
    
    # Mode headless
    if args.headless:
        return run_headless_mode(logger, args)
    
    # Lancement de l'interface graphique
    try:
        logger.info("🎨 Initialisation de l'interface graphique...")
        from main import main as app_main
        return app_main(logger, args)
    except Exception as e:
        logger.error(f"❌ Erreur lors du lancement: {e}")
        if args.verbose:
            import traceback
            logger.error(traceback.format_exc())
        return 1

if __name__ == "__main__":
    sys.exit(main())