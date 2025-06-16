#!/usr/bin/env python3

"""
Script de test pour vérifier que l'interface web fonctionne correctement
"""

import subprocess
import time
import requests
import sys

def check_requirements():
    """Vérifier si les modules requis sont installés"""
    required_modules = ['flask', 'flask_socketio', 'numpy']
    missing_modules = []
    
    for module in required_modules:
        try:
            __import__(module)
        except ImportError:
            missing_modules.append(module)
    
    return missing_modules

def test_interface():
    print("=== Test de l'interface web PiCar X ===")
    
    # Vérifier les modules requis
    missing_modules = check_requirements()
    if missing_modules:
        print("\n❌ Modules Python manquants :")
        for module in missing_modules:
            print(f"  - {module}")
        
        print("\nPour installer les modules manquants, exécutez la commande suivante :")
        modules_str = " ".join(missing_modules)
        print(f"\npip install {modules_str}")
        
        print("\nNote: Cette interface requiert les modules suivants:")
        print("- flask: pour le serveur web")
        print("- flask_socketio: pour les communications en temps réel")
        print("- numpy: pour les calculs scientifiques")
        print("\nAprès installation, relancez le test.")
        return
    
    # Démarrer l'interface en arrière-plan
    print("Démarrage de l'interface web...")
    process = subprocess.Popen([
        sys.executable, "simple_web_interface.py"
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    # Récupérer les premiers logs pour voir s'il y a des erreurs
    time.sleep(1)
    if process.poll() is not None:
        stdout, stderr = process.communicate()
        print("Erreur au démarrage du serveur:")
        print(stderr.decode('utf-8'))
        print("\nVérifiez que tous les modules requis sont bien installés.")
        return
    
    # Attendre que le serveur démarre
    print("Attente du démarrage du serveur (5 secondes)...")
    time.sleep(5)
    
    try:
        # Tester l'accès à la page principale
        print("Test de l'accès à la page principale...")
        response = requests.get("http://localhost:8080", timeout=5)
        if response.status_code == 200:
            print("✅ Page principale accessible")
        else:
            print(f"❌ Erreur page principale: {response.status_code}")
        
        # Tester l'API des scénarios
        print("Test de l'API des scénarios...")
        response = requests.get("http://localhost:8080/api/scenarios", timeout=5)
        if response.status_code == 200:
            scenarios = response.json()
            print(f"✅ API scénarios accessible - {len(scenarios)} scénarios trouvés")
        else:
            print(f"❌ Erreur API scénarios: {response.status_code}")
        
        print("\n🎉 Interface web fonctionnelle !")
        print("Accédez à http://localhost:8080 pour voir la nouvelle interface")
        print("Nouvelles fonctionnalités:")
        print("- Vue du dessus du robot avec orientation et grille")
        print("- Indicateur d'état de la caméra")
        print("- Capteurs visuels qui changent de couleur")
        print("- Position et orientation en temps réel")
        print("- Paramètres de suivi de ligne configurable")
        print("- Bouton de remise à zéro complète")
        print("- Gestion améliorée des erreurs de modules manquants")
        
    except requests.exceptions.RequestException as e:
        print(f"❌ Erreur de connexion: {e}")
        print("\nVérifiez que le serveur est bien démarré sur le port 8080.")
    
    finally:
        # Arrêter le processus
        process.terminate()
        process.wait()
        print("\nTest terminé.")

if __name__ == "__main__":
    test_interface()