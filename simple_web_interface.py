#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Interface web simplifiée pour le simulateur PiCar X
Version sans ROS pour démarrage rapide
"""

import os
import sys
import time
import threading
import json
import logging
import tempfile
import subprocess
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
import numpy as np

# Configuration du logging
logging.basicConfig(level=logging.INFO, 
                   format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Initialisation de Flask
template_folder = os.path.join('web_interface', 'templates')
app = Flask(__name__, template_folder=template_folder)
app.config['SECRET_KEY'] = 'picar_simulator_secret_key'
socketio = SocketIO(app, cors_allowed_origins="*")

# Données simulées
simulated_data = {
    'ultrasonic': 25.5,
    'ir_left': False,
    'ir_right': False,
    'position': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
    'battery': {'voltage': 7.4, 'percentage': 85.0},
    'imu': {
        'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'linear_acceleration': {'x': 0.0, 'y': 0.0, 'z': 0.0}
    }
}

def get_camera_status():
    """Simuler l'état de la caméra"""
    # Simuler un état de caméra qui change périodiquement
    return (time.time() % 10) < 5  # ON pendant 5s, OFF pendant 5s

def update_simulated_data():
    """Mettre à jour les données simulées"""
    global simulated_data
    
    while True:
        # Simuler des changements de données
        simulated_data['ultrasonic'] = 20 + 10 * np.sin(time.time() * 0.5)
        simulated_data['ir_left'] = (time.time() % 4) < 2
        simulated_data['ir_right'] = (time.time() % 3) < 1.5
        
        # Position qui bouge lentement
        t = time.time() * 0.1
        simulated_data['position']['x'] = 5 * np.cos(t)
        simulated_data['position']['y'] = 5 * np.sin(t)
        simulated_data['position']['theta'] = t
        
        # Batterie qui diminue lentement
        simulated_data['battery']['percentage'] = max(0, 100 - (time.time() % 1000) * 0.1)
        
        # IMU avec du bruit
        simulated_data['imu']['angular_velocity']['z'] = 0.1 * np.random.randn()
        simulated_data['imu']['linear_acceleration']['x'] = 0.5 * np.random.randn()
        
        # Envoyer les données via WebSocket
        socketio.emit('sensor_data', simulated_data)
        
        # Envoyer l'état de la caméra
        camera_active = get_camera_status()
        socketio.emit('camera_status', {'active': camera_active})
        
        time.sleep(0.1)

# Démarrer le thread de simulation
simulation_thread = threading.Thread(target=update_simulated_data)
simulation_thread.daemon = True
simulation_thread.start()

# Routes Flask

@app.route('/')
def index():
    """Page d'accueil"""
    return render_template('index.html')

@app.route('/api/execute_code', methods=['POST'])
def execute_code():
    """Exécuter du code Python"""
    code = request.json.get('code', '')
    
    if not code:
        return jsonify({'success': False, 'output': 'Aucun code fourni'})
    
    # Créer un fichier temporaire pour le code
    with tempfile.NamedTemporaryFile(suffix='.py', delete=False) as temp_file:
        temp_filename = temp_file.name
        temp_file.write(code.encode('utf-8'))
    
    try:
        # Exécuter le code dans un processus séparé
        process = subprocess.Popen(
            [sys.executable, temp_filename],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            env=dict(os.environ, PYTHONIOENCODING='utf-8')  # Force l'encodage UTF-8
        )
        
        # Récupérer la sortie avec un timeout
        stdout, stderr = process.communicate(timeout=30)
        
        if process.returncode == 0:
            output = stdout
            success = True
        else:
            # Vérifier si l'erreur est liée à un module manquant
            if "No module named" in stderr:
                module_name = stderr.split("No module named")[1].split("'")[1]
                output = f"Erreur: Module manquant - '{module_name}'\n\n"
                output += f"Solution: Installez le module avec:\npip install {module_name}\n\n"
                output += "Note: Dans l'environnement web simulé, certains modules peuvent ne pas être disponibles.\n"
                output += "Pour une utilisation complète, exécutez ce code sur le robot réel ou dans un environnement\n"
                output += "avec tous les modules installés."
            else:
                output = f"Erreur:\n{stderr}"
            success = False
        
        return jsonify({'success': success, 'output': output})
    
    except subprocess.TimeoutExpired:
        process.kill()
        return jsonify({
            'success': False, 
            'output': 'Timeout: l\'exécution a pris trop de temps'
        })
    
    except Exception as e:
        return jsonify({
            'success': False, 
            'output': f'Erreur: {str(e)}'
        })
    
    finally:
        # Supprimer le fichier temporaire
        try:
            os.unlink(temp_filename)
        except:
            pass

@app.route('/api/scenarios')
def get_scenarios():
    """Récupérer la liste des scénarios disponibles"""
    scenarios = [
        {
            'id': 'tutorial_1',
            'name': 'Tutoriel 1: Premiers pas',
            'description': 'Apprendre les bases du contrôle du PiCar X',
            'difficulty': 'Débutant',
            'completed': False
        },
        {
            'id': 'demo',
            'name': 'Mode Démo',
            'description': 'Démonstration des fonctionnalités de base',
            'difficulty': 'Débutant',
            'completed': False
        }
    ]
    
    return jsonify(scenarios)

@app.route('/api/scenario/<scenario_id>')
def get_scenario(scenario_id):
    """Récupérer les détails d'un scénario"""
    if scenario_id == 'demo':
        code_template = '''# Mode Démo - PiCar X Simulator
print("=== Bienvenue dans le simulateur PiCar X ===")
print("Ce simulateur fonctionne en mode démo.")
print()
print("Fonctionnalités disponibles:")
print("- Interface web interactive")
print("- Simulation de capteurs")
print("- Exécution de code Python")
print("- Visualisation en temps réel")
print()
print("Pour utiliser le simulateur complet avec ROS 2:")
print("1. Installez ROS 2 Humble")
print("2. Utilisez Docker avec: docker-compose up ros_gazebo")
print()
print("Données des capteurs simulées:")
print(f"- Capteur ultrason: {25.5:.1f} cm")
print(f"- Capteur IR gauche: {'Détecté' if False else 'Non détecté'}")
print(f"- Capteur IR droit: {'Détecté' if False else 'Non détecté'}")
print(f"- Batterie: {85.0:.1f}%")
'''
        
        return jsonify({
            'id': 'demo',
            'name': 'Mode Démo',
            'description': 'Démonstration des fonctionnalités de base',
            'instructions': [
                'Explorez l\'interface web',
                'Observez les données des capteurs',
                'Testez l\'exécution de code',
                'Visualisez le flux vidéo simulé'
            ],
            'code_template': code_template,
            'difficulty': 'Débutant',
            'estimated_time': '5 minutes',
            'completed': False
        })
    
    return jsonify({'error': 'Scénario non trouvé'}), 404

# Événements WebSocket

@socketio.on('connect')
def handle_connect():
    """Gestion de la connexion WebSocket"""
    logger.info("Client connecté")
    # Envoyer immédiatement les données actuelles
    emit('sensor_data', simulated_data)

@socketio.on('disconnect')
def handle_disconnect():
    """Gestion de la déconnexion WebSocket"""
    logger.info("Client déconnecté")

# Point d'entrée principal

def main():
    """Fonction principale"""
    print("=== PiCar X Simulator - Mode Démo ===")
    print("Interface web disponible sur: http://localhost:8080")
    print("Appuyez sur Ctrl+C pour arrêter")
    print()
    
    try:
        # Démarrer le serveur Flask
        socketio.run(app, host='0.0.0.0', port=8080, debug=False)
    
    except KeyboardInterrupt:
        logger.info("Arrêt demandé par l'utilisateur")

if __name__ == '__main__':
    main()