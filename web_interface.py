#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Compatibilité multiplateforme
import os
import sys
import platform

"""
Interface web pour le simulateur PiCar X

Cette application Flask fournit une interface web pour:
- Visualiser le flux vidéo de la caméra
- Écrire et exécuter du code Python
- Voir les données des capteurs en temps réel
- Suivre la progression des missions
"""

import os
import sys
import time
import threading
import json
import base64
import logging
from functools import wraps
import traceback
import tempfile
import subprocess
import rclpy
from rclpy.node import Node
from flask import Flask, render_template, request, jsonify, Response
from flask_socketio import SocketIO, emit
import cv2
import numpy as np
from cv_bridge import CvBridge

# Messages ROS
from sensor_msgs.msg import Image, Range, Imu
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState

# Configuration du logging
logging.basicConfig(level=logging.INFO, 
                   format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Initialisation de Flask
# Utiliser des chemins compatibles avec tous les systèmes d'exploitation
static_folder = os.path.join('web_interface', 'static')
template_folder = os.path.join('web_interface', 'templates')
app = Flask(__name__, 
           static_folder=static_folder, 
           template_folder=template_folder)
app.config['SECRET_KEY'] = 'picar_simulator_secret_key'
socketio = SocketIO(app, cors_allowed_origins="*")

# Classe pour l'interface ROS
class WebInterfaceNode(Node):
    def __init__(self):
        super().__init__('web_interface_node')
        
        # Bridge pour convertir les images ROS en OpenCV
        self.cv_bridge = CvBridge()
        
        # Dernières données reçues
        self.latest_data = {
            'camera_image': None,
            'ultrasonic': float('inf'),
            'ir_left': False,
            'ir_right': False,
            'position': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'battery': {'voltage': 7.4, 'percentage': 100.0},
            'imu': {'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                   'linear_acceleration': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
        }
        
        # Verrou pour l'accès thread-safe aux données
        self.data_lock = threading.RLock()
        
        # Abonnements aux topics ROS
        self.camera_sub = self.create_subscription(
            Image, 'picar/camera/image_raw', self.camera_callback, 10)
        
        self.ultrasonic_sub = self.create_subscription(
            Range, 'picar/sensors/ultrasonic', self.ultrasonic_callback, 10)
        
        self.ir_left_sub = self.create_subscription(
            Range, 'picar/sensors/ir_left', self.ir_left_callback, 10)
        
        self.ir_right_sub = self.create_subscription(
            Range, 'picar/sensors/ir_right', self.ir_right_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, 'picar/odom', self.odom_callback, 10)
        
        self.battery_sub = self.create_subscription(
            BatteryState, 'picar/power/battery', self.battery_callback, 10)
        
        self.imu_sub = self.create_subscription(
            Imu, 'picar/sensors/imu', self.imu_callback, 10)
        
        # Timer pour envoyer les données via WebSocket
        self.timer = self.create_timer(0.1, self.send_data_callback)
        
        self.get_logger().info("Interface web ROS initialisée")
    
    def camera_callback(self, msg):
        """Callback pour le flux de la caméra"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Convertir l'image en JPEG pour le streaming
            _, jpeg_image = cv2.imencode('.jpg', cv_image)
            encoded_image = base64.b64encode(jpeg_image).decode('utf-8')
            
            with self.data_lock:
                self.latest_data['camera_image'] = encoded_image
        except Exception as e:
            self.get_logger().error(f"Erreur lors du traitement de l'image: {e}")
    
    def ultrasonic_callback(self, msg):
        """Callback pour le capteur ultrason"""
        with self.data_lock:
            self.latest_data['ultrasonic'] = msg.range
    
    def ir_left_callback(self, msg):
        """Callback pour le capteur IR gauche"""
        with self.data_lock:
            # Considérer que la ligne est détectée si la distance est inférieure à 5 cm
            self.latest_data['ir_left'] = msg.range < 0.05
    
    def ir_right_callback(self, msg):
        """Callback pour le capteur IR droit"""
        with self.data_lock:
            # Considérer que la ligne est détectée si la distance est inférieure à 5 cm
            self.latest_data['ir_right'] = msg.range < 0.05
    
    def odom_callback(self, msg):
        """Callback pour l'odométrie"""
        with self.data_lock:
            # Position
            self.latest_data['position']['x'] = msg.pose.pose.position.x
            self.latest_data['position']['y'] = msg.pose.pose.position.y
            
            # Orientation (conversion quaternion -> euler)
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            
            # Formule de conversion quaternion -> angle yaw
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            theta = np.arctan2(siny_cosp, cosy_cosp)
            
            self.latest_data['position']['theta'] = theta
    
    def battery_callback(self, msg):
        """Callback pour l'état de la batterie"""
        with self.data_lock:
            self.latest_data['battery']['voltage'] = msg.voltage
            self.latest_data['battery']['percentage'] = msg.percentage
    
    def imu_callback(self, msg):
        """Callback pour l'IMU"""
        with self.data_lock:
            # Vitesse angulaire
            self.latest_data['imu']['angular_velocity']['x'] = msg.angular_velocity.x
            self.latest_data['imu']['angular_velocity']['y'] = msg.angular_velocity.y
            self.latest_data['imu']['angular_velocity']['z'] = msg.angular_velocity.z
            
            # Accélération linéaire
            self.latest_data['imu']['linear_acceleration']['x'] = msg.linear_acceleration.x
            self.latest_data['imu']['linear_acceleration']['y'] = msg.linear_acceleration.y
            self.latest_data['imu']['linear_acceleration']['z'] = msg.linear_acceleration.z
    
    def send_data_callback(self):
        """Envoyer les données via WebSocket"""
        try:
            with self.data_lock:
                # Copier les données pour éviter les modifications pendant l'envoi
                data_to_send = {
                    'ultrasonic': self.latest_data['ultrasonic'],
                    'ir_left': self.latest_data['ir_left'],
                    'ir_right': self.latest_data['ir_right'],
                    'position': self.latest_data['position'],
                    'battery': self.latest_data['battery'],
                    'imu': self.latest_data['imu']
                }
                
                # Envoyer l'image séparément pour réduire la charge
                if self.latest_data['camera_image']:
                    socketio.emit('camera_frame', {'image': self.latest_data['camera_image']})
            
            # Envoyer les données des capteurs
            socketio.emit('sensor_data', data_to_send)
        except Exception as e:
            self.get_logger().error(f"Erreur lors de l'envoi des données: {e}")


# Instance globale du nœud ROS
ros_node = None
ros_thread = None

def start_ros_node():
    """Démarrer le nœud ROS dans un thread séparé"""
    global ros_node, ros_thread
    
    def run_ros():
        global ros_node
        rclpy.init()
        ros_node = WebInterfaceNode()
        rclpy.spin(ros_node)
    
    ros_thread = threading.Thread(target=run_ros)
    ros_thread.daemon = True
    ros_thread.start()
    logger.info("Nœud ROS démarré dans un thread séparé")

def stop_ros_node():
    """Arrêter le nœud ROS"""
    global ros_node
    if ros_node:
        ros_node.destroy_node()
        rclpy.shutdown()
        logger.info("Nœud ROS arrêté")

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
            text=True
        )
        
        # Récupérer la sortie avec un timeout
        stdout, stderr = process.communicate(timeout=30)
        
        if process.returncode == 0:
            output = stdout
            success = True
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
            'output': f'Erreur: {str(e)}\n{traceback.format_exc()}'
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
            'id': 'tutorial_2',
            'name': 'Tutoriel 2: Suivi de ligne',
            'description': 'Apprendre à suivre une ligne avec les capteurs IR',
            'difficulty': 'Débutant',
            'completed': False
        },
        {
            'id': 'challenge_1',
            'name': 'Défi 1: Slalom',
            'description': 'Naviguer entre les cônes sans les toucher',
            'difficulty': 'Intermédiaire',
            'completed': False
        },
        {
            'id': 'challenge_2',
            'name': 'Défi 2: Labyrinthe',
            'description': 'Trouver la sortie du labyrinthe',
            'difficulty': 'Avancé',
            'completed': False
        }
    ]
    
    return jsonify(scenarios)

@app.route('/api/scenario/<scenario_id>')
def get_scenario(scenario_id):
    """Récupérer les détails d'un scénario"""
    # Exemple pour tutorial_1
    if scenario_id == 'tutorial_1':
        tutorial_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tutorial_1.py')
        with open(tutorial_path, 'r', encoding='utf-8') as f:
            code = f.read()
        
        return jsonify({
            'id': 'tutorial_1',
            'name': 'Tutoriel 1: Premiers pas',
            'description': 'Apprendre les bases du contrôle du PiCar X',
            'instructions': [
                'Initialiser le robot',
                'Avancer de 50 cm',
                'Tourner de 90 degrés',
                'Orienter la caméra',
                'Configurer des événements',
                'Détecter un obstacle'
            ],
            'code_template': code,
            'difficulty': 'Débutant',
            'estimated_time': '10 minutes',
            'completed': False
        })
    
    return jsonify({'error': 'Scénario non trouvé'}), 404

# Événements WebSocket

@socketio.on('connect')
def handle_connect():
    """Gestion de la connexion WebSocket"""
    logger.info("Client connecté")

@socketio.on('disconnect')
def handle_disconnect():
    """Gestion de la déconnexion WebSocket"""
    logger.info("Client déconnecté")

# Point d'entrée principal

def main():
    """Fonction principale"""
    try:
        # Démarrer le nœud ROS
        start_ros_node()
        
        # Démarrer le serveur Flask
        socketio.run(app, host='0.0.0.0', port=8080, debug=True, use_reloader=False)
    
    except KeyboardInterrupt:
        logger.info("Arrêt demandé par l'utilisateur")
    
    finally:
        # Arrêter le nœud ROS
        stop_ros_node()


if __name__ == '__main__':
    main()