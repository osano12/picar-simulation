#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PiCar X API - Interface Python pour contrôler le robot PiCar X simulé

Cette API fournit une interface simple et intuitive pour contrôler le robot
PiCar X dans l'environnement de simulation. Elle est conçue pour être
pédagogique et facile à utiliser pour les débutants.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
import time
import math
import numpy as np
from functools import partial

# Messages ROS
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Range, Image, Imu
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState

# Traitement d'image
try:
    import cv2
    from cv_bridge import CvBridge
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


class PiCar(Node):
    """
    Classe principale pour contrôler le PiCar X
    """
    
    def __init__(self, name="picar_controller"):
        """
        Initialise le contrôleur PiCar
        
        Args:
            name (str): Nom du nœud ROS
        """
        super().__init__(name)
        
        # Groupe de callbacks pour permettre l'exécution concurrente
        self.callback_group = ReentrantCallbackGroup()
        
        # Paramètres du robot
        self.wheel_radius = 0.03  # 3 cm
        self.wheel_separation = 0.13  # 13 cm
        self.max_linear_speed = 0.5  # 50 cm/s
        self.max_angular_speed = 1.5  # ~90 deg/s
        
        # État du robot
        self.current_position = (0.0, 0.0)  # (x, y) en mètres
        self.current_orientation = 0.0  # en radians
        self.current_linear_speed = 0.0  # en m/s
        self.current_angular_speed = 0.0  # en rad/s
        self.battery_voltage = 7.4  # en volts
        self.battery_percentage = 100.0  # en pourcentage
        
        # État des capteurs
        self.ultrasonic_distance = float('inf')  # en mètres
        self.ir_left_detected = False
        self.ir_right_detected = False
        self.camera_image = None
        self.imu_data = None
        
        # Callbacks enregistrés par l'utilisateur
        self.callbacks = {
            'ultrasonic_close': [],
            'line_detected': [],
            'line_lost': [],
            'battery_low': [],
            'collision': []
        }
        
        # Seuils pour les événements
        self.thresholds = {
            'ultrasonic_close': 0.2,  # 20 cm
            'battery_low': 6.5,  # 6.5V
            'line_detection': 0.05  # 5 cm
        }
        
        # Publishers
        self.vel_publisher = self.create_publisher(
            Twist, 'picar/cmd_vel', 10)
        
        self.steering_publisher = self.create_publisher(
            JointTrajectory, 'picar/steering_cmd', 10)
        
        self.camera_pan_publisher = self.create_publisher(
            JointTrajectory, 'picar/camera_pan_cmd', 10)
        
        self.camera_tilt_publisher = self.create_publisher(
            JointTrajectory, 'picar/camera_tilt_cmd', 10)
        
        # Subscribers
        self.ultrasonic_subscriber = self.create_subscription(
            Range, 'picar/sensors/ultrasonic', 
            self._ultrasonic_callback, 10,
            callback_group=self.callback_group)
        
        self.ir_left_subscriber = self.create_subscription(
            Range, 'picar/sensors/ir_left', 
            self._ir_left_callback, 10,
            callback_group=self.callback_group)
        
        self.ir_right_subscriber = self.create_subscription(
            Range, 'picar/sensors/ir_right', 
            self._ir_right_callback, 10,
            callback_group=self.callback_group)
        
        self.odom_subscriber = self.create_subscription(
            Odometry, 'picar/odom', 
            self._odom_callback, 10,
            callback_group=self.callback_group)
        
        self.battery_subscriber = self.create_subscription(
            BatteryState, 'picar/power/battery', 
            self._battery_callback, 10,
            callback_group=self.callback_group)
        
        self.imu_subscriber = self.create_subscription(
            Imu, 'picar/sensors/imu', 
            self._imu_callback, 10,
            callback_group=self.callback_group)
        
        # Subscriber pour la caméra (si OpenCV est disponible)
        if CV_AVAILABLE:
            self.cv_bridge = CvBridge()
            self.camera_subscriber = self.create_subscription(
                Image, 'picar/camera/image_raw', 
                self._camera_callback, 10,
                callback_group=self.callback_group)
        
        # Timer pour vérifier les événements
        self.event_timer = self.create_timer(
            0.1, self._check_events, callback_group=self.callback_group)
        
        # Verrou pour les opérations thread-safe
        self._lock = threading.RLock()
        
        self.get_logger().info("PiCar X API initialisée")
    
    # ===== Méthodes de contrôle de base =====
    
    def set_speed(self, linear_speed, angular_speed=0.0):
        """
        Définit la vitesse du robot
        
        Args:
            linear_speed (float): Vitesse linéaire en cm/s
            angular_speed (float): Vitesse angulaire en deg/s
        """
        # Conversion des unités
        linear_speed_m = linear_speed / 100.0  # cm/s -> m/s
        angular_speed_rad = math.radians(angular_speed)  # deg/s -> rad/s
        
        # Limitation des vitesses
        linear_speed_m = max(-self.max_linear_speed, 
                           min(self.max_linear_speed, linear_speed_m))
        angular_speed_rad = max(-self.max_angular_speed, 
                              min(self.max_angular_speed, angular_speed_rad))
        
        # Création et envoi du message
        twist = Twist()
        twist.linear.x = linear_speed_m
        twist.angular.z = angular_speed_rad
        self.vel_publisher.publish(twist)
        
        # Mise à jour de l'état
        with self._lock:
            self.current_linear_speed = linear_speed_m
            self.current_angular_speed = angular_speed_rad
    
    def stop(self):
        """Arrête le robot"""
        self.set_speed(0.0, 0.0)
    
    def set_steering(self, angle):
        """
        Définit l'angle de direction
        
        Args:
            angle (float): Angle de direction en degrés (-30 à 30)
        """
        # Limitation de l'angle
        angle = max(-30.0, min(30.0, angle))
        
        # Conversion en radians
        angle_rad = math.radians(angle)
        
        # Création du message de trajectoire
        trajectory = JointTrajectory()
        trajectory.joint_names = ['steering_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [angle_rad]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 500000000  # 0.5 secondes
        
        trajectory.points = [point]
        
        # Envoi du message
        self.steering_publisher.publish(trajectory)
    
    def set_camera_position(self, pan=0.0, tilt=0.0):
        """
        Définit la position de la caméra
        
        Args:
            pan (float): Angle horizontal en degrés (-45 à 45)
            tilt (float): Angle vertical en degrés (-20 à 20)
        """
        # Limitation des angles
        pan = max(-45.0, min(45.0, pan))
        tilt = max(-20.0, min(20.0, tilt))
        
        # Conversion en radians
        pan_rad = math.radians(pan)
        tilt_rad = math.radians(tilt)
        
        # Création et envoi des messages de trajectoire
        pan_trajectory = JointTrajectory()
        pan_trajectory.joint_names = ['camera_pan_joint']
        
        pan_point = JointTrajectoryPoint()
        pan_point.positions = [pan_rad]
        pan_point.time_from_start.sec = 0
        pan_point.time_from_start.nanosec = 500000000  # 0.5 secondes
        
        pan_trajectory.points = [pan_point]
        
        tilt_trajectory = JointTrajectory()
        tilt_trajectory.joint_names = ['camera_tilt_joint']
        
        tilt_point = JointTrajectoryPoint()
        tilt_point.positions = [tilt_rad]
        tilt_point.time_from_start.sec = 0
        tilt_point.time_from_start.nanosec = 500000000  # 0.5 secondes
        
        tilt_trajectory.points = [tilt_point]
        
        # Envoi des messages
        self.camera_pan_publisher.publish(pan_trajectory)
        self.camera_tilt_publisher.publish(tilt_trajectory)
    
    # ===== Méthodes de haut niveau =====
    
    def drive(self, distance_cm, speed=20.0):
        """
        Avance le robot d'une distance spécifiée
        
        Args:
            distance_cm (float): Distance à parcourir en cm
            speed (float): Vitesse en cm/s
        """
        # Déterminer la direction
        direction = 1.0 if distance_cm >= 0 else -1.0
        distance_cm = abs(distance_cm)
        speed = abs(speed)
        
        # Enregistrer la position de départ
        start_x, start_y = self.current_position
        
        # Commencer à avancer
        self.set_speed(direction * speed)
        
        # Attendre jusqu'à ce que la distance soit parcourue
        distance_traveled = 0.0
        while distance_traveled < distance_cm / 100.0:  # Conversion en mètres
            # Calculer la distance parcourue
            current_x, current_y = self.current_position
            distance_traveled = math.sqrt((current_x - start_x)**2 + 
                                         (current_y - start_y)**2)
            
            # Petite pause pour éviter de surcharger le CPU
            time.sleep(0.01)
        
        # Arrêter le robot
        self.stop()
    
    def turn(self, angle_deg):
        """
        Fait tourner le robot sur place d'un angle spécifié
        
        Args:
            angle_deg (float): Angle de rotation en degrés
        """
        # Déterminer la direction
        direction = 1.0 if angle_deg >= 0 else -1.0
        angle_deg = abs(angle_deg)
        
        # Enregistrer l'orientation de départ
        start_orientation = self.current_orientation
        target_orientation = start_orientation + math.radians(angle_deg) * direction
        
        # Normaliser l'orientation cible entre -pi et pi
        target_orientation = ((target_orientation + math.pi) % (2 * math.pi)) - math.pi
        
        # Commencer à tourner
        self.set_speed(0.0, direction * 45.0)  # 45 deg/s
        
        # Attendre jusqu'à ce que l'angle soit atteint
        while True:
            # Calculer la différence d'angle
            angle_diff = abs(self.current_orientation - target_orientation)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            
            # Arrêter si on est assez proche de l'angle cible
            if angle_diff < math.radians(2.0):  # Tolérance de 2 degrés
                break
            
            # Petite pause pour éviter de surcharger le CPU
            time.sleep(0.01)
        
        # Arrêter le robot
        self.stop()
    
    def follow_line(self, speed=15.0, max_duration_sec=60.0):
        """
        Suit une ligne noire au sol
        
        Args:
            speed (float): Vitesse en cm/s
            max_duration_sec (float): Durée maximale en secondes
        """
        start_time = time.time()
        
        # Paramètres de l'algorithme de suivi de ligne
        base_speed = speed
        turn_factor = 0.5  # Facteur de correction pour les virages
        
        # Boucle principale de suivi de ligne
        while time.time() - start_time < max_duration_sec:
            # Vérifier si les capteurs IR détectent une ligne
            if self.ir_left_detected and self.ir_right_detected:
                # Les deux capteurs détectent la ligne, avancer tout droit
                self.set_speed(base_speed, 0.0)
            elif self.ir_left_detected:
                # Seulement le capteur gauche détecte la ligne, tourner à gauche
                self.set_speed(base_speed * 0.8, -30.0 * turn_factor)
            elif self.ir_right_detected:
                # Seulement le capteur droit détecte la ligne, tourner à droite
                self.set_speed(base_speed * 0.8, 30.0 * turn_factor)
            else:
                # Aucun capteur ne détecte la ligne, arrêter et déclencher l'événement
                self.stop()
                self._trigger_event('line_lost')
                break
            
            # Petite pause pour éviter de surcharger le CPU
            time.sleep(0.01)
        
        # Arrêter le robot à la fin
        self.stop()
    
    def avoid_obstacle(self):
        """
        Évite un obstacle détecté par le capteur ultrason
        """
        # Arrêter d'abord
        self.stop()
        
        # Tourner la caméra pour regarder à gauche et à droite
        self.set_camera_position(pan=30.0)
        time.sleep(0.5)  # Attendre que la caméra se positionne
        
        # Vérifier la distance à gauche
        left_distance = self.ultrasonic_distance
        
        # Regarder à droite
        self.set_camera_position(pan=-30.0)
        time.sleep(1.0)  # Attendre que la caméra se positionne
        
        # Vérifier la distance à droite
        right_distance = self.ultrasonic_distance
        
        # Remettre la caméra au centre
        self.set_camera_position(pan=0.0)
        
        # Décider de la direction à prendre
        if left_distance > right_distance:
            # Plus d'espace à gauche, tourner à gauche
            self.turn(90.0)
        else:
            # Plus d'espace à droite, tourner à droite
            self.turn(-90.0)
        
        # Avancer un peu
        self.drive(30.0)
    
    def track_color(self, rgb_color, speed=15.0, max_duration_sec=60.0):
        """
        Suit un objet de couleur spécifiée
        
        Args:
            rgb_color (list): Couleur RGB à suivre [r, g, b]
            speed (float): Vitesse en cm/s
            max_duration_sec (float): Durée maximale en secondes
        """
        if not CV_AVAILABLE:
            self.get_logger().error("OpenCV n'est pas disponible. Impossible de suivre une couleur.")
            return
        
        start_time = time.time()
        
        # Convertir la couleur RGB en HSV
        rgb_color = np.uint8([[rgb_color]])
        hsv_color = cv2.cvtColor(rgb_color, cv2.COLOR_RGB2HSV)[0][0]
        
        # Définir les plages de couleur HSV
        lower_bound = np.array([max(0, hsv_color[0] - 10), 100, 100])
        upper_bound = np.array([min(179, hsv_color[0] + 10), 255, 255])
        
        # Boucle principale de suivi de couleur
        while time.time() - start_time < max_duration_sec:
            if self.camera_image is None:
                time.sleep(0.1)
                continue
            
            # Convertir l'image en HSV
            hsv_image = cv2.cvtColor(self.camera_image, cv2.COLOR_BGR2HSV)
            
            # Créer un masque pour la couleur spécifiée
            mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
            
            # Trouver les contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # Trouver le plus grand contour
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                if area > 100:  # Ignorer les petits contours
                    # Trouver le centre du contour
                    M = cv2.moments(largest_contour)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Calculer l'erreur par rapport au centre de l'image
                        error_x = cx - self.camera_image.shape[1] / 2
                        
                        # Calculer la commande de direction
                        turn_amount = error_x / 10.0  # Facteur de conversion
                        
                        # Limiter la commande de direction
                        turn_amount = max(-30.0, min(30.0, turn_amount))
                        
                        # Calculer la vitesse en fonction de la taille du contour
                        adaptive_speed = min(speed, speed * (1000 / max(area, 100)))
                        
                        # Envoyer les commandes au robot
                        self.set_speed(adaptive_speed, turn_amount)
                    else:
                        self.stop()
                else:
                    self.stop()
            else:
                self.stop()
            
            # Petite pause pour éviter de surcharger le CPU
            time.sleep(0.05)
        
        # Arrêter le robot à la fin
        self.stop()
    
    # ===== Méthodes pour les événements =====
    
    def on_ultrasonic_close(self, threshold_cm=None):
        """
        Décorateur pour enregistrer un callback quand un obstacle est détecté
        
        Args:
            threshold_cm (float): Seuil de distance en cm
        """
        if threshold_cm is not None:
            self.thresholds['ultrasonic_close'] = threshold_cm / 100.0  # cm -> m
        
        def decorator(callback):
            self.callbacks['ultrasonic_close'].append(callback)
            return callback
        
        return decorator
    
    def on_line_detected(self):
        """
        Décorateur pour enregistrer un callback quand une ligne est détectée
        """
        def decorator(callback):
            self.callbacks['line_detected'].append(callback)
            return callback
        
        return decorator
    
    def on_line_lost(self):
        """
        Décorateur pour enregistrer un callback quand une ligne est perdue
        """
        def decorator(callback):
            self.callbacks['line_lost'].append(callback)
            return callback
        
        return decorator
    
    def on_battery_low(self, threshold_v=6.5):
        """
        Décorateur pour enregistrer un callback quand la batterie est faible
        
        Args:
            threshold_v (float): Seuil de tension en volts
        """
        self.thresholds['battery_low'] = threshold_v
        
        def decorator(callback):
            self.callbacks['battery_low'].append(callback)
            return callback
        
        return decorator
    
    def on_collision(self):
        """
        Décorateur pour enregistrer un callback en cas de collision
        """
        def decorator(callback):
            self.callbacks['collision'].append(callback)
            return callback
        
        return decorator
    
    # ===== Méthodes de callback pour les topics ROS =====
    
    def _ultrasonic_callback(self, msg):
        """Callback pour le capteur ultrason"""
        with self._lock:
            self.ultrasonic_distance = msg.range
    
    def _ir_left_callback(self, msg):
        """Callback pour le capteur IR gauche"""
        with self._lock:
            # Considérer que la ligne est détectée si la distance est inférieure au seuil
            self.ir_left_detected = msg.range < self.thresholds['line_detection']
    
    def _ir_right_callback(self, msg):
        """Callback pour le capteur IR droit"""
        with self._lock:
            # Considérer que la ligne est détectée si la distance est inférieure au seuil
            self.ir_right_detected = msg.range < self.thresholds['line_detection']
    
    def _odom_callback(self, msg):
        """Callback pour l'odométrie"""
        with self._lock:
            # Extraire la position
            self.current_position = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y
            )
            
            # Extraire l'orientation (conversion quaternion -> euler)
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            
            # Formule de conversion quaternion -> angle yaw
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            self.current_orientation = math.atan2(siny_cosp, cosy_cosp)
    
    def _battery_callback(self, msg):
        """Callback pour l'état de la batterie"""
        with self._lock:
            self.battery_voltage = msg.voltage
            self.battery_percentage = msg.percentage
    
    def _imu_callback(self, msg):
        """Callback pour l'IMU"""
        with self._lock:
            self.imu_data = msg
    
    def _camera_callback(self, msg):
        """Callback pour la caméra"""
        if CV_AVAILABLE:
            try:
                with self._lock:
                    self.camera_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception as e:
                self.get_logger().error(f"Erreur lors de la conversion de l'image: {e}")
    
    def _check_events(self):
        """Vérifie les conditions pour déclencher les événements"""
        with self._lock:
            # Vérifier la proximité d'un obstacle
            if (self.ultrasonic_distance < self.thresholds['ultrasonic_close']):
                self._trigger_event('ultrasonic_close')
            
            # Vérifier la détection de ligne
            if self.ir_left_detected or self.ir_right_detected:
                self._trigger_event('line_detected')
            
            # Vérifier le niveau de batterie
            if self.battery_voltage < self.thresholds['battery_low']:
                self._trigger_event('battery_low')
    
    def _trigger_event(self, event_name):
        """
        Déclenche un événement en appelant tous les callbacks enregistrés
        
        Args:
            event_name (str): Nom de l'événement
        """
        if event_name in self.callbacks:
            for callback in self.callbacks[event_name]:
                try:
                    callback()
                except Exception as e:
                    self.get_logger().error(f"Erreur dans le callback {event_name}: {e}")


def main(args=None):
    """Fonction principale pour exécuter le nœud PiCar"""
    rclpy.init(args=args)
    
    picar = PiCar()
    
    # Utiliser un exécuteur multi-thread pour gérer les callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(picar)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        picar.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()