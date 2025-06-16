#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Nœud de simulation des capteurs pour le PiCar X

Ce nœud implémente:
- Simulation des capteurs avec bruit et latence
- Pannes aléatoires des capteurs
- Effets météorologiques sur les capteurs
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
import time
import math
import numpy as np
import random
from collections import deque

# Messages ROS
from sensor_msgs.msg import Range, Image, Imu, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Float64, String
from sensor_msgs.msg import BatteryState

# OpenCV pour le traitement d'image
try:
    import cv2
    from cv_bridge import CvBridge
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


class DelayBuffer:
    """
    Tampon pour simuler la latence des capteurs
    """
    
    def __init__(self, delay_sec=0.0):
        """
        Initialise le tampon de délai
        
        Args:
            delay_sec (float): Délai en secondes
        """
        self.delay_sec = delay_sec
        self.buffer = deque()
    
    def add(self, data):
        """
        Ajoute des données au tampon
        
        Args:
            data: Données à ajouter
        """
        self.buffer.append((time.monotonic(), data))
    
    def get(self):
        """
        Récupère les données après le délai
        
        Returns:
            Les données après le délai ou None si aucune donnée n'est disponible
        """
        if not self.buffer:
            return None
        
        current_time = time.monotonic()
        
        # Vérifier si les données les plus anciennes ont dépassé le délai
        oldest_time, oldest_data = self.buffer[0]
        if current_time - oldest_time >= self.delay_sec:
            self.buffer.popleft()
            return oldest_data
        
        return None


class SensorSimNode(Node):
    """Nœud ROS 2 pour la simulation des capteurs du PiCar X"""
    
    def __init__(self):
        """Initialise le nœud de simulation des capteurs"""
        super().__init__('sensor_sim_node')
        
        # Groupe de callbacks pour permettre l'exécution concurrente
        self.callback_group = ReentrantCallbackGroup()
        
        # Paramètres des capteurs
        self.declare_parameter('ultrasonic_noise_stddev', 0.01)  # ±1cm comme spécifié
        self.declare_parameter('ultrasonic_latency_min', 0.05)   # 50ms comme spécifié
        self.declare_parameter('ultrasonic_latency_max', 0.07)   # 70ms comme spécifié
        self.declare_parameter('ir_false_detection_prob', 0.05)  # 5% comme spécifié
        self.declare_parameter('imu_gyro_drift', 0.00035)        # 0.02°/s comme spécifié
        self.declare_parameter('imu_accel_noise', 0.02)          # ±0.02g comme spécifié
        self.declare_parameter('camera_noise', 0.1)              # σ=10% comme spécifié
        self.declare_parameter('camera_latency', 0.1)            # 100ms comme spécifié
        self.declare_parameter('sensor_failure_prob', 0.001)     # 1 drop-out/10min en moyenne
        self.declare_parameter('sensor_failure_duration_min', 1.0)
        self.declare_parameter('sensor_failure_duration_max', 5.0)
        
        # Récupérer les paramètres
        self.ultrasonic_noise_stddev = self.get_parameter('ultrasonic_noise_stddev').value
        self.ultrasonic_latency_min = self.get_parameter('ultrasonic_latency_min').value
        self.ultrasonic_latency_max = self.get_parameter('ultrasonic_latency_max').value
        self.ir_false_detection_prob = self.get_parameter('ir_false_detection_prob').value
        self.imu_gyro_drift = self.get_parameter('imu_gyro_drift').value
        self.imu_accel_noise = self.get_parameter('imu_accel_noise').value
        self.camera_noise = self.get_parameter('camera_noise').value
        self.camera_latency = self.get_parameter('camera_latency').value
        self.sensor_failure_prob = self.get_parameter('sensor_failure_prob').value
        self.sensor_failure_duration_min = self.get_parameter('sensor_failure_duration_min').value
        self.sensor_failure_duration_max = self.get_parameter('sensor_failure_duration_max').value
        
        # État des capteurs
        self.ultrasonic_range = float('inf')
        self.ir_left_range = float('inf')
        self.ir_right_range = float('inf')
        self.imu_data = None
        self.camera_image = None
        self.camera_info = None
        self.odom_data = None
        self.battery_voltage = 7.4
        self.battery_percentage = 100.0
        self.battery_current = 0.0
        
        # État des pannes
        self.sensor_failures = {
            'ultrasonic': False,
            'ir_left': False,
            'ir_right': False,
            'imu': False,
            'camera': False
        }
        self.failure_end_times = {
            'ultrasonic': 0.0,
            'ir_left': 0.0,
            'ir_right': 0.0,
            'imu': 0.0,
            'camera': 0.0
        }
        
        # Conditions météorologiques
        self.weather_condition = "clear"
        self.weather_modifiers = {
            'clear': {
                'friction': 1.0,
                'latency': 1.0,
                'visibility': 1.0
            },
            'rain': {
                'friction': 0.7,
                'latency': 1.2,
                'visibility': 0.8
            },
            'fog': {
                'friction': 0.9,
                'latency': 1.1,
                'visibility': 0.3
            }
        }
        
        # Tampons de délai pour les capteurs
        ultrasonic_latency = random.uniform(self.ultrasonic_latency_min, self.ultrasonic_latency_max)
        self.ultrasonic_buffer = DelayBuffer(ultrasonic_latency)
        self.ir_left_buffer = DelayBuffer(0.01)  # 10ms de latence
        self.ir_right_buffer = DelayBuffer(0.01)  # 10ms de latence
        self.imu_buffer = DelayBuffer(0.005)  # 5ms de latence
        self.camera_buffer = DelayBuffer(self.camera_latency)
        
        # Dérive du gyroscope
        self.gyro_drift = np.array([0.0, 0.0, 0.0])
        
        # Bridge OpenCV
        if CV_AVAILABLE:
            self.cv_bridge = CvBridge()
        
        # Publishers
        self.ultrasonic_pub = self.create_publisher(
            Range, 'picar/sensors/ultrasonic', 10)
        
        self.ir_left_pub = self.create_publisher(
            Range, 'picar/sensors/ir_left', 10)
        
        self.ir_right_pub = self.create_publisher(
            Range, 'picar/sensors/ir_right', 10)
        
        self.imu_pub = self.create_publisher(
            Imu, 'picar/sensors/imu', 10)
        
        self.camera_pub = self.create_publisher(
            Image, 'picar/camera/image_raw', 10)
        
        self.camera_info_pub = self.create_publisher(
            CameraInfo, 'picar/camera/camera_info', 10)
        
        self.battery_pub = self.create_publisher(
            BatteryState, 'picar/power/battery', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'picar/odom', self._odom_callback, 10,
            callback_group=self.callback_group)
        
        self.weather_sub = self.create_subscription(
            String, 'picar/world/weather', self._weather_callback, 10,
            callback_group=self.callback_group)
        
        self.power_load_sub = self.create_subscription(
            Float64, 'picar/power/load', self._power_load_callback, 10,
            callback_group=self.callback_group)
        
        # Timers
        self.ultrasonic_timer = self.create_timer(
            0.05, self._ultrasonic_timer_callback, callback_group=self.callback_group)
        
        self.ir_timer = self.create_timer(
            0.05, self._ir_timer_callback, callback_group=self.callback_group)
        
        self.imu_timer = self.create_timer(
            0.005, self._imu_timer_callback, callback_group=self.callback_group)  # 200Hz
        
        self.camera_timer = self.create_timer(
            0.033, self._camera_timer_callback, callback_group=self.callback_group)  # 30Hz
        
        self.battery_timer = self.create_timer(
            1.0, self._battery_timer_callback, callback_group=self.callback_group)
        
        self.failure_timer = self.create_timer(
            1.0, self._failure_timer_callback, callback_group=self.callback_group)
        
        # Verrou pour les opérations thread-safe
        self._lock = threading.RLock()
        
        self.get_logger().info("Nœud de simulation des capteurs initialisé")
    
    def _odom_callback(self, msg):
        """
        Callback pour l'odométrie
        
        Args:
            msg (Odometry): Message d'odométrie
        """
        with self._lock:
            self.odom_data = msg
    
    def _weather_callback(self, msg):
        """
        Callback pour les conditions météorologiques
        
        Args:
            msg (String): Message de condition météorologique
        """
        with self._lock:
            condition = msg.data
            if condition in self.weather_modifiers:
                self.weather_condition = condition
                
                # Mettre à jour les latences des capteurs en fonction de la météo
                latency_modifier = self.weather_modifiers[condition]['latency']
                ultrasonic_latency = random.uniform(
                    self.ultrasonic_latency_min * latency_modifier,
                    self.ultrasonic_latency_max * latency_modifier
                )
                self.ultrasonic_buffer.delay_sec = ultrasonic_latency
                self.camera_buffer.delay_sec = self.camera_latency * latency_modifier
                
                self.get_logger().info(f"Condition météorologique changée: {condition}")
    
    def _power_load_callback(self, msg):
        """
        Callback pour la charge de puissance
        
        Args:
            msg (Float64): Message de charge de puissance
        """
        with self._lock:
            self.battery_current = msg.data
    
    def _ultrasonic_timer_callback(self):
        """Timer pour publier les données du capteur ultrason"""
        with self._lock:
            # Vérifier si le capteur est en panne
            if self.sensor_failures['ultrasonic']:
                return
            
            # Récupérer les données du tampon
            range_data = self.ultrasonic_buffer.get()
            if range_data is not None:
                self.ultrasonic_pub.publish(range_data)
    
    def _ir_timer_callback(self):
        """Timer pour publier les données des capteurs IR"""
        with self._lock:
            # Capteur IR gauche
            if not self.sensor_failures['ir_left']:
                ir_left_data = self.ir_left_buffer.get()
                if ir_left_data is not None:
                    self.ir_left_pub.publish(ir_left_data)
            
            # Capteur IR droit
            if not self.sensor_failures['ir_right']:
                ir_right_data = self.ir_right_buffer.get()
                if ir_right_data is not None:
                    self.ir_right_pub.publish(ir_right_data)
    
    def _imu_timer_callback(self):
        """Timer pour publier les données de l'IMU"""
        with self._lock:
            # Vérifier si le capteur est en panne
            if self.sensor_failures['imu']:
                return
            
            # Récupérer les données du tampon
            imu_data = self.imu_buffer.get()
            if imu_data is not None:
                self.imu_pub.publish(imu_data)
    
    def _camera_timer_callback(self):
        """Timer pour publier les données de la caméra"""
        with self._lock:
            # Vérifier si le capteur est en panne
            if self.sensor_failures['camera']:
                return
            
            # Récupérer les données du tampon
            camera_data = self.camera_buffer.get()
            if camera_data is not None and CV_AVAILABLE:
                # Publier l'image
                self.camera_pub.publish(camera_data)
                
                # Publier les informations de la caméra
                if self.camera_info is not None:
                    self.camera_info_pub.publish(self.camera_info)
    
    def _battery_timer_callback(self):
        """Timer pour publier les données de la batterie"""
        with self._lock:
            # Simuler la décharge de la batterie
            if self.battery_current > 0:
                # Chute de tension sous charge
                voltage_drop = 0.5 * self.battery_current
                current_voltage = self.battery_voltage - voltage_drop
                
                # Limiter la tension minimale
                current_voltage = max(6.0, current_voltage)
                
                # Décharge progressive
                self.battery_percentage -= 0.05 * self.battery_current
                self.battery_percentage = max(0.0, self.battery_percentage)
            else:
                current_voltage = self.battery_voltage
            
            # Créer le message de batterie
            battery_msg = BatteryState()
            battery_msg.voltage = current_voltage
            battery_msg.percentage = self.battery_percentage
            battery_msg.current = self.battery_current
            battery_msg.charge = 2.2 * self.battery_percentage / 100.0  # 2200mAh
            battery_msg.capacity = 2.2  # 2200mAh
            battery_msg.design_capacity = 2.2  # 2200mAh
            battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
            battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
            battery_msg.present = True
            
            # Publier le message
            self.battery_pub.publish(battery_msg)
    
    def _failure_timer_callback(self):
        """Timer pour gérer les pannes aléatoires des capteurs"""
        with self._lock:
            current_time = time.monotonic()
            
            # Vérifier si des pannes doivent être terminées
            for sensor in self.sensor_failures:
                if self.sensor_failures[sensor] and current_time >= self.failure_end_times[sensor]:
                    self.sensor_failures[sensor] = False
                    self.get_logger().info(f"Capteur {sensor} rétabli")
            
            # Générer de nouvelles pannes aléatoires
            for sensor in self.sensor_failures:
                if not self.sensor_failures[sensor] and random.random() < self.sensor_failure_prob:
                    self.sensor_failures[sensor] = True
                    duration = random.uniform(
                        self.sensor_failure_duration_min,
                        self.sensor_failure_duration_max
                    )
                    self.failure_end_times[sensor] = current_time + duration
                    self.get_logger().warn(f"Panne du capteur {sensor} pendant {duration:.1f}s")
    
    def update_ultrasonic(self, range_value):
        """
        Met à jour les données du capteur ultrason
        
        Args:
            range_value (float): Distance mesurée en mètres
        """
        with self._lock:
            # Ajouter du bruit gaussien
            noisy_range = range_value + np.random.normal(0, self.ultrasonic_noise_stddev)
            
            # Limiter la plage
            noisy_range = max(0.02, min(4.0, noisy_range))
            
            # Créer le message
            range_msg = Range()
            range_msg.header.stamp = self.get_clock().now().to_msg()
            range_msg.header.frame_id = "ultrasonic_link"
            range_msg.radiation_type = Range.ULTRASOUND
            range_msg.field_of_view = 0.2  # ~11 degrés
            range_msg.min_range = 0.02
            range_msg.max_range = 4.0
            range_msg.range = noisy_range
            
            # Ajouter au tampon de délai
            self.ultrasonic_buffer.add(range_msg)
    
    def update_ir(self, left_range, right_range):
        """
        Met à jour les données des capteurs IR
        
        Args:
            left_range (float): Distance mesurée par le capteur gauche en mètres
            right_range (float): Distance mesurée par le capteur droit en mètres
        """
        with self._lock:
            # Ajouter des fausses détections aléatoires
            if random.random() < self.ir_false_detection_prob:
                left_range = 0.03  # Fausse détection à 3cm
            
            if random.random() < self.ir_false_detection_prob:
                right_range = 0.03  # Fausse détection à 3cm
            
            # Créer les messages
            left_msg = Range()
            left_msg.header.stamp = self.get_clock().now().to_msg()
            left_msg.header.frame_id = "ir_left_link"
            left_msg.radiation_type = Range.INFRARED
            left_msg.field_of_view = 0.1  # ~6 degrés
            left_msg.min_range = 0.01
            left_msg.max_range = 0.1
            left_msg.range = left_range
            
            right_msg = Range()
            right_msg.header.stamp = self.get_clock().now().to_msg()
            right_msg.header.frame_id = "ir_right_link"
            right_msg.radiation_type = Range.INFRARED
            right_msg.field_of_view = 0.1  # ~6 degrés
            right_msg.min_range = 0.01
            right_msg.max_range = 0.1
            right_msg.range = right_range
            
            # Ajouter aux tampons de délai
            self.ir_left_buffer.add(left_msg)
            self.ir_right_buffer.add(right_msg)
    
    def update_imu(self, angular_velocity, linear_acceleration, orientation):
        """
        Met à jour les données de l'IMU
        
        Args:
            angular_velocity (Vector3): Vitesse angulaire
            linear_acceleration (Vector3): Accélération linéaire
            orientation (Quaternion): Orientation
        """
        with self._lock:
            # Mettre à jour la dérive du gyroscope
            self.gyro_drift += np.random.normal(0, self.imu_gyro_drift, 3)
            
            # Ajouter la dérive et le bruit
            noisy_angular_velocity = Vector3()
            noisy_angular_velocity.x = angular_velocity.x + self.gyro_drift[0]
            noisy_angular_velocity.y = angular_velocity.y + self.gyro_drift[1]
            noisy_angular_velocity.z = angular_velocity.z + self.gyro_drift[2]
            
            noisy_linear_acceleration = Vector3()
            noisy_linear_acceleration.x = linear_acceleration.x + np.random.normal(0, self.imu_accel_noise)
            noisy_linear_acceleration.y = linear_acceleration.y + np.random.normal(0, self.imu_accel_noise)
            noisy_linear_acceleration.z = linear_acceleration.z + np.random.normal(0, self.imu_accel_noise)
            
            # Créer le message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"
            imu_msg.orientation = orientation
            imu_msg.angular_velocity = noisy_angular_velocity
            imu_msg.linear_acceleration = noisy_linear_acceleration
            
            # Covariances (incertitudes)
            imu_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            imu_msg.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            imu_msg.linear_acceleration_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            
            # Ajouter au tampon de délai
            self.imu_buffer.add(imu_msg)
    
    def update_camera(self, image):
        """
        Met à jour les données de la caméra
        
        Args:
            image: Image OpenCV (numpy array)
        """
        with self._lock:
            if not CV_AVAILABLE or image is None:
                return
            
            # Appliquer les effets météorologiques
            visibility = self.weather_modifiers[self.weather_condition]['visibility']
            if visibility < 1.0:
                # Simuler le brouillard/pluie en réduisant le contraste et la luminosité
                image = cv2.convertScaleAbs(image, alpha=visibility, beta=(1-visibility)*50)
            
            # Ajouter du bruit gaussien
            noise = np.random.normal(0, self.camera_noise * 255, image.shape).astype(np.uint8)
            noisy_image = cv2.add(image, noise)
            
            # Appliquer la distorsion de l'objectif
            if self.camera_info is not None:
                k1, k2, p1, p2, k3 = self.camera_info.d
                camera_matrix = np.array(self.camera_info.k).reshape(3, 3)
                h, w = noisy_image.shape[:2]
                new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, (k1, k2, p1, p2, k3), (w, h), 0)
                distorted_image = cv2.undistort(noisy_image, camera_matrix, (k1, k2, p1, p2, k3), None, new_camera_matrix)
                x, y, w, h = roi
                distorted_image = distorted_image[y:y+h, x:x+w]
            else:
                distorted_image = noisy_image
            
            # Convertir en message ROS
            try:
                img_msg = self.cv_bridge.cv2_to_imgmsg(distorted_image, "bgr8")
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = "camera_link"
                
                # Ajouter au tampon de délai
                self.camera_buffer.add(img_msg)
            except Exception as e:
                self.get_logger().error(f"Erreur lors de la conversion de l'image: {e}")
    
    def set_camera_info(self, width, height, fx, fy, cx, cy, k1, k2, p1, p2, k3):
        """
        Définit les paramètres de calibration de la caméra
        
        Args:
            width (int): Largeur de l'image
            height (int): Hauteur de l'image
            fx, fy (float): Focales
            cx, cy (float): Centre optique
            k1, k2, k3 (float): Coefficients de distorsion radiale
            p1, p2 (float): Coefficients de distorsion tangentielle
        """
        with self._lock:
            info_msg = CameraInfo()
            info_msg.header.frame_id = "camera_link"
            info_msg.height = height
            info_msg.width = width
            
            # Matrice de calibration
            info_msg.k = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            
            # Coefficients de distorsion
            info_msg.d = [k1, k2, p1, p2, k3]
            
            # Matrice de projection
            info_msg.p = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
            
            # Matrice de rectification
            info_msg.r = [1, 0, 0, 0, 1, 0, 0, 0, 1]
            
            self.camera_info = info_msg


def main(args=None):
    """Fonction principale pour exécuter le nœud de simulation des capteurs"""
    rclpy.init(args=args)
    
    sensor_sim = SensorSimNode()
    
    # Définir les paramètres de calibration de la caméra
    sensor_sim.set_camera_info(
        width=640,
        height=480,
        fx=500,
        fy=500,
        cx=320,
        cy=240,
        k1=0.1,
        k2=0.05,
        p1=0.002,
        p2=0.002,
        k3=0.01
    )
    
    # Utiliser un exécuteur multi-thread pour gérer les callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(sensor_sim)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        sensor_sim.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()