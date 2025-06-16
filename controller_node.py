#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Compatibilité multiplateforme
import os
import sys
import platform

"""
Nœud contrôleur PID pour le PiCar X

Ce nœud implémente:
- Contrôleur PID pour la vitesse et le braquage (50 Hz)
- Anti-windup
- Auto-tune
- Callbacks pour les événements (ligne perdue, obstacle proche)
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
import time
import math
import numpy as np
from enum import Enum

# Messages ROS
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Range
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav_msgs.msg import Odometry


class PIDController:
    """Implémentation d'un contrôleur PID avec anti-windup et auto-tune"""
    
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limits=(-float('inf'), float('inf')),
                 sample_time=0.02, auto_tune=False):
        """
        Initialise le contrôleur PID
        
        Args:
            kp (float): Gain proportionnel
            ki (float): Gain intégral
            kd (float): Gain dérivé
            output_limits (tuple): Limites de sortie (min, max)
            sample_time (float): Temps d'échantillonnage en secondes
            auto_tune (bool): Activer l'auto-tune
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.sample_time = sample_time
        self.auto_tune = auto_tune
        
        self.reset()
    
    def reset(self):
        """Réinitialise le contrôleur"""
        self.last_time = time.monotonic()
        self.last_error = 0.0
        self.integral = 0.0
        self.last_output = 0.0
        
        # Variables pour l'auto-tune
        self.auto_tune_phase = 0
        self.auto_tune_cycles = 0
        self.auto_tune_peaks = []
        self.auto_tune_output_step = 0.1
    
    def compute(self, setpoint, process_value):
        """
        Calcule la sortie du contrôleur
        
        Args:
            setpoint (float): Valeur cible
            process_value (float): Valeur actuelle du processus
            
        Returns:
            float: Sortie du contrôleur
        """
        current_time = time.monotonic()
        dt = current_time - self.last_time
        
        # Vérifier si le temps d'échantillonnage est écoulé
        if dt < self.sample_time:
            return self.last_output
        
        # Calculer l'erreur
        error = setpoint - process_value
        
        # Calculer les termes PID
        p_term = self.kp * error
        
        # Terme intégral avec anti-windup
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Limiter le terme intégral (anti-windup)
        if self.output_limits[0] is not None and self.output_limits[1] is not None:
            i_term = max(self.output_limits[0], min(i_term, self.output_limits[1]))
        
        # Terme dérivé
        d_term = 0.0
        if dt > 0:
            d_term = self.kd * (error - self.last_error) / dt
        
        # Calculer la sortie
        output = p_term + i_term + d_term
        
        # Limiter la sortie
        if self.output_limits[0] is not None:
            output = max(self.output_limits[0], output)
        if self.output_limits[1] is not None:
            output = min(output, self.output_limits[1])
        
        # Auto-tune si activé
        if self.auto_tune:
            self._auto_tune(setpoint, process_value, output)
        
        # Mettre à jour les variables
        self.last_time = current_time
        self.last_error = error
        self.last_output = output
        
        return output
    
    def _auto_tune(self, setpoint, process_value, output):
        """
        Implémentation de l'auto-tune basée sur la méthode de Ziegler-Nichols
        
        Args:
            setpoint (float): Valeur cible
            process_value (float): Valeur actuelle du processus
            output (float): Sortie calculée
        """
        # Cette implémentation est simplifiée et devrait être étendue
        # pour une utilisation réelle
        
        # Détecter les oscillations
        if len(self.auto_tune_peaks) < 2:
            # Appliquer une perturbation
            if self.auto_tune_phase == 0:
                output = self.auto_tune_output_step
                self.auto_tune_phase = 1
            elif self.auto_tune_phase == 1 and process_value > setpoint:
                output = -self.auto_tune_output_step
                self.auto_tune_phase = 2
                self.auto_tune_peaks.append(process_value)
            elif self.auto_tune_phase == 2 and process_value < setpoint:
                output = self.auto_tune_output_step
                self.auto_tune_phase = 1
                self.auto_tune_peaks.append(process_value)
        else:
            # Calculer les paramètres PID basés sur l'amplitude et la période
            amplitude = abs(self.auto_tune_peaks[0] - self.auto_tune_peaks[1])
            period = time.monotonic() - self.last_time
            
            # Règles de Ziegler-Nichols
            ku = 4 * self.auto_tune_output_step / (math.pi * amplitude)
            tu = period
            
            self.kp = 0.6 * ku
            self.ki = 1.2 * ku / tu
            self.kd = 0.075 * ku * tu
            
            # Désactiver l'auto-tune
            self.auto_tune = False
            self.reset()


class ControlMode(Enum):
    """Modes de contrôle du robot"""
    MANUAL = 0
    VELOCITY = 1
    POSITION = 2


class ControllerNode(Node):
    """Nœud ROS 2 pour le contrôleur du PiCar X"""
    
    def __init__(self):
        """Initialise le nœud contrôleur"""
        super().__init__('controller_node')
        
        # Groupe de callbacks pour permettre l'exécution concurrente
        self.callback_group = ReentrantCallbackGroup()
        
        # Paramètres du contrôleur
        self.declare_parameter('control_rate', 50.0)  # Hz
        self.declare_parameter('velocity_kp', 1.0)
        self.declare_parameter('velocity_ki', 0.1)
        self.declare_parameter('velocity_kd', 0.05)
        self.declare_parameter('steering_kp', 2.0)
        self.declare_parameter('steering_ki', 0.0)
        self.declare_parameter('steering_kd', 0.1)
        self.declare_parameter('auto_tune', False)
        self.declare_parameter('ultrasonic_threshold', 0.2)  # 20cm
        self.declare_parameter('line_detection_threshold', 0.05)  # 5cm
        
        # Récupérer les paramètres
        control_rate = self.get_parameter('control_rate').value
        velocity_kp = self.get_parameter('velocity_kp').value
        velocity_ki = self.get_parameter('velocity_ki').value
        velocity_kd = self.get_parameter('velocity_kd').value
        steering_kp = self.get_parameter('steering_kp').value
        steering_ki = self.get_parameter('steering_ki').value
        steering_kd = self.get_parameter('steering_kd').value
        auto_tune = self.get_parameter('auto_tune').value
        self.ultrasonic_threshold = self.get_parameter('ultrasonic_threshold').value
        self.line_detection_threshold = self.get_parameter('line_detection_threshold').value
        
        # Contrôleurs PID
        self.velocity_pid = PIDController(
            kp=velocity_kp,
            ki=velocity_ki,
            kd=velocity_kd,
            output_limits=(-1.0, 1.0),
            sample_time=1.0/control_rate,
            auto_tune=auto_tune
        )
        
        self.steering_pid = PIDController(
            kp=steering_kp,
            ki=steering_ki,
            kd=steering_kd,
            output_limits=(-0.5236, 0.5236),  # ±30 degrés en radians
            sample_time=1.0/control_rate,
            auto_tune=auto_tune
        )
        
        # État du robot
        self.current_velocity = 0.0
        self.current_steering = 0.0
        self.target_velocity = 0.0
        self.target_steering = 0.0
        self.control_mode = ControlMode.VELOCITY
        self.emergency_stop = False
        
        # État des capteurs
        self.ultrasonic_distance = float('inf')
        self.ir_left_detected = False
        self.ir_right_detected = False
        
        # Callbacks enregistrés
        self.callbacks = {
            'on_line_lost': [],
            'on_ultrasonic_close': []
        }
        
        # Publishers
        self.motor_cmd_pub = self.create_publisher(
            Float64, 'picar/motors/cmd', 10)
        
        self.steering_cmd_pub = self.create_publisher(
            JointTrajectory, 'picar/steering/cmd', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_callback, 10,
            callback_group=self.callback_group)
        
        self.odom_sub = self.create_subscription(
            Odometry, 'picar/odom', self._odom_callback, 10,
            callback_group=self.callback_group)
        
        self.ultrasonic_sub = self.create_subscription(
            Range, 'picar/sensors/ultrasonic', self._ultrasonic_callback, 10,
            callback_group=self.callback_group)
        
        self.ir_left_sub = self.create_subscription(
            Range, 'picar/sensors/ir_left', self._ir_left_callback, 10,
            callback_group=self.callback_group)
        
        self.ir_right_sub = self.create_subscription(
            Range, 'picar/sensors/ir_right', self._ir_right_callback, 10,
            callback_group=self.callback_group)
        
        # Timer pour la boucle de contrôle
        self.control_timer = self.create_timer(
            1.0/control_rate, self._control_loop, callback_group=self.callback_group)
        
        # Verrou pour les opérations thread-safe
        self._lock = threading.RLock()
        
        self.get_logger().info("Nœud contrôleur initialisé")
    
    def _cmd_vel_callback(self, msg):
        """
        Callback pour les commandes de vitesse
        
        Args:
            msg (Twist): Message de commande de vitesse
        """
        with self._lock:
            self.target_velocity = msg.linear.x
            self.target_steering = msg.angular.z
            self.control_mode = ControlMode.VELOCITY
    
    def _odom_callback(self, msg):
        """
        Callback pour l'odométrie
        
        Args:
            msg (Odometry): Message d'odométrie
        """
        with self._lock:
            self.current_velocity = msg.twist.twist.linear.x
    
    def _ultrasonic_callback(self, msg):
        """
        Callback pour le capteur ultrason
        
        Args:
            msg (Range): Message du capteur ultrason
        """
        with self._lock:
            self.ultrasonic_distance = msg.range
            
            # Vérifier si un obstacle est détecté
            if self.ultrasonic_distance < self.ultrasonic_threshold:
                self._trigger_event('on_ultrasonic_close')
    
    def _ir_left_callback(self, msg):
        """
        Callback pour le capteur IR gauche
        
        Args:
            msg (Range): Message du capteur IR
        """
        with self._lock:
            # Considérer que la ligne est détectée si la distance est inférieure au seuil
            was_detected = self.ir_left_detected
            self.ir_left_detected = msg.range < self.line_detection_threshold
            
            # Vérifier si la ligne est perdue
            if was_detected and not self.ir_left_detected and not self.ir_right_detected:
                self._trigger_event('on_line_lost')
    
    def _ir_right_callback(self, msg):
        """
        Callback pour le capteur IR droit
        
        Args:
            msg (Range): Message du capteur IR
        """
        with self._lock:
            # Considérer que la ligne est détectée si la distance est inférieure au seuil
            was_detected = self.ir_right_detected
            self.ir_right_detected = msg.range < self.line_detection_threshold
            
            # Vérifier si la ligne est perdue
            if was_detected and not self.ir_right_detected and not self.ir_left_detected:
                self._trigger_event('on_line_lost')
    
    def _control_loop(self):
        """Boucle de contrôle principale (50 Hz)"""
        with self._lock:
            # Vérifier si l'arrêt d'urgence est activé
            if self.emergency_stop:
                self._send_motor_cmd(0.0)
                self._send_steering_cmd(0.0)
                return
            
            # Contrôle de la vitesse
            if self.control_mode == ControlMode.VELOCITY:
                # Calculer la commande de vitesse avec le PID
                velocity_cmd = self.velocity_pid.compute(
                    self.target_velocity, self.current_velocity)
                
                # Calculer la commande de direction avec le PID
                steering_cmd = self.steering_pid.compute(
                    self.target_steering, self.current_steering)
                
                # Envoyer les commandes
                self._send_motor_cmd(velocity_cmd)
                self._send_steering_cmd(steering_cmd)
    
    def _send_motor_cmd(self, cmd):
        """
        Envoie une commande aux moteurs
        
        Args:
            cmd (float): Commande de vitesse normalisée (-1.0 à 1.0)
        """
        motor_msg = Float64()
        motor_msg.data = cmd
        self.motor_cmd_pub.publish(motor_msg)
    
    def _send_steering_cmd(self, angle):
        """
        Envoie une commande de direction
        
        Args:
            angle (float): Angle de direction en radians
        """
        # Création du message de trajectoire
        trajectory = JointTrajectory()
        trajectory.joint_names = ['steering_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [angle]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000  # 0.1 secondes
        
        trajectory.points = [point]
        
        # Envoi du message
        self.steering_cmd_pub.publish(trajectory)
        
        # Mettre à jour l'angle de direction actuel
        self.current_steering = angle
    
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
    
    def on_line_lost(self, callback):
        """
        Enregistre un callback pour l'événement de perte de ligne
        
        Args:
            callback (callable): Fonction à appeler
        """
        self.callbacks['on_line_lost'].append(callback)
    
    def on_ultrasonic_close(self, callback):
        """
        Enregistre un callback pour l'événement d'obstacle proche
        
        Args:
            callback (callable): Fonction à appeler
        """
        self.callbacks['on_ultrasonic_close'].append(callback)
    
    def emergency_stop_activate(self):
        """Active l'arrêt d'urgence"""
        with self._lock:
            self.emergency_stop = True
            self._send_motor_cmd(0.0)
            self.get_logger().warn("Arrêt d'urgence activé")
    
    def emergency_stop_release(self):
        """Désactive l'arrêt d'urgence"""
        with self._lock:
            self.emergency_stop = False
            self.get_logger().info("Arrêt d'urgence désactivé")


def main(args=None):
    """Fonction principale pour exécuter le nœud contrôleur"""
    rclpy.init(args=args)
    
    controller = ControllerNode()
    
    # Utiliser un exécuteur multi-thread pour gérer les callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()