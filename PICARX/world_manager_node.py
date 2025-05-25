#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Nœud de gestion de l'environnement pour le PiCar X

Ce nœud implémente:
- Gestion des zones (atelier, urbain, tout-terrain, défi)
- Gestion des conditions météorologiques
- Cycle jour/nuit
- Gestion des objets dynamiques (piétons, obstacles mobiles)
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
from enum import Enum

# Messages ROS
from std_msgs.msg import String, Float64, Bool, Int32
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray


class Zone(Enum):
    """Zones de l'environnement"""
    WORKSHOP = 0
    URBAN = 1
    OFFROAD = 2
    CHALLENGE = 3


class Weather(Enum):
    """Conditions météorologiques"""
    CLEAR = 0
    RAIN = 1
    FOG = 2


class TimeOfDay(Enum):
    """Moments de la journée"""
    DAY = 0
    NIGHT = 1


class WorldManagerNode(Node):
    """Nœud ROS 2 pour la gestion de l'environnement du PiCar X"""
    
    def __init__(self):
        """Initialise le nœud de gestion de l'environnement"""
        super().__init__('world_manager_node')
        
        # Groupe de callbacks pour permettre l'exécution concurrente
        self.callback_group = ReentrantCallbackGroup()
        
        # Paramètres
        self.declare_parameter('update_rate', 10.0)  # Hz
        self.declare_parameter('weather_change_prob', 0.001)  # Probabilité de changement météo par seconde
        self.declare_parameter('day_night_cycle_duration', 600.0)  # Durée du cycle jour/nuit en secondes
        self.declare_parameter('pedestrian_count', 5)  # Nombre de piétons dans la zone urbaine
        self.declare_parameter('obstacle_count', 3)  # Nombre d'obstacles mobiles
        
        # Récupérer les paramètres
        self.update_rate = self.get_parameter('update_rate').value
        self.weather_change_prob = self.get_parameter('weather_change_prob').value
        self.day_night_cycle_duration = self.get_parameter('day_night_cycle_duration').value
        self.pedestrian_count = self.get_parameter('pedestrian_count').value
        self.obstacle_count = self.get_parameter('obstacle_count').value
        
        # État du monde
        self.current_zone = Zone.WORKSHOP
        self.current_weather = Weather.CLEAR
        self.current_time_of_day = TimeOfDay.DAY
        self.simulation_time = 0.0  # Temps de simulation en secondes
        
        # Objets dynamiques
        self.pedestrians = []  # Liste des piétons
        self.obstacles = []    # Liste des obstacles mobiles
        
        # Définition des zones
        self.zones = {
            Zone.WORKSHOP: {
                'name': 'workshop',
                'center': (0.0, 0.0),
                'radius': 5.0,
                'friction': 0.8,  # Bitume
                'pedestrians_enabled': False,
                'obstacles_enabled': False
            },
            Zone.URBAN: {
                'name': 'urban',
                'center': (10.0, 0.0),
                'radius': 10.0,
                'friction': 0.8,  # Bitume
                'pedestrians_enabled': True,
                'obstacles_enabled': True
            },
            Zone.OFFROAD: {
                'name': 'offroad',
                'center': (0.0, 10.0),
                'radius': 8.0,
                'friction': 0.4,  # Herbe
                'pedestrians_enabled': False,
                'obstacles_enabled': True
            },
            Zone.CHALLENGE: {
                'name': 'challenge',
                'center': (10.0, 10.0),
                'radius': 7.0,
                'friction': 0.6,  # Carrelage
                'pedestrians_enabled': True,
                'obstacles_enabled': True
            }
        }
        
        # Position du robot
        self.robot_position = (0.0, 0.0)
        
        # Publishers
        self.zone_pub = self.create_publisher(
            String, 'picar/world/zone', 10)
        
        self.friction_pub = self.create_publisher(
            Float64, 'picar/world/friction', 10)
        
        self.weather_pub = self.create_publisher(
            String, 'picar/world/weather', 10)
        
        self.time_pub = self.create_publisher(
            String, 'picar/world/time', 10)
        
        self.event_pub = self.create_publisher(
            String, 'picar/world/events', 10)
        
        self.pedestrian_pub = self.create_publisher(
            MarkerArray, 'picar/world/pedestrians', 10)
        
        self.obstacle_pub = self.create_publisher(
            MarkerArray, 'picar/world/obstacles', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'picar/odom', self._odom_callback, 10,
            callback_group=self.callback_group)
        
        self.zone_change_sub = self.create_subscription(
            String, 'picar/world/set_zone', self._zone_change_callback, 10,
            callback_group=self.callback_group)
        
        self.weather_change_sub = self.create_subscription(
            String, 'picar/world/set_weather', self._weather_change_callback, 10,
            callback_group=self.callback_group)
        
        self.time_change_sub = self.create_subscription(
            String, 'picar/world/set_time', self._time_change_callback, 10,
            callback_group=self.callback_group)
        
        # Timers
        self.update_timer = self.create_timer(
            1.0/self.update_rate, self._update_timer_callback, callback_group=self.callback_group)
        
        self.weather_timer = self.create_timer(
            1.0, self._weather_timer_callback, callback_group=self.callback_group)
        
        self.time_timer = self.create_timer(
            1.0, self._time_timer_callback, callback_group=self.callback_group)
        
        # Initialiser les objets dynamiques
        self._init_pedestrians()
        self._init_obstacles()
        
        # Verrou pour les opérations thread-safe
        self._lock = threading.RLock()
        
        self.get_logger().info("Nœud de gestion de l'environnement initialisé")
    
    def _odom_callback(self, msg):
        """
        Callback pour l'odométrie
        
        Args:
            msg (Odometry): Message d'odométrie
        """
        with self._lock:
            # Mettre à jour la position du robot
            self.robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            
            # Vérifier si le robot a changé de zone
            self._check_zone_change()
    
    def _zone_change_callback(self, msg):
        """
        Callback pour le changement manuel de zone
        
        Args:
            msg (String): Message de changement de zone
        """
        with self._lock:
            zone_name = msg.data.lower()
            for zone in Zone:
                if self.zones[zone]['name'] == zone_name:
                    self._change_zone(zone)
                    break
    
    def _weather_change_callback(self, msg):
        """
        Callback pour le changement manuel de météo
        
        Args:
            msg (String): Message de changement de météo
        """
        with self._lock:
            weather_name = msg.data.lower()
            if weather_name == 'clear':
                self._change_weather(Weather.CLEAR)
            elif weather_name == 'rain':
                self._change_weather(Weather.RAIN)
            elif weather_name == 'fog':
                self._change_weather(Weather.FOG)
    
    def _time_change_callback(self, msg):
        """
        Callback pour le changement manuel de l'heure
        
        Args:
            msg (String): Message de changement d'heure
        """
        with self._lock:
            time_name = msg.data.lower()
            if time_name == 'day':
                self._change_time(TimeOfDay.DAY)
            elif time_name == 'night':
                self._change_time(TimeOfDay.NIGHT)
    
    def _update_timer_callback(self):
        """Timer pour mettre à jour l'état du monde"""
        with self._lock:
            # Mettre à jour les objets dynamiques
            self._update_pedestrians()
            self._update_obstacles()
            
            # Publier l'état du monde
            self._publish_world_state()
    
    def _weather_timer_callback(self):
        """Timer pour gérer les changements météorologiques aléatoires"""
        with self._lock:
            # Changement aléatoire de météo
            if random.random() < self.weather_change_prob:
                # Choisir une nouvelle météo différente de l'actuelle
                new_weather = random.choice([w for w in Weather if w != self.current_weather])
                self._change_weather(new_weather)
    
    def _time_timer_callback(self):
        """Timer pour gérer le cycle jour/nuit"""
        with self._lock:
            # Mettre à jour le temps de simulation
            self.simulation_time += 1.0
            
            # Vérifier si le cycle jour/nuit doit changer
            cycle_position = (self.simulation_time % self.day_night_cycle_duration) / self.day_night_cycle_duration
            
            if cycle_position < 0.5 and self.current_time_of_day != TimeOfDay.DAY:
                self._change_time(TimeOfDay.DAY)
            elif cycle_position >= 0.5 and self.current_time_of_day != TimeOfDay.NIGHT:
                self._change_time(TimeOfDay.NIGHT)
    
    def _check_zone_change(self):
        """Vérifie si le robot a changé de zone"""
        # Déterminer la zone actuelle
        current_zone = self.current_zone
        min_distance = float('inf')
        
        for zone in Zone:
            center = self.zones[zone]['center']
            radius = self.zones[zone]['radius']
            
            # Calculer la distance au centre de la zone
            dx = self.robot_position[0] - center[0]
            dy = self.robot_position[1] - center[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Vérifier si le robot est dans cette zone
            if distance < radius and distance < min_distance:
                current_zone = zone
                min_distance = distance
        
        # Changer de zone si nécessaire
        if current_zone != self.current_zone:
            self._change_zone(current_zone)
    
    def _change_zone(self, new_zone):
        """
        Change la zone actuelle
        
        Args:
            new_zone (Zone): Nouvelle zone
        """
        self.current_zone = new_zone
        zone_name = self.zones[new_zone]['name']
        
        # Publier le changement de zone
        zone_msg = String()
        zone_msg.data = zone_name
        self.zone_pub.publish(zone_msg)
        
        # Publier le coefficient de friction
        friction_msg = Float64()
        friction_msg.data = self.zones[new_zone]['friction']
        self.friction_pub.publish(friction_msg)
        
        # Publier un événement
        event_msg = String()
        event_msg.data = f"zone_changed:{zone_name}"
        self.event_pub.publish(event_msg)
        
        self.get_logger().info(f"Zone changée: {zone_name}")
    
    def _change_weather(self, new_weather):
        """
        Change la condition météorologique
        
        Args:
            new_weather (Weather): Nouvelle condition météorologique
        """
        self.current_weather = new_weather
        
        # Publier le changement de météo
        weather_msg = String()
        if new_weather == Weather.CLEAR:
            weather_msg.data = "clear"
        elif new_weather == Weather.RAIN:
            weather_msg.data = "rain"
        elif new_weather == Weather.FOG:
            weather_msg.data = "fog"
        
        self.weather_pub.publish(weather_msg)
        
        # Publier un événement
        event_msg = String()
        event_msg.data = f"weather_changed:{weather_msg.data}"
        self.event_pub.publish(event_msg)
        
        self.get_logger().info(f"Météo changée: {weather_msg.data}")
    
    def _change_time(self, new_time):
        """
        Change l'heure de la journée
        
        Args:
            new_time (TimeOfDay): Nouvelle heure
        """
        self.current_time_of_day = new_time
        
        # Publier le changement d'heure
        time_msg = String()
        if new_time == TimeOfDay.DAY:
            time_msg.data = "day"
        elif new_time == TimeOfDay.NIGHT:
            time_msg.data = "night"
        
        self.time_pub.publish(time_msg)
        
        # Publier un événement
        event_msg = String()
        event_msg.data = f"time_changed:{time_msg.data}"
        self.event_pub.publish(event_msg)
        
        self.get_logger().info(f"Heure changée: {time_msg.data}")
    
    def _init_pedestrians(self):
        """Initialise les piétons"""
        self.pedestrians = []
        
        for i in range(self.pedestrian_count):
            # Position aléatoire dans la zone urbaine
            center = self.zones[Zone.URBAN]['center']
            radius = self.zones[Zone.URBAN]['radius'] * 0.8  # 80% du rayon pour éviter les bords
            
            angle = random.uniform(0, 2 * math.pi)
            distance = random.uniform(0, radius)
            
            x = center[0] + distance * math.cos(angle)
            y = center[1] + distance * math.sin(angle)
            
            # Direction aléatoire
            direction = random.uniform(0, 2 * math.pi)
            
            # Vitesse aléatoire (0.5 - 1.5 m/s)
            speed = random.uniform(0.5, 1.5)
            
            self.pedestrians.append({
                'id': i,
                'position': (x, y),
                'direction': direction,
                'speed': speed,
                'state': 'walking',  # walking, waiting, crossing
                'wait_time': 0.0,
                'color': (random.random(), random.random(), random.random(), 1.0)
            })
    
    def _init_obstacles(self):
        """Initialise les obstacles mobiles"""
        self.obstacles = []
        
        for i in range(self.obstacle_count):
            # Position aléatoire dans la zone de défi
            center = self.zones[Zone.CHALLENGE]['center']
            radius = self.zones[Zone.CHALLENGE]['radius'] * 0.8  # 80% du rayon pour éviter les bords
            
            angle = random.uniform(0, 2 * math.pi)
            distance = random.uniform(0, radius)
            
            x = center[0] + distance * math.cos(angle)
            y = center[1] + distance * math.sin(angle)
            
            # Direction aléatoire
            direction = random.uniform(0, 2 * math.pi)
            
            # Vitesse aléatoire (0.2 - 1.0 m/s)
            speed = random.uniform(0.2, 1.0)
            
            # Type d'obstacle aléatoire
            obstacle_type = random.choice(['box', 'cylinder', 'sphere'])
            
            # Taille aléatoire
            if obstacle_type == 'box':
                size = (random.uniform(0.3, 0.5), random.uniform(0.3, 0.5), random.uniform(0.3, 0.5))
            elif obstacle_type == 'cylinder':
                size = (random.uniform(0.2, 0.4), random.uniform(0.3, 0.6))  # rayon, hauteur
            else:  # sphere
                size = random.uniform(0.2, 0.4)  # rayon
            
            self.obstacles.append({
                'id': i,
                'position': (x, y),
                'direction': direction,
                'speed': speed,
                'type': obstacle_type,
                'size': size,
                'color': (random.random(), random.random(), random.random(), 1.0)
            })
    
    def _update_pedestrians(self):
        """Met à jour les positions et comportements des piétons"""
        # Vérifier si les piétons sont activés dans la zone actuelle
        if not self.zones[self.current_zone]['pedestrians_enabled']:
            return
        
        dt = 1.0 / self.update_rate
        
        for pedestrian in self.pedestrians:
            # Mettre à jour selon l'état
            if pedestrian['state'] == 'walking':
                # Déplacer le piéton
                x, y = pedestrian['position']
                direction = pedestrian['direction']
                speed = pedestrian['speed']
                
                # Nouvelle position
                x += speed * dt * math.cos(direction)
                y += speed * dt * math.sin(direction)
                
                # Vérifier les limites de la zone urbaine
                center = self.zones[Zone.URBAN]['center']
                radius = self.zones[Zone.URBAN]['radius']
                
                dx = x - center[0]
                dy = y - center[1]
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance > radius:
                    # Rebondir sur les bords
                    angle_to_center = math.atan2(center[1] - y, center[0] - x)
                    direction = 2 * angle_to_center - direction
                    
                    # Ajuster la position
                    x = center[0] + radius * math.cos(angle_to_center + math.pi)
                    y = center[1] + radius * math.sin(angle_to_center + math.pi)
                
                # Mettre à jour la position et la direction
                pedestrian['position'] = (x, y)
                pedestrian['direction'] = direction
                
                # Changer d'état aléatoirement
                if random.random() < 0.01:  # 1% de chance par mise à jour
                    pedestrian['state'] = 'waiting'
                    pedestrian['wait_time'] = random.uniform(1.0, 5.0)  # Attendre 1-5 secondes
            
            elif pedestrian['state'] == 'waiting':
                # Décrémenter le temps d'attente
                pedestrian['wait_time'] -= dt
                
                if pedestrian['wait_time'] <= 0:
                    # Changer d'état
                    if random.random() < 0.7:  # 70% de chance de reprendre la marche
                        pedestrian['state'] = 'walking'
                        # Nouvelle direction aléatoire
                        pedestrian['direction'] = random.uniform(0, 2 * math.pi)
                    else:
                        pedestrian['state'] = 'crossing'
                        # Direction vers un point aléatoire
                        target_angle = random.uniform(0, 2 * math.pi)
                        target_distance = random.uniform(2.0, 5.0)
                        target_x = pedestrian['position'][0] + target_distance * math.cos(target_angle)
                        target_y = pedestrian['position'][1] + target_distance * math.sin(target_angle)
                        
                        # Direction vers la cible
                        dx = target_x - pedestrian['position'][0]
                        dy = target_y - pedestrian['position'][1]
                        pedestrian['direction'] = math.atan2(dy, dx)
            
            elif pedestrian['state'] == 'crossing':
                # Déplacer le piéton plus lentement
                x, y = pedestrian['position']
                direction = pedestrian['direction']
                speed = pedestrian['speed'] * 0.7  # Plus lent pendant la traversée
                
                # Nouvelle position
                x += speed * dt * math.cos(direction)
                y += speed * dt * math.sin(direction)
                
                # Mettre à jour la position
                pedestrian['position'] = (x, y)
                
                # Changer d'état aléatoirement
                if random.random() < 0.05:  # 5% de chance par mise à jour
                    pedestrian['state'] = 'walking'
    
    def _update_obstacles(self):
        """Met à jour les positions des obstacles mobiles"""
        # Vérifier si les obstacles sont activés dans la zone actuelle
        if not self.zones[self.current_zone]['obstacles_enabled']:
            return
        
        dt = 1.0 / self.update_rate
        
        for obstacle in self.obstacles:
            # Déplacer l'obstacle
            x, y = obstacle['position']
            direction = obstacle['direction']
            speed = obstacle['speed']
            
            # Nouvelle position
            x += speed * dt * math.cos(direction)
            y += speed * dt * math.sin(direction)
            
            # Vérifier les limites de la zone
            center = self.zones[Zone.CHALLENGE]['center']
            radius = self.zones[Zone.CHALLENGE]['radius']
            
            dx = x - center[0]
            dy = y - center[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance > radius:
                # Rebondir sur les bords
                angle_to_center = math.atan2(center[1] - y, center[0] - x)
                direction = 2 * angle_to_center - direction
                
                # Ajuster la position
                x = center[0] + radius * math.cos(angle_to_center + math.pi)
                y = center[1] + radius * math.sin(angle_to_center + math.pi)
            
            # Mettre à jour la position et la direction
            obstacle['position'] = (x, y)
            obstacle['direction'] = direction
            
            # Changer de direction aléatoirement
            if random.random() < 0.02:  # 2% de chance par mise à jour
                obstacle['direction'] += random.uniform(-math.pi/4, math.pi/4)
    
    def _publish_world_state(self):
        """Publie l'état du monde"""
        # Publier les piétons
        if self.zones[self.current_zone]['pedestrians_enabled']:
            pedestrian_markers = MarkerArray()
            
            for pedestrian in self.pedestrians:
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "pedestrians"
                marker.id = pedestrian['id']
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                
                # Position
                marker.pose.position.x = pedestrian['position'][0]
                marker.pose.position.y = pedestrian['position'][1]
                marker.pose.position.z = 0.9  # Hauteur du piéton
                
                # Orientation (direction de marche)
                quat = self._euler_to_quaternion(0, 0, pedestrian['direction'])
                marker.pose.orientation.x = quat[0]
                marker.pose.orientation.y = quat[1]
                marker.pose.orientation.z = quat[2]
                marker.pose.orientation.w = quat[3]
                
                # Taille
                marker.scale.x = 0.4  # Largeur
                marker.scale.y = 0.4  # Profondeur
                marker.scale.z = 1.8  # Hauteur
                
                # Couleur
                marker.color.r = pedestrian['color'][0]
                marker.color.g = pedestrian['color'][1]
                marker.color.b = pedestrian['color'][2]
                marker.color.a = pedestrian['color'][3]
                
                pedestrian_markers.markers.append(marker)
            
            self.pedestrian_pub.publish(pedestrian_markers)
        
        # Publier les obstacles
        if self.zones[self.current_zone]['obstacles_enabled']:
            obstacle_markers = MarkerArray()
            
            for obstacle in self.obstacles:
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "obstacles"
                marker.id = obstacle['id']
                
                # Type d'obstacle
                if obstacle['type'] == 'box':
                    marker.type = Marker.CUBE
                elif obstacle['type'] == 'cylinder':
                    marker.type = Marker.CYLINDER
                else:  # sphere
                    marker.type = Marker.SPHERE
                
                marker.action = Marker.ADD
                
                # Position
                marker.pose.position.x = obstacle['position'][0]
                marker.pose.position.y = obstacle['position'][1]
                
                if obstacle['type'] == 'box':
                    marker.pose.position.z = obstacle['size'][2] / 2
                elif obstacle['type'] == 'cylinder':
                    marker.pose.position.z = obstacle['size'][1] / 2
                else:  # sphere
                    marker.pose.position.z = obstacle['size']
                
                # Orientation (direction de mouvement)
                quat = self._euler_to_quaternion(0, 0, obstacle['direction'])
                marker.pose.orientation.x = quat[0]
                marker.pose.orientation.y = quat[1]
                marker.pose.orientation.z = quat[2]
                marker.pose.orientation.w = quat[3]
                
                # Taille
                if obstacle['type'] == 'box':
                    marker.scale.x = obstacle['size'][0]
                    marker.scale.y = obstacle['size'][1]
                    marker.scale.z = obstacle['size'][2]
                elif obstacle['type'] == 'cylinder':
                    marker.scale.x = obstacle['size'][0] * 2  # Diamètre
                    marker.scale.y = obstacle['size'][0] * 2  # Diamètre
                    marker.scale.z = obstacle['size'][1]      # Hauteur
                else:  # sphere
                    marker.scale.x = obstacle['size'] * 2  # Diamètre
                    marker.scale.y = obstacle['size'] * 2  # Diamètre
                    marker.scale.z = obstacle['size'] * 2  # Diamètre
                
                # Couleur
                marker.color.r = obstacle['color'][0]
                marker.color.g = obstacle['color'][1]
                marker.color.b = obstacle['color'][2]
                marker.color.a = obstacle['color'][3]
                
                obstacle_markers.markers.append(marker)
            
            self.obstacle_pub.publish(obstacle_markers)
    
    def _euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convertit des angles d'Euler en quaternion
        
        Args:
            roll (float): Angle de roulis en radians
            pitch (float): Angle de tangage en radians
            yaw (float): Angle de lacet en radians
            
        Returns:
            tuple: Quaternion (x, y, z, w)
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return (x, y, z, w)


def main(args=None):
    """Fonction principale pour exécuter le nœud de gestion de l'environnement"""
    rclpy.init(args=args)
    
    world_manager = WorldManagerNode()
    
    # Utiliser un exécuteur multi-thread pour gérer les callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(world_manager)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        world_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()