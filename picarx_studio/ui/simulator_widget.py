#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Widget de simulation 2D pour PiCarX Studio
"""

import math
import time
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QSlider
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QPointF, QRectF
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QPolygonF, QFont
from core.logger import LoggerMixin

class SimulatorWidget(QWidget, LoggerMixin):
    """Widget de simulation 2D du robot PiCarX"""
    
    # Signaux
    sensor_data_updated = pyqtSignal(dict)
    robot_position_changed = pyqtSignal(tuple)
    collision_detected = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        # Position et état du robot
        self.robot_x = 400  # Position X du robot (pixels)
        self.robot_y = 300  # Position Y du robot (pixels)
        self.robot_angle = 0  # Angle du robot (degrés)
        self.robot_speed = 0  # Vitesse actuelle
        self.robot_turn_speed = 0  # Vitesse de rotation
        
        # Paramètres du robot
        self.robot_width = 40
        self.robot_height = 60
        self.max_speed = 50
        self.max_turn_speed = 60
        
        # État de la simulation
        self.simulation_running = False
        self.show_sensors = True
        self.show_trail = True
        self.show_grid = True
        self.trail_points = []
        
        # Obstacles dans l'environnement - Carte améliorée
        self.obstacles = [
            # Murs périphériques
            QRectF(50, 50, 700, 20),      # Mur haut
            QRectF(50, 530, 700, 20),     # Mur bas
            QRectF(50, 50, 20, 500),      # Mur gauche
            QRectF(730, 50, 20, 500),     # Mur droit
            
            # Obstacles centraux - Labyrinthe simple
            QRectF(150, 150, 100, 20),    # Obstacle horizontal 1
            QRectF(350, 200, 20, 100),    # Obstacle vertical 1
            QRectF(500, 120, 80, 80),     # Carré central
            QRectF(200, 350, 150, 20),    # Obstacle horizontal 2
            QRectF(450, 400, 20, 80),     # Obstacle vertical 2
            
            # Zones d'intérêt
            QRectF(100, 250, 60, 60),     # Station de recharge
            QRectF(600, 300, 80, 40),     # Zone de collecte
            QRectF(300, 100, 40, 40),     # Point de contrôle 1
            QRectF(550, 450, 40, 40),     # Point de contrôle 2
        ]
        
        # Ligne à suivre améliorée - Circuit plus complexe
        self.line_path = [
            # Circuit principal
            QPointF(100, 500),
            QPointF(200, 500),
            QPointF(300, 450),
            QPointF(400, 400),
            QPointF(500, 350),
            QPointF(600, 300),
            QPointF(650, 250),
            QPointF(680, 200),
            QPointF(650, 150),
            QPointF(600, 120),
            QPointF(500, 100),
            QPointF(400, 120),
            QPointF(300, 150),
            QPointF(200, 200),
            QPointF(150, 250),
            QPointF(120, 300),
            QPointF(150, 350),
            QPointF(200, 400),
            QPointF(150, 450),
            QPointF(100, 500),  # Boucle fermée
        ]
        
        # Points d'intérêt sur la carte
        self.points_of_interest = [
            {'pos': QPointF(100, 250), 'type': 'charge', 'name': 'Station de recharge'},
            {'pos': QPointF(600, 300), 'type': 'collect', 'name': 'Zone de collecte'},
            {'pos': QPointF(300, 100), 'type': 'checkpoint', 'name': 'Point de contrôle A'},
            {'pos': QPointF(550, 450), 'type': 'checkpoint', 'name': 'Point de contrôle B'},
            {'pos': QPointF(400, 300), 'type': 'start', 'name': 'Position de départ'},
        ]
        
        self.init_ui()
        self.setup_simulation_timer()
        
    def init_ui(self):
        """Initialiser l'interface utilisateur"""
        layout = QVBoxLayout(self)
        
        # Panneau de contrôle rapide
        control_layout = QHBoxLayout()
        
        # Boutons de contrôle
        self.btn_forward = QPushButton("↑")
        self.btn_backward = QPushButton("↓")
        self.btn_left = QPushButton("←")
        self.btn_right = QPushButton("→")
        self.btn_stop = QPushButton("STOP")
        
        # Style des boutons
        button_style = """
            QPushButton {
                font-size: 16px;
                font-weight: bold;
                min-width: 40px;
                min-height: 40px;
                border: 2px solid #555;
                border-radius: 5px;
                background-color: #4a4a4a;
                color: white;
            }
            QPushButton:pressed {
                background-color: #666;
            }
            QPushButton:hover {
                background-color: #555;
            }
        """
        
        stop_button_style = button_style + """
            QPushButton {
                background-color: #cc4444;
                font-size: 12px;
            }
            QPushButton:pressed {
                background-color: #dd5555;
            }
        """
        
        for btn in [self.btn_forward, self.btn_backward, self.btn_left, self.btn_right]:
            btn.setStyleSheet(button_style)
        
        self.btn_stop.setStyleSheet(stop_button_style)
        
        # Connecter les boutons
        self.btn_forward.pressed.connect(lambda: self.set_robot_speed(self.max_speed, 0))
        self.btn_forward.released.connect(lambda: self.set_robot_speed(0, 0))
        self.btn_backward.pressed.connect(lambda: self.set_robot_speed(-self.max_speed, 0))
        self.btn_backward.released.connect(lambda: self.set_robot_speed(0, 0))
        self.btn_left.pressed.connect(lambda: self.set_robot_speed(0, -self.max_turn_speed))
        self.btn_left.released.connect(lambda: self.set_robot_speed(0, 0))
        self.btn_right.pressed.connect(lambda: self.set_robot_speed(0, self.max_turn_speed))
        self.btn_right.released.connect(lambda: self.set_robot_speed(0, 0))
        self.btn_stop.clicked.connect(lambda: self.set_robot_speed(0, 0))
        
        control_layout.addWidget(QLabel("Contrôle rapide:"))
        control_layout.addWidget(self.btn_forward)
        control_layout.addWidget(self.btn_backward)
        control_layout.addWidget(self.btn_left)
        control_layout.addWidget(self.btn_right)
        control_layout.addWidget(self.btn_stop)
        control_layout.addStretch()
        
        # Options d'affichage
        self.cb_sensors = QPushButton("Capteurs")
        self.cb_sensors.setCheckable(True)
        self.cb_sensors.setChecked(True)
        self.cb_sensors.clicked.connect(self.toggle_sensors)
        
        self.cb_trail = QPushButton("Trajectoire")
        self.cb_trail.setCheckable(True)
        self.cb_trail.setChecked(True)
        self.cb_trail.clicked.connect(self.toggle_trail)
        
        self.cb_grid = QPushButton("Grille")
        self.cb_grid.setCheckable(True)
        self.cb_grid.setChecked(True)
        self.cb_grid.clicked.connect(self.toggle_grid)
        
        for btn in [self.cb_sensors, self.cb_trail, self.cb_grid]:
            btn.setStyleSheet("""
                QPushButton {
                    font-size: 10px;
                    padding: 4px 8px;
                    border: 1px solid #555;
                    border-radius: 3px;
                    background-color: #3a3a3a;
                    color: white;
                }
                QPushButton:checked {
                    background-color: #0078d4;
                }
                QPushButton:hover {
                    background-color: #4a4a4a;
                }
            """)
        
        control_layout.addWidget(QLabel("Affichage:"))
        control_layout.addWidget(self.cb_sensors)
        control_layout.addWidget(self.cb_trail)
        control_layout.addWidget(self.cb_grid)
        
        layout.addLayout(control_layout)
        
        # Zone de dessin (sera gérée par paintEvent)
        self.setMinimumSize(800, 600)
        self.setStyleSheet("background-color: #1e1e1e; border: 1px solid #555;")
        
        # Permettre le focus pour les événements clavier
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        
    def setup_simulation_timer(self):
        """Configurer le timer de simulation"""
        self.simulation_timer = QTimer()
        self.simulation_timer.timeout.connect(self.update_simulation)
        self.simulation_timer.setInterval(50)  # 20 FPS
        
    def toggle_sensors(self):
        """Basculer l'affichage des capteurs"""
        self.show_sensors = self.cb_sensors.isChecked()
        self.update()
        
    def toggle_trail(self):
        """Basculer l'affichage de la trajectoire"""
        self.show_trail = self.cb_trail.isChecked()
        if not self.show_trail:
            self.trail_points.clear()
        self.update()
        
    def toggle_grid(self):
        """Basculer l'affichage de la grille"""
        self.show_grid = self.cb_grid.isChecked()
        self.update()
        
    def start_simulation(self):
        """Démarrer la simulation"""
        self.simulation_running = True
        self.simulation_timer.start()
        self.logger.info("Simulation du robot démarrée")
        
    def enable_movement(self):
        """Activer le mouvement du robot (même sans simulation active)"""
        if not self.simulation_timer.isActive():
            self.simulation_timer.start()
            self.logger.debug("Timer de mouvement activé")
        
    def stop_simulation(self):
        """Arrêter la simulation"""
        self.simulation_running = False
        self.simulation_timer.stop()
        self.robot_speed = 0
        self.robot_turn_speed = 0
        self.logger.info("Simulation du robot arrêtée")
        
    def reset_robot(self):
        """Réinitialiser la position du robot"""
        self.robot_x = 400
        self.robot_y = 300
        self.robot_angle = 0
        self.robot_speed = 0
        self.robot_turn_speed = 0
        self.trail_points.clear()
        self.update()
        self.robot_position_changed.emit((self.robot_x, self.robot_y, self.robot_angle))
        
    def set_robot_speed(self, speed, turn_speed):
        """Définir la vitesse du robot"""
        self.robot_speed = max(-self.max_speed, min(self.max_speed, speed))
        self.robot_turn_speed = max(-self.max_turn_speed, min(self.max_turn_speed, turn_speed))
        
        # Activer le mouvement si une vitesse est définie
        if abs(speed) > 0 or abs(turn_speed) > 0:
            self.enable_movement()
        
    def set_max_speed(self, speed):
        """Définir la vitesse maximale"""
        self.max_speed = speed
        
    def move_robot(self, direction):
        """Déplacer le robot dans une direction"""
        if direction == "forward":
            self.set_robot_speed(self.max_speed, 0)
        elif direction == "backward":
            self.set_robot_speed(-self.max_speed, 0)
        elif direction == "left":
            self.set_robot_speed(0, -self.max_turn_speed)
        elif direction == "right":
            self.set_robot_speed(0, self.max_turn_speed)
            
    def stop_robot(self):
        """Arrêter le robot"""
        self.set_robot_speed(0, 0)
        
    def keyPressEvent(self, event):
        """Gérer les événements clavier"""
        if event.key() == Qt.Key.Key_Up:
            self.set_robot_speed(self.max_speed, 0)
        elif event.key() == Qt.Key.Key_Down:
            self.set_robot_speed(-self.max_speed, 0)
        elif event.key() == Qt.Key.Key_Left:
            self.set_robot_speed(0, -self.max_turn_speed)
        elif event.key() == Qt.Key.Key_Right:
            self.set_robot_speed(0, self.max_turn_speed)
        elif event.key() == Qt.Key.Key_Space:
            self.set_robot_speed(0, 0)
        else:
            super().keyPressEvent(event)
            
    def keyReleaseEvent(self, event):
        """Gérer le relâchement des touches"""
        if event.key() in [Qt.Key.Key_Up, Qt.Key.Key_Down, Qt.Key.Key_Left, Qt.Key.Key_Right]:
            self.set_robot_speed(0, 0)
        else:
            super().keyReleaseEvent(event)
        
    def update_simulation(self):
        """Mettre à jour la simulation"""
        # Permettre le mouvement même si la simulation n'est pas "officiellement" démarrée
        # mais seulement si le robot a une vitesse
        if not self.simulation_running and abs(self.robot_speed) < 1 and abs(self.robot_turn_speed) < 1:
            return
            
        # Calculer le déplacement
        dt = 0.05  # 50ms
        
        # Rotation
        self.robot_angle += self.robot_turn_speed * dt
        self.robot_angle = self.robot_angle % 360
        
        # Translation
        angle_rad = math.radians(self.robot_angle)
        dx = self.robot_speed * math.cos(angle_rad) * dt
        dy = self.robot_speed * math.sin(angle_rad) * dt
        
        new_x = self.robot_x + dx
        new_y = self.robot_y + dy
        
        # Vérifier les collisions avec les bords
        margin = max(self.robot_width, self.robot_height) / 2
        if margin <= new_x <= self.width() - margin:
            self.robot_x = new_x
        else:
            self.collision_detected.emit("border")
            self.robot_speed = 0  # Arrêter le robot
            
        if margin <= new_y <= self.height() - margin:
            self.robot_y = new_y
        else:
            self.collision_detected.emit("border")
            self.robot_speed = 0  # Arrêter le robot
            
        # Vérifier les collisions avec les obstacles
        robot_rect = QRectF(
            self.robot_x - self.robot_width/2,
            self.robot_y - self.robot_height/2,
            self.robot_width,
            self.robot_height
        )
        
        for i, obstacle in enumerate(self.obstacles):
            if robot_rect.intersects(obstacle):
                self.collision_detected.emit(f"obstacle_{i}")
                # Reculer légèrement
                self.robot_x -= dx * 0.5
                self.robot_y -= dy * 0.5
                self.robot_speed = 0  # Arrêter le robot
                break
                
        # Ajouter au trail
        if self.show_trail and (abs(self.robot_speed) > 1 or abs(self.robot_turn_speed) > 1):
            self.trail_points.append(QPointF(self.robot_x, self.robot_y))
            if len(self.trail_points) > 300:  # Limiter la longueur du trail
                self.trail_points.pop(0)
                
        # Simuler les capteurs
        sensor_data = self.simulate_sensors()
        self.sensor_data_updated.emit(sensor_data)
        
        # Émettre la position
        self.robot_position_changed.emit((self.robot_x, self.robot_y, self.robot_angle))
        
        # Redessiner
        self.update()
        
    def simulate_sensors(self):
        """Simuler les données des capteurs"""
        # Capteur ultrasonique (distance devant)
        ultrasonic_distance = self.calculate_distance_to_obstacle()
        
        # Capteurs IR (détection de ligne)
        ir_left, ir_right = self.detect_line()
        
        # IMU simulé
        imu_data = {
            'angular_velocity': {'z': self.robot_turn_speed * 0.1},
            'linear_acceleration': {'x': self.robot_speed * 0.05}
        }
        
        # Batterie simulée (diminue lentement)
        battery_percentage = max(10, 100 - (time.time() % 3600) * 0.025)
        
        return {
            'ultrasonic': ultrasonic_distance,
            'ir_left': ir_left,
            'ir_right': ir_right,
            'imu': imu_data,
            'position': {
                'x': self.robot_x,
                'y': self.robot_y,
                'theta': self.robot_angle
            },
            'battery': {
                'percentage': battery_percentage,
                'voltage': 6.0 + (battery_percentage / 100) * 2.4
            }
        }
        
    def calculate_distance_to_obstacle(self):
        """Calculer la distance au prochain obstacle"""
        # Rayon de détection du capteur ultrasonique
        max_range = 200
        angle_rad = math.radians(self.robot_angle)
        
        # Point de départ (avant du robot)
        start_x = self.robot_x + (self.robot_height/2) * math.cos(angle_rad)
        start_y = self.robot_y + (self.robot_height/2) * math.sin(angle_rad)
        
        # Vérifier la distance aux obstacles
        min_distance = max_range
        
        for distance in range(1, max_range, 2):  # Pas de 2 pour optimiser
            check_x = start_x + distance * math.cos(angle_rad)
            check_y = start_y + distance * math.sin(angle_rad)
            
            # Vérifier les bords
            if check_x <= 0 or check_x >= self.width() or check_y <= 0 or check_y >= self.height():
                min_distance = min(min_distance, distance)
                break
                
            # Vérifier les obstacles
            check_point = QPointF(check_x, check_y)
            for obstacle in self.obstacles:
                if obstacle.contains(check_point):
                    min_distance = min(min_distance, distance)
                    break
            
            if min_distance < max_range:
                break
                    
        return min_distance
        
    def detect_line(self):
        """Détecter la ligne avec les capteurs IR"""
        # Positions des capteurs IR (gauche et droite du robot)
        angle_rad = math.radians(self.robot_angle)
        sensor_offset = 20
        
        # Capteur gauche
        left_x = self.robot_x - sensor_offset * math.sin(angle_rad)
        left_y = self.robot_y + sensor_offset * math.cos(angle_rad)
        
        # Capteur droit
        right_x = self.robot_x + sensor_offset * math.sin(angle_rad)
        right_y = self.robot_y - sensor_offset * math.cos(angle_rad)
        
        # Vérifier la proximité avec la ligne
        line_threshold = 25
        
        ir_left = self.is_near_line(left_x, left_y, line_threshold)
        ir_right = self.is_near_line(right_x, right_y, line_threshold)
        
        return ir_left, ir_right
        
    def is_near_line(self, x, y, threshold):
        """Vérifier si un point est proche de la ligne"""
        point = QPointF(x, y)
        
        for i in range(len(self.line_path) - 1):
            # Distance du point au segment de ligne
            p1 = self.line_path[i]
            p2 = self.line_path[i + 1]
            
            # Calcul de la distance point-segment
            A = x - p1.x()
            B = y - p1.y()
            C = p2.x() - p1.x()
            D = p2.y() - p1.y()
            
            dot = A * C + B * D
            len_sq = C * C + D * D
            
            if len_sq == 0:
                distance = math.sqrt(A * A + B * B)
            else:
                param = dot / len_sq
                if param < 0:
                    xx, yy = p1.x(), p1.y()
                elif param > 1:
                    xx, yy = p2.x(), p2.y()
                else:
                    xx = p1.x() + param * C
                    yy = p1.y() + param * D
                    
                dx = x - xx
                dy = y - yy
                distance = math.sqrt(dx * dx + dy * dy)
                
            if distance < threshold:
                return True
                
        return False
        
    def paintEvent(self, event):
        """Dessiner la simulation"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        # Fond
        painter.fillRect(self.rect(), QColor(25, 25, 25))
        
        # Grille
        if self.show_grid:
            self.draw_grid(painter)
        
        # Ligne à suivre
        self.draw_line_path(painter)
        
        # Obstacles
        self.draw_obstacles(painter)
        
        # Trail du robot
        if self.show_trail:
            self.draw_trail(painter)
            
        # Robot
        self.draw_robot(painter)
        
        # Capteurs
        if self.show_sensors:
            self.draw_sensors(painter)
            
        # Informations
        self.draw_info(painter)
        
    def draw_grid(self, painter):
        """Dessiner la grille de fond"""
        painter.setPen(QPen(QColor(50, 50, 50), 1))
        
        grid_size = 50
        for x in range(0, self.width(), grid_size):
            painter.drawLine(x, 0, x, self.height())
        for y in range(0, self.height(), grid_size):
            painter.drawLine(0, y, self.width(), y)
            
    def draw_line_path(self, painter):
        """Dessiner la ligne à suivre"""
        painter.setPen(QPen(QColor(255, 255, 0), 6))
        
        for i in range(len(self.line_path) - 1):
            painter.drawLine(self.line_path[i], self.line_path[i + 1])
            
        # Dessiner les points de contrôle
        painter.setBrush(QBrush(QColor(255, 255, 0)))
        for point in self.line_path:
            painter.drawEllipse(point, 3, 3)
            
    def draw_obstacles(self, painter):
        """Dessiner les obstacles avec différents styles"""
        for i, obstacle in enumerate(self.obstacles):
            if i < 4:  # Murs périphériques
                painter.setBrush(QBrush(QColor(100, 100, 100)))
                painter.setPen(QPen(QColor(150, 150, 150), 3))
            elif i < 9:  # Obstacles centraux
                painter.setBrush(QBrush(QColor(180, 60, 60)))
                painter.setPen(QPen(QColor(220, 100, 100), 2))
            else:  # Zones d'intérêt
                if i == 9:  # Station de recharge
                    painter.setBrush(QBrush(QColor(60, 180, 60)))
                    painter.setPen(QPen(QColor(100, 220, 100), 2))
                elif i == 10:  # Zone de collecte
                    painter.setBrush(QBrush(QColor(60, 60, 180)))
                    painter.setPen(QPen(QColor(100, 100, 220), 2))
                else:  # Points de contrôle
                    painter.setBrush(QBrush(QColor(180, 180, 60)))
                    painter.setPen(QPen(QColor(220, 220, 100), 2))
            
            painter.drawRect(obstacle)
            
        # Dessiner les points d'intérêt avec des icônes
        self.draw_points_of_interest(painter)
            
    def draw_points_of_interest(self, painter):
        """Dessiner les points d'intérêt avec des icônes"""
        painter.setFont(QFont("Arial", 8, QFont.Weight.Bold))
        
        for poi in self.points_of_interest:
            pos = poi['pos']
            poi_type = poi['type']
            name = poi['name']
            
            # Couleur selon le type
            if poi_type == 'charge':
                color = QColor(60, 255, 60)
                icon = "🔋"
            elif poi_type == 'collect':
                color = QColor(60, 60, 255)
                icon = "📦"
            elif poi_type == 'checkpoint':
                color = QColor(255, 255, 60)
                icon = "🏁"
            elif poi_type == 'start':
                color = QColor(255, 100, 100)
                icon = "🏠"
            else:
                color = QColor(200, 200, 200)
                icon = "?"
            
            # Dessiner l'icône
            painter.setPen(QPen(color, 2))
            painter.setBrush(QBrush(color.darker()))
            painter.drawEllipse(pos, 15, 15)
            
            # Dessiner le texte de l'icône
            painter.setPen(QPen(QColor(255, 255, 255)))
            painter.drawText(pos.x() - 5, pos.y() + 3, icon)
            
            # Dessiner le nom si on est proche
            robot_pos = QPointF(self.robot_x, self.robot_y)
            distance = ((pos.x() - robot_pos.x())**2 + (pos.y() - robot_pos.y())**2)**0.5
            if distance < 50:
                painter.setPen(QPen(color))
                painter.drawText(pos.x() + 20, pos.y() - 10, name)
            
    def draw_trail(self, painter):
        """Dessiner le trail du robot"""
        if len(self.trail_points) < 2:
            return
            
        # Gradient de couleur pour le trail
        for i in range(1, len(self.trail_points)):
            alpha = int(255 * (i / len(self.trail_points)))
            painter.setPen(QPen(QColor(100, 200, 255, alpha), 3))
            painter.drawLine(self.trail_points[i-1], self.trail_points[i])
            
    def draw_robot(self, painter):
        """Dessiner le robot"""
        painter.save()
        painter.translate(self.robot_x, self.robot_y)
        painter.rotate(self.robot_angle)
        
        # Corps du robot
        robot_rect = QRectF(-self.robot_width/2, -self.robot_height/2, 
                           self.robot_width, self.robot_height)
        
        # Gradient pour le corps
        painter.setBrush(QBrush(QColor(100, 150, 255)))
        painter.setPen(QPen(QColor(150, 200, 255), 2))
        painter.drawRoundedRect(robot_rect, 5, 5)
        
        # Direction (flèche)
        arrow = QPolygonF([
            QPointF(0, -self.robot_height/2 - 5),
            QPointF(-8, -self.robot_height/2 + 10),
            QPointF(8, -self.robot_height/2 + 10)
        ])
        
        painter.setBrush(QBrush(QColor(255, 200, 100)))
        painter.setPen(QPen(QColor(255, 220, 120), 1))
        painter.drawPolygon(arrow)
        
        # Roues
        wheel_color = QColor(60, 60, 60)
        painter.setBrush(QBrush(wheel_color))
        painter.setPen(QPen(QColor(80, 80, 80), 1))
        
        # Roues avant
        painter.drawRect(-self.robot_width/2 - 5, -self.robot_height/2 + 5, 8, 15)
        painter.drawRect(self.robot_width/2 - 3, -self.robot_height/2 + 5, 8, 15)
        
        # Roues arrière
        painter.drawRect(-self.robot_width/2 - 5, self.robot_height/2 - 20, 8, 15)
        painter.drawRect(self.robot_width/2 - 3, self.robot_height/2 - 20, 8, 15)
        
        painter.restore()
        
    def draw_sensors(self, painter):
        """Dessiner les capteurs et leurs données"""
        # Capteur ultrasonique
        distance = self.calculate_distance_to_obstacle()
        angle_rad = math.radians(self.robot_angle)
        
        start_x = self.robot_x + (self.robot_height/2) * math.cos(angle_rad)
        start_y = self.robot_y + (self.robot_height/2) * math.sin(angle_rad)
        
        end_x = start_x + distance * math.cos(angle_rad)
        end_y = start_y + distance * math.sin(angle_rad)
        
        # Couleur selon la distance
        if distance < 30:
            color = QColor(255, 100, 100, 180)
        elif distance < 80:
            color = QColor(255, 200, 100, 150)
        else:
            color = QColor(100, 255, 100, 120)
            
        painter.setPen(QPen(color, 3))
        painter.drawLine(start_x, start_y, end_x, end_y)
        
        # Point d'impact
        painter.setBrush(QBrush(color))
        painter.drawEllipse(QPointF(end_x, end_y), 4, 4)
        
        # Capteurs IR
        sensor_offset = 20
        left_x = self.robot_x - sensor_offset * math.sin(angle_rad)
        left_y = self.robot_y + sensor_offset * math.cos(angle_rad)
        right_x = self.robot_x + sensor_offset * math.sin(angle_rad)
        right_y = self.robot_y - sensor_offset * math.cos(angle_rad)
        
        ir_left, ir_right = self.detect_line()
        
        # Capteur gauche
        color = QColor(0, 255, 0) if ir_left else QColor(255, 100, 100)
        painter.setBrush(QBrush(color))
        painter.setPen(QPen(color.darker(), 2))
        painter.drawEllipse(QPointF(left_x, left_y), 6, 6)
        
        # Capteur droit
        color = QColor(0, 255, 0) if ir_right else QColor(255, 100, 100)
        painter.setBrush(QBrush(color))
        painter.setPen(QPen(color.darker(), 2))
        painter.drawEllipse(QPointF(right_x, right_y), 6, 6)
        
    def draw_info(self, painter):
        """Dessiner les informations de debug"""
        painter.setPen(QPen(QColor(255, 255, 255)))
        painter.setFont(QFont("Arial", 9))
        
        info_text = [
            f"Position: ({self.robot_x:.0f}, {self.robot_y:.0f})",
            f"Angle: {self.robot_angle:.0f}°",
            f"Vitesse: {self.robot_speed:.0f}",
            f"Rotation: {self.robot_turn_speed:.0f}",
            f"Distance: {self.calculate_distance_to_obstacle():.0f} px",
        ]
        
        # Fond semi-transparent pour le texte
        painter.setBrush(QBrush(QColor(0, 0, 0, 150)))
        painter.setPen(QPen(QColor(100, 100, 100)))
        painter.drawRect(5, 5, 180, len(info_text) * 16 + 10)
        
        # Texte
        painter.setPen(QPen(QColor(255, 255, 255)))
        y_offset = 20
        for i, text in enumerate(info_text):
            painter.drawText(10, y_offset + i * 16, text)
            
        # Instructions de contrôle
        if not self.simulation_running:
            painter.setPen(QPen(QColor(200, 200, 200)))
            painter.setFont(QFont("Arial", 8))
            instructions = [
                "Contrôles clavier:",
                "↑↓←→ : Déplacer",
                "Espace : Arrêter",
                "Clic : Focus requis"
            ]
            
            painter.setBrush(QBrush(QColor(0, 0, 0, 150)))
            painter.setPen(QPen(QColor(100, 100, 100)))
            painter.drawRect(self.width() - 140, 5, 135, len(instructions) * 14 + 10)
            
            painter.setPen(QPen(QColor(200, 200, 200)))
            for i, instruction in enumerate(instructions):
                painter.drawText(self.width() - 135, 20 + i * 14, instruction)