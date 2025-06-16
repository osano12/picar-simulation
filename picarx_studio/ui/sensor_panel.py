#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Panneau d'affichage des capteurs pour PiCarX Studio
"""

from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                            QLabel, QProgressBar, QGroupBox, QLCDNumber,
                            QFrame, QCheckBox)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QFont, QColor, QPalette

class SensorPanel(QWidget):
    """Panneau d'affichage des données des capteurs"""
    
    def __init__(self):
        super().__init__()
        self.sensor_data = {}
        self.init_ui()
        
    def init_ui(self):
        """Initialiser l'interface utilisateur"""
        layout = QVBoxLayout(self)
        
        # Groupe capteur ultrasonique
        ultrasonic_group = self.create_ultrasonic_group()
        layout.addWidget(ultrasonic_group)
        
        # Groupe capteurs IR
        ir_group = self.create_ir_group()
        layout.addWidget(ir_group)
        
        # Groupe IMU
        imu_group = self.create_imu_group()
        layout.addWidget(imu_group)
        
        # Groupe position
        position_group = self.create_position_group()
        layout.addWidget(position_group)
        
        # Groupe batterie
        battery_group = self.create_battery_group()
        layout.addWidget(battery_group)
        
        layout.addStretch()
        
    def create_ultrasonic_group(self):
        """Créer le groupe du capteur ultrasonique"""
        group = QGroupBox("Capteur Ultrasonique")
        layout = QVBoxLayout(group)
        
        # Affichage LCD de la distance
        self.ultrasonic_lcd = QLCDNumber(4)
        self.ultrasonic_lcd.setStyleSheet("""
            QLCDNumber {
                background-color: #2a2a2a;
                color: #00ff00;
                border: 2px solid #555;
                border-radius: 5px;
            }
        """)
        self.ultrasonic_lcd.display("0.0")
        layout.addWidget(self.ultrasonic_lcd)
        
        # Label d'unité
        unit_label = QLabel("cm")
        unit_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        unit_label.setStyleSheet("font-weight: bold; color: #888;")
        layout.addWidget(unit_label)
        
        # Barre de progression visuelle
        self.ultrasonic_bar = QProgressBar()
        self.ultrasonic_bar.setRange(0, 200)  # 0-200 cm
        self.ultrasonic_bar.setValue(0)
        self.ultrasonic_bar.setStyleSheet("""
            QProgressBar {
                border: 2px solid #555;
                border-radius: 5px;
                text-align: center;
                background-color: #2a2a2a;
            }
            QProgressBar::chunk {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #ff0000, stop:0.3 #ffff00, stop:1 #00ff00);
                border-radius: 3px;
            }
        """)
        layout.addWidget(self.ultrasonic_bar)
        
        # Indicateur de proximité
        self.proximity_label = QLabel("Distance normale")
        self.proximity_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.proximity_label.setStyleSheet("font-size: 10px; color: #00ff00;")
        layout.addWidget(self.proximity_label)
        
        return group
        
    def create_ir_group(self):
        """Créer le groupe des capteurs IR"""
        group = QGroupBox("Capteurs Infrarouges")
        layout = QGridLayout(group)
        
        # Capteur IR gauche
        layout.addWidget(QLabel("Gauche:"), 0, 0)
        self.ir_left_indicator = QFrame()
        self.ir_left_indicator.setFixedSize(30, 30)
        self.ir_left_indicator.setStyleSheet("""
            QFrame {
                border: 2px solid #555;
                border-radius: 15px;
                background-color: #ff0000;
            }
        """)
        layout.addWidget(self.ir_left_indicator, 0, 1)
        
        self.ir_left_label = QLabel("Non détecté")
        self.ir_left_label.setStyleSheet("font-size: 10px; color: #888;")
        layout.addWidget(self.ir_left_label, 0, 2)
        
        # Capteur IR droit
        layout.addWidget(QLabel("Droit:"), 1, 0)
        self.ir_right_indicator = QFrame()
        self.ir_right_indicator.setFixedSize(30, 30)
        self.ir_right_indicator.setStyleSheet("""
            QFrame {
                border: 2px solid #555;
                border-radius: 15px;
                background-color: #ff0000;
            }
        """)
        layout.addWidget(self.ir_right_indicator, 1, 1)
        
        self.ir_right_label = QLabel("Non détecté")
        self.ir_right_label.setStyleSheet("font-size: 10px; color: #888;")
        layout.addWidget(self.ir_right_label, 1, 2)
        
        # Indicateur de suivi de ligne
        separator = QFrame()
        separator.setFrameShape(QFrame.Shape.HLine)
        separator.setStyleSheet("color: #555;")
        layout.addWidget(separator, 2, 0, 1, 3)
        
        self.line_status_label = QLabel("Ligne: Non détectée")
        self.line_status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.line_status_label.setStyleSheet("font-weight: bold; color: #ff8800;")
        layout.addWidget(self.line_status_label, 3, 0, 1, 3)
        
        return group
        
    def create_imu_group(self):
        """Créer le groupe IMU"""
        group = QGroupBox("IMU (Inertial Measurement Unit)")
        layout = QGridLayout(group)
        
        # Vitesse angulaire
        layout.addWidget(QLabel("Vitesse angulaire:"), 0, 0)
        self.angular_velocity_label = QLabel("0.0 °/s")
        self.angular_velocity_label.setStyleSheet("font-family: monospace; color: #00aaff;")
        layout.addWidget(self.angular_velocity_label, 0, 1)
        
        # Accélération linéaire
        layout.addWidget(QLabel("Accélération:"), 1, 0)
        self.linear_acceleration_label = QLabel("0.0 m/s²")
        self.linear_acceleration_label.setStyleSheet("font-family: monospace; color: #00aaff;")
        layout.addWidget(self.linear_acceleration_label, 1, 1)
        
        # Barres de progression pour les valeurs
        self.angular_velocity_bar = QProgressBar()
        self.angular_velocity_bar.setRange(-100, 100)
        self.angular_velocity_bar.setValue(0)
        self.angular_velocity_bar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #555;
                border-radius: 3px;
                text-align: center;
                background-color: #2a2a2a;
            }
            QProgressBar::chunk {
                background-color: #00aaff;
                border-radius: 2px;
            }
        """)
        layout.addWidget(self.angular_velocity_bar, 2, 0, 1, 2)
        
        self.linear_acceleration_bar = QProgressBar()
        self.linear_acceleration_bar.setRange(-50, 50)
        self.linear_acceleration_bar.setValue(0)
        self.linear_acceleration_bar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #555;
                border-radius: 3px;
                text-align: center;
                background-color: #2a2a2a;
            }
            QProgressBar::chunk {
                background-color: #00ff88;
                border-radius: 2px;
            }
        """)
        layout.addWidget(self.linear_acceleration_bar, 3, 0, 1, 2)
        
        return group
        
    def create_position_group(self):
        """Créer le groupe de position"""
        group = QGroupBox("Position et Orientation")
        layout = QGridLayout(group)
        
        # Position X
        layout.addWidget(QLabel("X:"), 0, 0)
        self.position_x_label = QLabel("0.0")
        self.position_x_label.setStyleSheet("font-family: monospace; color: #ffaa00;")
        layout.addWidget(self.position_x_label, 0, 1)
        layout.addWidget(QLabel("pixels"), 0, 2)
        
        # Position Y
        layout.addWidget(QLabel("Y:"), 1, 0)
        self.position_y_label = QLabel("0.0")
        self.position_y_label.setStyleSheet("font-family: monospace; color: #ffaa00;")
        layout.addWidget(self.position_y_label, 1, 1)
        layout.addWidget(QLabel("pixels"), 1, 2)
        
        # Orientation
        layout.addWidget(QLabel("Angle:"), 2, 0)
        self.orientation_label = QLabel("0.0")
        self.orientation_label.setStyleSheet("font-family: monospace; color: #ffaa00;")
        layout.addWidget(self.orientation_label, 2, 1)
        layout.addWidget(QLabel("degrés"), 2, 2)
        
        # Boussole visuelle (simple)
        self.compass_label = QLabel("↑")
        self.compass_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.compass_label.setStyleSheet("""
            font-size: 24px;
            font-weight: bold;
            color: #ff6600;
            border: 2px solid #555;
            border-radius: 20px;
            min-width: 40px;
            min-height: 40px;
        """)
        layout.addWidget(self.compass_label, 0, 3, 3, 1)
        
        return group
        
    def create_battery_group(self):
        """Créer le groupe batterie"""
        group = QGroupBox("Batterie (Simulée)")
        layout = QVBoxLayout(group)
        
        # Niveau de batterie
        battery_layout = QHBoxLayout()
        battery_layout.addWidget(QLabel("Niveau:"))
        
        self.battery_bar = QProgressBar()
        self.battery_bar.setRange(0, 100)
        self.battery_bar.setValue(85)
        self.battery_bar.setStyleSheet("""
            QProgressBar {
                border: 2px solid #555;
                border-radius: 5px;
                text-align: center;
                background-color: #2a2a2a;
            }
            QProgressBar::chunk {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #ff0000, stop:0.2 #ff8800, stop:0.5 #ffff00, stop:1 #00ff00);
                border-radius: 3px;
            }
        """)
        battery_layout.addWidget(self.battery_bar)
        
        self.battery_percentage_label = QLabel("85%")
        self.battery_percentage_label.setStyleSheet("font-weight: bold; color: #00ff00;")
        battery_layout.addWidget(self.battery_percentage_label)
        
        layout.addLayout(battery_layout)
        
        # Tension
        voltage_layout = QHBoxLayout()
        voltage_layout.addWidget(QLabel("Tension:"))
        self.voltage_label = QLabel("7.4 V")
        self.voltage_label.setStyleSheet("font-family: monospace; color: #00aaff;")
        voltage_layout.addWidget(self.voltage_label)
        voltage_layout.addStretch()
        
        layout.addLayout(voltage_layout)
        
        # État de charge
        self.charging_label = QLabel("Décharge normale")
        self.charging_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.charging_label.setStyleSheet("font-size: 10px; color: #888;")
        layout.addWidget(self.charging_label)
        
        return group
        
    def update_sensors(self, sensor_data):
        """Mettre à jour l'affichage des capteurs"""
        self.sensor_data = sensor_data
        
        # Capteur ultrasonique
        if 'ultrasonic' in sensor_data:
            distance = sensor_data['ultrasonic']
            self.ultrasonic_lcd.display(f"{distance:.1f}")
            self.ultrasonic_bar.setValue(int(distance))
            
            # Indicateur de proximité
            if distance < 20:
                self.proximity_label.setText("TRÈS PROCHE!")
                self.proximity_label.setStyleSheet("font-size: 10px; color: #ff0000; font-weight: bold;")
            elif distance < 50:
                self.proximity_label.setText("Proche")
                self.proximity_label.setStyleSheet("font-size: 10px; color: #ff8800;")
            else:
                self.proximity_label.setText("Distance normale")
                self.proximity_label.setStyleSheet("font-size: 10px; color: #00ff00;")
                
        # Capteurs IR
        if 'ir_left' in sensor_data:
            ir_left = sensor_data['ir_left']
            if ir_left:
                self.ir_left_indicator.setStyleSheet("""
                    QFrame {
                        border: 2px solid #555;
                        border-radius: 15px;
                        background-color: #00ff00;
                    }
                """)
                self.ir_left_label.setText("Détecté")
                self.ir_left_label.setStyleSheet("font-size: 10px; color: #00ff00;")
            else:
                self.ir_left_indicator.setStyleSheet("""
                    QFrame {
                        border: 2px solid #555;
                        border-radius: 15px;
                        background-color: #ff0000;
                    }
                """)
                self.ir_left_label.setText("Non détecté")
                self.ir_left_label.setStyleSheet("font-size: 10px; color: #888;")
                
        if 'ir_right' in sensor_data:
            ir_right = sensor_data['ir_right']
            if ir_right:
                self.ir_right_indicator.setStyleSheet("""
                    QFrame {
                        border: 2px solid #555;
                        border-radius: 15px;
                        background-color: #00ff00;
                    }
                """)
                self.ir_right_label.setText("Détecté")
                self.ir_right_label.setStyleSheet("font-size: 10px; color: #00ff00;")
            else:
                self.ir_right_indicator.setStyleSheet("""
                    QFrame {
                        border: 2px solid #555;
                        border-radius: 15px;
                        background-color: #ff0000;
                    }
                """)
                self.ir_right_label.setText("Non détecté")
                self.ir_right_label.setStyleSheet("font-size: 10px; color: #888;")
                
        # État de suivi de ligne
        if 'ir_left' in sensor_data and 'ir_right' in sensor_data:
            ir_left = sensor_data['ir_left']
            ir_right = sensor_data['ir_right']
            
            if ir_left and ir_right:
                self.line_status_label.setText("Ligne: Sur la ligne")
                self.line_status_label.setStyleSheet("font-weight: bold; color: #00ff00;")
            elif ir_left:
                self.line_status_label.setText("Ligne: Décalage à droite")
                self.line_status_label.setStyleSheet("font-weight: bold; color: #ffaa00;")
            elif ir_right:
                self.line_status_label.setText("Ligne: Décalage à gauche")
                self.line_status_label.setStyleSheet("font-weight: bold; color: #ffaa00;")
            else:
                self.line_status_label.setText("Ligne: Non détectée")
                self.line_status_label.setStyleSheet("font-weight: bold; color: #ff8800;")
                
        # IMU
        if 'imu' in sensor_data:
            imu_data = sensor_data['imu']
            
            if 'angular_velocity' in imu_data:
                angular_z = imu_data['angular_velocity'].get('z', 0)
                self.angular_velocity_label.setText(f"{angular_z:.2f} °/s")
                self.angular_velocity_bar.setValue(int(angular_z))
                
            if 'linear_acceleration' in imu_data:
                accel_x = imu_data['linear_acceleration'].get('x', 0)
                self.linear_acceleration_label.setText(f"{accel_x:.2f} m/s²")
                self.linear_acceleration_bar.setValue(int(accel_x * 10))
                
        # Position
        if 'position' in sensor_data:
            position = sensor_data['position']
            self.position_x_label.setText(f"{position.get('x', 0):.1f}")
            self.position_y_label.setText(f"{position.get('y', 0):.1f}")
            
            angle = position.get('theta', 0)
            self.orientation_label.setText(f"{angle:.1f}")
            
            # Mettre à jour la boussole
            self.update_compass(angle)
            
        # Batterie (simulée)
        if 'battery' in sensor_data:
            battery = sensor_data['battery']
            percentage = battery.get('percentage', 85)
            voltage = battery.get('voltage', 7.4)
            
            self.battery_bar.setValue(int(percentage))
            self.battery_percentage_label.setText(f"{percentage:.0f}%")
            self.voltage_label.setText(f"{voltage:.1f} V")
            
            # Couleur selon le niveau
            if percentage > 50:
                self.battery_percentage_label.setStyleSheet("font-weight: bold; color: #00ff00;")
            elif percentage > 20:
                self.battery_percentage_label.setStyleSheet("font-weight: bold; color: #ffaa00;")
            else:
                self.battery_percentage_label.setStyleSheet("font-weight: bold; color: #ff0000;")
                
    def update_compass(self, angle):
        """Mettre à jour la boussole visuelle"""
        # Normaliser l'angle
        angle = angle % 360
        
        # Déterminer la direction principale
        if -22.5 <= angle <= 22.5 or angle >= 337.5:
            direction = "↑"  # Nord
        elif 22.5 < angle <= 67.5:
            direction = "↗"  # Nord-Est
        elif 67.5 < angle <= 112.5:
            direction = "→"  # Est
        elif 112.5 < angle <= 157.5:
            direction = "↘"  # Sud-Est
        elif 157.5 < angle <= 202.5:
            direction = "↓"  # Sud
        elif 202.5 < angle <= 247.5:
            direction = "↙"  # Sud-Ouest
        elif 247.5 < angle <= 292.5:
            direction = "←"  # Ouest
        else:  # 292.5 < angle < 337.5
            direction = "↖"  # Nord-Ouest
            
        self.compass_label.setText(direction)