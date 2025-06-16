#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Panneau de contrôle pour PiCarX Studio
"""

from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                            QPushButton, QLabel, QSlider, QSpinBox, QGroupBox,
                            QCheckBox, QComboBox, QProgressBar)
from PyQt6.QtCore import Qt, pyqtSignal, QTimer
from PyQt6.QtGui import QFont, QIcon

class ControlPanel(QWidget):
    """Panneau de contrôle du robot"""
    
    # Signaux
    move_command = pyqtSignal(str)  # forward, backward, left, right
    stop_command = pyqtSignal()
    speed_changed = pyqtSignal(int)
    mode_changed = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.current_mode = "manual"
        self.auto_mode_active = False
        self.init_ui()
        
    def init_ui(self):
        """Initialiser l'interface utilisateur"""
        layout = QVBoxLayout(self)
        
        # Groupe de contrôle manuel
        manual_group = self.create_manual_control_group()
        layout.addWidget(manual_group)
        
        # Groupe de paramètres
        params_group = self.create_parameters_group()
        layout.addWidget(params_group)
        
        # Groupe de modes automatiques
        auto_group = self.create_auto_modes_group()
        layout.addWidget(auto_group)
        
        # Groupe de statut
        status_group = self.create_status_group()
        layout.addWidget(status_group)
        
        layout.addStretch()
        
    def create_manual_control_group(self):
        """Créer le groupe de contrôle manuel"""
        group = QGroupBox("🎮 Contrôle Manuel")
        layout = QVBoxLayout(group)
        
        # Boutons directionnels
        button_layout = QGridLayout()
        
        # Style des boutons
        button_style = """
            QPushButton {
                font-size: 18px;
                font-weight: bold;
                min-width: 50px;
                min-height: 50px;
                border: 2px solid #555;
                border-radius: 8px;
                background-color: #4a4a4a;
                color: white;
            }
            QPushButton:pressed {
                background-color: #666;
                border-color: #777;
            }
            QPushButton:hover {
                background-color: #555;
            }
        """
        
        # Bouton avant
        self.btn_forward = QPushButton("↑")
        self.btn_forward.setStyleSheet(button_style)
        self.btn_forward.pressed.connect(lambda: self.move_command.emit("forward"))
        self.btn_forward.released.connect(self.stop_command.emit)
        button_layout.addWidget(self.btn_forward, 0, 1)
        
        # Boutons gauche et droite
        self.btn_left = QPushButton("←")
        self.btn_left.setStyleSheet(button_style)
        self.btn_left.pressed.connect(lambda: self.move_command.emit("left"))
        self.btn_left.released.connect(self.stop_command.emit)
        button_layout.addWidget(self.btn_left, 1, 0)
        
        self.btn_right = QPushButton("→")
        self.btn_right.setStyleSheet(button_style)
        self.btn_right.pressed.connect(lambda: self.move_command.emit("right"))
        self.btn_right.released.connect(self.stop_command.emit)
        button_layout.addWidget(self.btn_right, 1, 2)
        
        # Bouton arrière
        self.btn_backward = QPushButton("↓")
        self.btn_backward.setStyleSheet(button_style)
        self.btn_backward.pressed.connect(lambda: self.move_command.emit("backward"))
        self.btn_backward.released.connect(self.stop_command.emit)
        button_layout.addWidget(self.btn_backward, 2, 1)
        
        # Bouton STOP au centre
        self.btn_stop = QPushButton("STOP")
        self.btn_stop.setStyleSheet(button_style + """
            QPushButton {
                background-color: #cc4444;
                font-size: 12px;
            }
            QPushButton:pressed {
                background-color: #dd5555;
            }
        """)
        self.btn_stop.clicked.connect(self.stop_command.emit)
        button_layout.addWidget(self.btn_stop, 1, 1)
        
        layout.addLayout(button_layout)
        
        # Instructions
        instructions = QLabel("💡 Utilisez aussi les flèches du clavier")
        instructions.setStyleSheet("font-size: 9px; color: #888; font-style: italic;")
        instructions.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(instructions)
        
        return group
        
    def create_parameters_group(self):
        """Créer le groupe de paramètres"""
        group = QGroupBox("⚙️ Paramètres")
        layout = QVBoxLayout(group)
        
        # Vitesse
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("Vitesse:"))
        
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(10, 100)
        self.speed_slider.setValue(50)
        self.speed_slider.valueChanged.connect(self.on_speed_changed)
        self.speed_slider.setStyleSheet("""
            QSlider::groove:horizontal {
                border: 1px solid #555;
                height: 8px;
                background: #2a2a2a;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: #0078d4;
                border: 1px solid #555;
                width: 18px;
                margin: -5px 0;
                border-radius: 9px;
            }
            QSlider::handle:horizontal:hover {
                background: #106ebe;
            }
        """)
        speed_layout.addWidget(self.speed_slider)
        
        self.speed_label = QLabel("50%")
        self.speed_label.setMinimumWidth(40)
        self.speed_label.setStyleSheet("font-weight: bold; color: #0078d4;")
        speed_layout.addWidget(self.speed_label)
        
        layout.addLayout(speed_layout)
        
        # Vitesse de rotation
        turn_layout = QHBoxLayout()
        turn_layout.addWidget(QLabel("Rotation:"))
        
        self.turn_slider = QSlider(Qt.Orientation.Horizontal)
        self.turn_slider.setRange(10, 100)
        self.turn_slider.setValue(60)
        self.turn_slider.valueChanged.connect(self.on_turn_speed_changed)
        self.turn_slider.setStyleSheet(self.speed_slider.styleSheet())
        turn_layout.addWidget(self.turn_slider)
        
        self.turn_label = QLabel("60%")
        self.turn_label.setMinimumWidth(40)
        self.turn_label.setStyleSheet("font-weight: bold; color: #0078d4;")
        turn_layout.addWidget(self.turn_label)
        
        layout.addLayout(turn_layout)
        
        # Sensibilité des capteurs
        sensor_layout = QHBoxLayout()
        sensor_layout.addWidget(QLabel("Sensibilité IR:"))
        
        self.sensor_slider = QSlider(Qt.Orientation.Horizontal)
        self.sensor_slider.setRange(1, 10)
        self.sensor_slider.setValue(5)
        self.sensor_slider.setStyleSheet(self.speed_slider.styleSheet())
        sensor_layout.addWidget(self.sensor_slider)
        
        sensor_value_label = QLabel("5")
        sensor_value_label.setMinimumWidth(20)
        sensor_value_label.setStyleSheet("font-weight: bold; color: #0078d4;")
        self.sensor_slider.valueChanged.connect(lambda v: sensor_value_label.setText(str(v)))
        sensor_layout.addWidget(sensor_value_label)
        
        layout.addLayout(sensor_layout)
        
        return group
        
    def create_auto_modes_group(self):
        """Créer le groupe des modes automatiques"""
        group = QGroupBox("🤖 Modes Automatiques")
        layout = QVBoxLayout(group)
        
        # Sélecteur de mode
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("Mode:"))
        
        self.mode_combo = QComboBox()
        self.mode_combo.addItems([
            "Manuel",
            "Suivi de ligne",
            "Évitement d'obstacles",
            "Exploration",
            "Retour à la base"
        ])
        self.mode_combo.setStyleSheet("""
            QComboBox {
                padding: 5px;
                border: 1px solid #555;
                border-radius: 3px;
                background-color: #3a3a3a;
                color: white;
            }
            QComboBox::drop-down {
                border: none;
            }
            QComboBox::down-arrow {
                image: none;
                border-left: 5px solid transparent;
                border-right: 5px solid transparent;
                border-top: 5px solid #888;
            }
            QComboBox QAbstractItemView {
                background-color: #3a3a3a;
                color: white;
                selection-background-color: #0078d4;
            }
        """)
        self.mode_combo.currentTextChanged.connect(self.on_mode_changed)
        mode_layout.addWidget(self.mode_combo)
        
        layout.addLayout(mode_layout)
        
        # Boutons de contrôle automatique
        auto_buttons_layout = QHBoxLayout()
        
        self.btn_start_auto = QPushButton("▶️ Démarrer")
        self.btn_start_auto.setStyleSheet("""
            QPushButton {
                background-color: #44aa44;
                color: white;
                font-weight: bold;
                padding: 8px 12px;
                border: none;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #55bb55;
            }
            QPushButton:pressed {
                background-color: #339933;
            }
            QPushButton:disabled {
                background-color: #666;
                color: #aaa;
            }
        """)
        self.btn_start_auto.clicked.connect(self.start_auto_mode)
        auto_buttons_layout.addWidget(self.btn_start_auto)
        
        self.btn_stop_auto = QPushButton("⏹️ Arrêter")
        self.btn_stop_auto.setStyleSheet("""
            QPushButton {
                background-color: #aa4444;
                color: white;
                font-weight: bold;
                padding: 8px 12px;
                border: none;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #bb5555;
            }
            QPushButton:pressed {
                background-color: #993333;
            }
            QPushButton:disabled {
                background-color: #666;
                color: #aaa;
            }
        """)
        self.btn_stop_auto.clicked.connect(self.stop_auto_mode)
        self.btn_stop_auto.setEnabled(False)
        auto_buttons_layout.addWidget(self.btn_stop_auto)
        
        layout.addLayout(auto_buttons_layout)
        
        # Paramètres spécifiques au mode
        self.mode_params_widget = QWidget()
        self.mode_params_layout = QVBoxLayout(self.mode_params_widget)
        layout.addWidget(self.mode_params_widget)
        
        self.update_mode_parameters()
        
        return group
        
    def create_status_group(self):
        """Créer le groupe de statut"""
        group = QGroupBox("📊 Statut")
        layout = QVBoxLayout(group)
        
        # État actuel
        self.status_label = QLabel("✅ Prêt")
        self.status_label.setStyleSheet("font-weight: bold; color: #44aa44; font-size: 11px;")
        layout.addWidget(self.status_label)
        
        # Indicateur de mode
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("Mode actuel:"))
        self.current_mode_label = QLabel("Manuel")
        self.current_mode_label.setStyleSheet("font-weight: bold; color: #0078d4;")
        mode_layout.addWidget(self.current_mode_label)
        layout.addLayout(mode_layout)
        
        # Barre de progression pour les tâches automatiques
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        self.progress_bar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #555;
                border-radius: 3px;
                text-align: center;
                background-color: #2a2a2a;
            }
            QProgressBar::chunk {
                background-color: #0078d4;
                border-radius: 2px;
            }
        """)
        layout.addWidget(self.progress_bar)
        
        # Temps d'activité
        self.uptime_label = QLabel("Temps: 00:00")
        self.uptime_label.setStyleSheet("font-size: 9px; color: #888;")
        layout.addWidget(self.uptime_label)
        
        # Timer pour mettre à jour le temps
        self.uptime_timer = QTimer()
        self.uptime_timer.timeout.connect(self.update_uptime)
        self.uptime_timer.start(1000)  # Chaque seconde
        self.start_time = 0
        
        return group
        
    def on_speed_changed(self, value):
        """Gérer le changement de vitesse"""
        self.speed_label.setText(f"{value}%")
        self.speed_changed.emit(value)
        
    def on_turn_speed_changed(self, value):
        """Gérer le changement de vitesse de rotation"""
        self.turn_label.setText(f"{value}%")
        
    def on_mode_changed(self, mode_text):
        """Gérer le changement de mode"""
        mode_map = {
            "Manuel": "manual",
            "Suivi de ligne": "line_following",
            "Évitement d'obstacles": "obstacle_avoidance",
            "Exploration": "exploration",
            "Retour à la base": "return_home"
        }
        
        self.current_mode = mode_map.get(mode_text, "manual")
        self.current_mode_label.setText(mode_text)
        self.mode_changed.emit(self.current_mode)
        self.update_mode_parameters()
        
    def update_mode_parameters(self):
        """Mettre à jour les paramètres spécifiques au mode"""
        # Nettoyer les anciens paramètres
        for i in reversed(range(self.mode_params_layout.count())):
            child = self.mode_params_layout.itemAt(i).widget()
            if child:
                child.setParent(None)
            
        if self.current_mode == "line_following":
            # Paramètres pour le suivi de ligne
            kp_layout = QHBoxLayout()
            kp_layout.addWidget(QLabel("Gain P:"))
            kp_slider = QSlider(Qt.Orientation.Horizontal)
            kp_slider.setRange(1, 100)
            kp_slider.setValue(50)
            kp_slider.setStyleSheet(self.speed_slider.styleSheet())
            kp_layout.addWidget(kp_slider)
            kp_label = QLabel("0.5")
            kp_label.setStyleSheet("font-weight: bold; color: #0078d4;")
            kp_slider.valueChanged.connect(lambda v: kp_label.setText(f"{v/100:.2f}"))
            kp_layout.addWidget(kp_label)
            
            widget = QWidget()
            widget.setLayout(kp_layout)
            self.mode_params_layout.addWidget(widget)
            
        elif self.current_mode == "obstacle_avoidance":
            # Paramètres pour l'évitement d'obstacles
            distance_layout = QHBoxLayout()
            distance_layout.addWidget(QLabel("Distance sécurité:"))
            distance_spin = QSpinBox()
            distance_spin.setRange(10, 100)
            distance_spin.setValue(30)
            distance_spin.setSuffix(" cm")
            distance_spin.setStyleSheet("""
                QSpinBox {
                    padding: 3px;
                    border: 1px solid #555;
                    border-radius: 3px;
                    background-color: #3a3a3a;
                    color: white;
                }
            """)
            distance_layout.addWidget(distance_spin)
            
            widget = QWidget()
            widget.setLayout(distance_layout)
            self.mode_params_layout.addWidget(widget)
            
        elif self.current_mode == "exploration":
            # Paramètres pour l'exploration
            time_layout = QHBoxLayout()
            time_layout.addWidget(QLabel("Durée max:"))
            time_spin = QSpinBox()
            time_spin.setRange(30, 600)
            time_spin.setValue(120)
            time_spin.setSuffix(" sec")
            time_spin.setStyleSheet("""
                QSpinBox {
                    padding: 3px;
                    border: 1px solid #555;
                    border-radius: 3px;
                    background-color: #3a3a3a;
                    color: white;
                }
            """)
            time_layout.addWidget(time_spin)
            
            widget = QWidget()
            widget.setLayout(time_layout)
            self.mode_params_layout.addWidget(widget)
            
    def start_auto_mode(self):
        """Démarrer le mode automatique"""
        self.auto_mode_active = True
        self.btn_start_auto.setEnabled(False)
        self.btn_stop_auto.setEnabled(True)
        
        # Désactiver les contrôles manuels
        self.set_manual_controls_enabled(False)
        
        self.status_label.setText(f"🤖 Mode automatique: {self.current_mode}")
        self.status_label.setStyleSheet("font-weight: bold; color: #44aa44; font-size: 11px;")
        
        if self.current_mode in ["line_following", "exploration"]:
            self.progress_bar.setVisible(True)
            self.progress_bar.setRange(0, 0)  # Mode indéterminé
            
    def stop_auto_mode(self):
        """Arrêter le mode automatique"""
        self.auto_mode_active = False
        self.btn_start_auto.setEnabled(True)
        self.btn_stop_auto.setEnabled(False)
        
        # Réactiver les contrôles manuels
        self.set_manual_controls_enabled(True)
        
        self.status_label.setText("⏹️ Mode automatique arrêté")
        self.status_label.setStyleSheet("font-weight: bold; color: #aa4444; font-size: 11px;")
        
        self.progress_bar.setVisible(False)
        self.stop_command.emit()
        
    def set_manual_controls_enabled(self, enabled):
        """Activer/désactiver les contrôles manuels"""
        controls = [self.btn_forward, self.btn_backward, self.btn_left, 
                   self.btn_right, self.btn_stop]
        
        for control in controls:
            control.setEnabled(enabled)
            
        # Changer l'opacité visuelle
        opacity = "1.0" if enabled else "0.5"
        for control in controls:
            control.setStyleSheet(control.styleSheet() + f"opacity: {opacity};")
            
    def update_status(self, status_text, color="#44aa44"):
        """Mettre à jour le statut"""
        self.status_label.setText(status_text)
        self.status_label.setStyleSheet(f"font-weight: bold; color: {color}; font-size: 11px;")
        
    def update_progress(self, value):
        """Mettre à jour la barre de progression"""
        if self.progress_bar.isVisible():
            if self.progress_bar.maximum() == 0:
                self.progress_bar.setRange(0, 100)
            self.progress_bar.setValue(value)
            
    def update_uptime(self):
        """Mettre à jour le temps d'activité"""
        if self.start_time == 0:
            import time
            self.start_time = time.time()
        
        import time
        elapsed = int(time.time() - self.start_time)
        minutes = elapsed // 60
        seconds = elapsed % 60
        self.uptime_label.setText(f"Temps: {minutes:02d}:{seconds:02d}")
        
    def get_current_speed(self):
        """Obtenir la vitesse actuelle"""
        return self.speed_slider.value()
        
    def get_current_turn_speed(self):
        """Obtenir la vitesse de rotation actuelle"""
        return self.turn_slider.value()