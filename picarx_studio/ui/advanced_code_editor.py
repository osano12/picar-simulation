#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Éditeur de code Python avancé avec QScintilla pour PiCarX Studio
"""

import sys
import subprocess
import tempfile
import os
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
                            QLabel, QComboBox, QSplitter, QMessageBox, QCompleter)
from PyQt6.QtCore import Qt, pyqtSignal, QThread, QTimer, QStringListModel
from PyQt6.QtGui import QFont, QColor

try:
    from PyQt6.Qsci import QsciScintilla, QsciLexerPython, QsciAPIs
    QSCINTILLA_AVAILABLE = True
except ImportError:
    QSCINTILLA_AVAILABLE = False
    print("⚠️ QScintilla non disponible - utilisation de l'éditeur basique")

class AdvancedCodeEditor(QWidget):
    """Éditeur de code Python avancé avec QScintilla"""
    
    code_executed = pyqtSignal(dict)
    
    def __init__(self):
        super().__init__()
        self.execution_thread = None
        self.init_ui()
        self.setup_autocompletion()
        self.load_examples()
        
    def init_ui(self):
        """Initialiser l'interface utilisateur"""
        layout = QVBoxLayout(self)
        
        # Barre d'outils
        toolbar_layout = QHBoxLayout()
        
        # Sélecteur d'exemples
        toolbar_layout.addWidget(QLabel("Exemples:"))
        self.examples_combo = QComboBox()
        self.examples_combo.setStyleSheet("""
            QComboBox {
                padding: 5px;
                border: 1px solid #555;
                border-radius: 3px;
                background-color: #3a3a3a;
                color: white;
                min-width: 150px;
            }
            QComboBox QAbstractItemView {
                background-color: #3a3a3a;
                color: white;
                selection-background-color: #0078d4;
            }
        """)
        self.examples_combo.currentTextChanged.connect(self.load_example)
        toolbar_layout.addWidget(self.examples_combo)
        
        toolbar_layout.addStretch()
        
        # Boutons d'action
        self.btn_run = QPushButton("▶️ Exécuter")
        self.btn_run.setStyleSheet("""
            QPushButton {
                background-color: #44aa44;
                color: white;
                font-weight: bold;
                padding: 8px 16px;
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
        self.btn_run.clicked.connect(self.execute_code)
        toolbar_layout.addWidget(self.btn_run)
        
        self.btn_clear = QPushButton("🗑️ Effacer")
        self.btn_clear.setStyleSheet("""
            QPushButton {
                background-color: #aa4444;
                color: white;
                font-weight: bold;
                padding: 8px 16px;
                border: none;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #bb5555;
            }
            QPushButton:pressed {
                background-color: #993333;
            }
        """)
        self.btn_clear.clicked.connect(self.clear_editor)
        toolbar_layout.addWidget(self.btn_clear)
        
        self.btn_format = QPushButton("🎨 Formater")
        self.btn_format.setStyleSheet("""
            QPushButton {
                background-color: #4444aa;
                color: white;
                font-weight: bold;
                padding: 8px 16px;
                border: none;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #5555bb;
            }
            QPushButton:pressed {
                background-color: #333399;
            }
        """)
        self.btn_format.clicked.connect(self.format_code)
        toolbar_layout.addWidget(self.btn_format)
        
        layout.addLayout(toolbar_layout)
        
        # Éditeur de code
        if QSCINTILLA_AVAILABLE:
            self.text_editor = QsciScintilla()
            self.setup_qscintilla_editor()
        else:
            # Fallback vers l'éditeur basique
            from PyQt6.QtWidgets import QTextEdit
            self.text_editor = QTextEdit()
            self.setup_basic_editor()
        
        layout.addWidget(self.text_editor)
        
        # Indicateur de statut
        status_layout = QHBoxLayout()
        self.status_label = QLabel("Prêt à coder")
        self.status_label.setStyleSheet("color: #888; font-size: 10px; font-style: italic;")
        status_layout.addWidget(self.status_label)
        
        status_layout.addStretch()
        
        # Indicateur de ligne/colonne
        self.cursor_label = QLabel("Ligne: 1, Col: 1")
        self.cursor_label.setStyleSheet("color: #888; font-size: 10px;")
        status_layout.addWidget(self.cursor_label)
        
        layout.addLayout(status_layout)
        
    def setup_qscintilla_editor(self):
        """Configurer l'éditeur QScintilla"""
        # Lexer Python
        lexer = QsciLexerPython()
        lexer.setDefaultFont(QFont("Consolas", 11))
        
        # Couleurs du thème sombre
        lexer.setColor(QColor(212, 212, 212), QsciLexerPython.Default)  # Texte par défaut
        lexer.setColor(QColor(86, 156, 214), QsciLexerPython.Keyword)   # Mots-clés
        lexer.setColor(QColor(206, 145, 120), QsciLexerPython.String)   # Chaînes
        lexer.setColor(QColor(106, 153, 85), QsciLexerPython.Comment)   # Commentaires
        lexer.setColor(QColor(181, 206, 168), QsciLexerPython.Number)   # Nombres
        lexer.setColor(QColor(220, 220, 170), QsciLexerPython.FunctionMethodName)  # Fonctions
        
        self.text_editor.setLexer(lexer)
        
        # Configuration générale
        self.text_editor.setMarginType(0, QsciScintilla.MarginType.NumberMargin)
        self.text_editor.setMarginWidth(0, "0000")
        self.text_editor.setMarginLineNumbers(0, True)
        self.text_editor.setMarginsBackgroundColor(QColor(60, 60, 60))
        self.text_editor.setMarginsForegroundColor(QColor(200, 200, 200))
        
        # Couleurs de fond
        self.text_editor.setPaper(QColor(30, 30, 30))
        self.text_editor.setCaretLineVisible(True)
        self.text_editor.setCaretLineBackgroundColor(QColor(40, 40, 40))
        
        # Indentation
        self.text_editor.setIndentationsUseTabs(False)
        self.text_editor.setIndentationWidth(4)
        self.text_editor.setTabWidth(4)
        self.text_editor.setAutoIndent(True)
        
        # Parenthèses correspondantes
        self.text_editor.setBraceMatching(QsciScintilla.BraceMatch.SloppyBraceMatch)
        
        # Pliage de code
        self.text_editor.setFolding(QsciScintilla.FoldStyle.BoxedTreeFoldStyle)
        
        # Sélection
        self.text_editor.setSelectionBackgroundColor(QColor(0, 120, 212))
        
        # Événements
        self.text_editor.cursorPositionChanged.connect(self.update_cursor_position)
        
    def setup_basic_editor(self):
        """Configurer l'éditeur basique (fallback)"""
        self.text_editor.setFont(QFont("Consolas", 11))
        self.text_editor.setStyleSheet("""
            QTextEdit {
                background-color: #1e1e1e;
                color: #d4d4d4;
                border: 1px solid #555;
                border-radius: 5px;
                padding: 10px;
                line-height: 1.4;
            }
        """)
        
    def setup_autocompletion(self):
        """Configurer l'autocomplétion"""
        if QSCINTILLA_AVAILABLE:
            # API pour l'autocomplétion
            self.api = QsciAPIs(self.text_editor.lexer())
            
            # Mots-clés Python
            python_keywords = [
                'and', 'as', 'assert', 'break', 'class', 'continue', 'def',
                'del', 'elif', 'else', 'except', 'finally', 'for', 'from',
                'global', 'if', 'import', 'in', 'is', 'lambda', 'not', 'or',
                'pass', 'raise', 'return', 'try', 'while', 'with', 'yield',
                'True', 'False', 'None'
            ]
            
            # API PiCarX simulée
            picarx_api = [
                'robot.forward()', 'robot.backward()', 'robot.left()', 'robot.right()',
                'robot.stop()', 'robot.get_distance()', 'robot.get_ir_sensors()',
                'robot.set_speed()', 'robot.get_position()', 'robot.reset()',
                'sensors.ultrasonic', 'sensors.ir_left', 'sensors.ir_right',
                'sensors.imu', 'sensors.battery', 'print()', 'time.sleep()',
                'random.randint()', 'math.sqrt()', 'len()', 'range()', 'str()',
                'int()', 'float()', 'list()', 'dict()', 'tuple()'
            ]
            
            # Ajouter toutes les suggestions
            for keyword in python_keywords + picarx_api:
                self.api.add(keyword)
                
            self.api.prepare()
            
            # Configuration de l'autocomplétion
            self.text_editor.setAutoCompletionSource(QsciScintilla.AutoCompletionSource.AcsAll)
            self.text_editor.setAutoCompletionThreshold(2)
            self.text_editor.setAutoCompletionCaseSensitivity(False)
            
    def update_cursor_position(self, line, col):
        """Mettre à jour la position du curseur"""
        self.cursor_label.setText(f"Ligne: {line + 1}, Col: {col + 1}")
        
    def load_examples(self):
        """Charger les exemples de code"""
        examples = {
            "Sélectionner un exemple...": "",
            "Hello PiCarX": '''# Premier programme PiCarX
print("🤖 Hello, PiCarX Studio!")
print("Bienvenue dans l'environnement de simulation")

# Variables du robot
robot_name = "PiCarX"
version = "1.0"

print(f"Nom du robot: {robot_name}")
print(f"Version: {version}")

# Simulation de mouvement
print("\\n=== Test de mouvement ===")
print("🚀 Démarrage du robot...")
print("➡️ Avancer de 10 unités")
print("🔄 Tourner à droite de 90°")
print("➡️ Avancer de 5 unités")
print("⏹️ Arrêt du robot")
print("✅ Test terminé!")
''',
            "API PiCarX Complète": '''# Démonstration de l'API PiCarX complète
import time
import random

class RobotPiCarX:
    """Classe simulée du robot PiCarX"""
    
    def __init__(self):
        self.x = 0
        self.y = 0
        self.angle = 0
        self.speed = 30
        
    def forward(self, distance=10):
        """Avancer d'une distance donnée"""
        print(f"🤖 Avancer de {distance} unités")
        self.x += distance
        
    def backward(self, distance=10):
        """Reculer d'une distance donnée"""
        print(f"🔙 Reculer de {distance} unités")
        self.x -= distance
        
    def left(self, angle=90):
        """Tourner à gauche"""
        print(f"↩️ Tourner à gauche de {angle}°")
        self.angle -= angle
        
    def right(self, angle=90):
        """Tourner à droite"""
        print(f"↪️ Tourner à droite de {angle}°")
        self.angle += angle
        
    def stop(self):
        """Arrêter le robot"""
        print("⏹️ Arrêt du robot")
        
    def get_distance(self):
        """Obtenir la distance du capteur ultrasonique"""
        distance = random.randint(10, 200)
        print(f"📡 Distance mesurée: {distance} cm")
        return distance
        
    def get_ir_sensors(self):
        """Obtenir l'état des capteurs IR"""
        ir_left = random.choice([True, False])
        ir_right = random.choice([True, False])
        print(f"🔍 IR Gauche: {ir_left}, IR Droit: {ir_right}")
        return ir_left, ir_right
        
    def get_position(self):
        """Obtenir la position actuelle"""
        print(f"📍 Position: ({self.x}, {self.y}, {self.angle}°)")
        return self.x, self.y, self.angle

# Démonstration
robot = RobotPiCarX()

print("=== Démonstration API PiCarX ===")
robot.forward(20)
robot.right(90)
robot.forward(15)
robot.get_distance()
robot.get_ir_sensors()
robot.get_position()
robot.stop()
''',
            "Algorithme Avancé": '''# Algorithme de navigation intelligente
import random
import time

class SmartNavigation:
    """Navigation intelligente avec évitement d'obstacles"""
    
    def __init__(self):
        self.robot_x = 0
        self.robot_y = 0
        self.robot_angle = 0
        self.obstacles_detected = []
        self.path_history = []
        
    def scan_environment(self):
        """Scanner l'environnement autour du robot"""
        print("🔍 Scan de l'environnement...")
        
        # Simuler la détection d'obstacles
        obstacles = []
        for angle in range(0, 360, 45):
            distance = random.randint(20, 150)
            if distance < 50:
                obstacles.append((angle, distance))
                print(f"  ⚠️ Obstacle détecté à {angle}°: {distance}cm")
        
        return obstacles
    
    def find_best_path(self, obstacles):
        """Trouver le meilleur chemin"""
        print("🧠 Calcul du meilleur chemin...")
        
        # Zones libres (angles sans obstacles proches)
        free_angles = []
        for angle in range(0, 360, 30):
            is_free = True
            for obs_angle, obs_dist in obstacles:
                if abs(angle - obs_angle) < 45 and obs_dist < 80:
                    is_free = False
                    break
            if is_free:
                free_angles.append(angle)
        
        if free_angles:
            best_angle = random.choice(free_angles)
            print(f"  ✅ Meilleur angle: {best_angle}°")
            return best_angle
        else:
            print("  🚨 Aucun chemin libre - Reculer")
            return -1
    
    def execute_movement(self, target_angle):
        """Exécuter le mouvement vers l'angle cible"""
        if target_angle == -1:
            print("🔙 Reculer de 20 unités")
            self.robot_x -= 20
            return
        
        # Calculer la rotation nécessaire
        angle_diff = target_angle - self.robot_angle
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360
        
        if abs(angle_diff) > 10:
            direction = "droite" if angle_diff > 0 else "gauche"
            print(f"🔄 Tourner à {direction} de {abs(angle_diff)}°")
            self.robot_angle = target_angle
        
        print("🚀 Avancer de 30 unités")
        # Calculer nouvelle position (simplifié)
        import math
        rad = math.radians(self.robot_angle)
        self.robot_x += 30 * math.cos(rad)
        self.robot_y += 30 * math.sin(rad)
        
        self.path_history.append((self.robot_x, self.robot_y))
    
    def navigate(self, steps=8):
        """Algorithme principal de navigation"""
        print("🗺️ Démarrage de la navigation intelligente")
        print(f"Position initiale: ({self.robot_x}, {self.robot_y})")
        
        for step in range(steps):
            print(f"\\n--- Étape {step + 1} ---")
            
            # Scanner l'environnement
            obstacles = self.scan_environment()
            
            # Trouver le meilleur chemin
            best_angle = self.find_best_path(obstacles)
            
            # Exécuter le mouvement
            self.execute_movement(best_angle)
            
            # Afficher la position actuelle
            print(f"📍 Position: ({self.robot_x:.1f}, {self.robot_y:.1f})")
            
            # Pause pour la simulation
            time.sleep(0.3)
        
        print(f"\\n🏁 Navigation terminée!")
        print(f"Position finale: ({self.robot_x:.1f}, {self.robot_y:.1f})")
        print(f"Distance parcourue: {len(self.path_history)} points")

# Exécution
navigator = SmartNavigation()
navigator.navigate(6)
'''
        }
        
        for name, code in examples.items():
            self.examples_combo.addItem(name)
            
        self.examples = examples
        
    def load_example(self, example_name):
        """Charger un exemple de code"""
        if hasattr(self, 'examples') and example_name in self.examples and example_name != "Sélectionner un exemple...":
            if QSCINTILLA_AVAILABLE:
                self.text_editor.setText(self.examples[example_name])
            else:
                self.text_editor.setPlainText(self.examples[example_name])
            self.status_label.setText(f"Exemple chargé: {example_name}")
            
    def execute_code(self):
        """Exécuter le code Python"""
        if QSCINTILLA_AVAILABLE:
            code = self.text_editor.text().strip()
        else:
            code = self.text_editor.toPlainText().strip()
        
        if not code:
            self.status_label.setText("❌ Aucun code à exécuter")
            return
            
        if self.execution_thread and self.execution_thread.isRunning():
            self.status_label.setText("⚠️ Exécution en cours...")
            return
            
        self.btn_run.setEnabled(False)
        self.status_label.setText("⏳ Exécution en cours...")
        
        # Créer et démarrer le thread d'exécution
        from ui.code_editor import CodeExecutionThread
        self.execution_thread = CodeExecutionThread(code)
        self.execution_thread.execution_finished.connect(self.on_execution_finished)
        self.execution_thread.start()
        
    def on_execution_finished(self, result):
        """Gérer la fin d'exécution"""
        self.btn_run.setEnabled(True)
        
        if result['success']:
            self.status_label.setText("✅ Exécution réussie")
        else:
            self.status_label.setText("❌ Erreur d'exécution")
            
        # Émettre le signal avec le résultat
        self.code_executed.emit(result)
        
    def format_code(self):
        """Formater le code avec autopep8 (si disponible)"""
        try:
            import autopep8
            if QSCINTILLA_AVAILABLE:
                code = self.text_editor.text()
            else:
                code = self.text_editor.toPlainText()
                
            formatted_code = autopep8.fix_code(code)
            
            if QSCINTILLA_AVAILABLE:
                self.text_editor.setText(formatted_code)
            else:
                self.text_editor.setPlainText(formatted_code)
                
            self.status_label.setText("✨ Code formaté")
        except ImportError:
            QMessageBox.information(self, "Formatage", 
                                  "Le formatage automatique nécessite autopep8.\n"
                                  "Installez-le avec: pip install autopep8")
        except Exception as e:
            self.status_label.setText(f"❌ Erreur de formatage: {e}")
        
    def clear_editor(self):
        """Effacer l'éditeur"""
        if QSCINTILLA_AVAILABLE:
            self.text_editor.clear()
        else:
            self.text_editor.clear()
        self.examples_combo.setCurrentIndex(0)
        self.status_label.setText("🗑️ Éditeur effacé")
        
    def setPlainText(self, text):
        """Définir le texte de l'éditeur"""
        if QSCINTILLA_AVAILABLE:
            self.text_editor.setText(text)
        else:
            self.text_editor.setPlainText(text)
        
    def toPlainText(self):
        """Obtenir le texte de l'éditeur"""
        if QSCINTILLA_AVAILABLE:
            return self.text_editor.text()
        else:
            return self.text_editor.toPlainText()
        
    def clear(self):
        """Effacer l'éditeur"""
        self.clear_editor()