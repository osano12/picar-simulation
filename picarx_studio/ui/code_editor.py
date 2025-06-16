#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Éditeur de code Python pour PiCarX Studio
"""

import sys
import subprocess
import tempfile
import os
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QTextEdit, 
                            QPushButton, QLabel, QComboBox, QSplitter)
from PyQt6.QtCore import Qt, pyqtSignal, QThread, QTimer
from PyQt6.QtGui import QFont, QTextCharFormat, QColor, QSyntaxHighlighter, QTextDocument

class PythonHighlighter(QSyntaxHighlighter):
    """Coloration syntaxique pour Python"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.highlighting_rules = []
        
        # Format pour les mots-clés
        keyword_format = QTextCharFormat()
        keyword_format.setForeground(QColor(86, 156, 214))  # Bleu
        keyword_format.setFontWeight(QFont.Weight.Bold)
        
        keywords = [
            'and', 'as', 'assert', 'break', 'class', 'continue', 'def',
            'del', 'elif', 'else', 'except', 'exec', 'finally', 'for',
            'from', 'global', 'if', 'import', 'in', 'is', 'lambda',
            'not', 'or', 'pass', 'print', 'raise', 'return', 'try',
            'while', 'with', 'yield', 'True', 'False', 'None'
        ]
        
        for keyword in keywords:
            pattern = f'\\b{keyword}\\b'
            self.highlighting_rules.append((pattern, keyword_format))
        
        # Format pour les chaînes de caractères
        string_format = QTextCharFormat()
        string_format.setForeground(QColor(206, 145, 120))  # Orange
        self.highlighting_rules.append(('".*"', string_format))
        self.highlighting_rules.append(("'.*'", string_format))
        
        # Format pour les commentaires
        comment_format = QTextCharFormat()
        comment_format.setForeground(QColor(106, 153, 85))  # Vert
        comment_format.setFontItalic(True)
        self.highlighting_rules.append(('#.*', comment_format))
        
        # Format pour les nombres
        number_format = QTextCharFormat()
        number_format.setForeground(QColor(181, 206, 168))  # Vert clair
        self.highlighting_rules.append(('\\b[0-9]+\\b', number_format))
        
        # Format pour les fonctions
        function_format = QTextCharFormat()
        function_format.setForeground(QColor(220, 220, 170))  # Jaune
        self.highlighting_rules.append(('\\b[A-Za-z_][A-Za-z0-9_]*(?=\\()', function_format))
        
    def highlightBlock(self, text):
        """Appliquer la coloration syntaxique"""
        import re
        for pattern, format in self.highlighting_rules:
            for match in re.finditer(pattern, text):
                start, end = match.span()
                self.setFormat(start, end - start, format)

class CodeExecutionThread(QThread):
    """Thread pour exécuter le code Python"""
    
    execution_finished = pyqtSignal(dict)
    
    def __init__(self, code):
        super().__init__()
        self.code = code
        
    def run(self):
        """Exécuter le code"""
        try:
            # Créer un fichier temporaire
            with tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False) as temp_file:
                temp_file.write(self.code)
                temp_filename = temp_file.name
            
            # Exécuter le code
            process = subprocess.Popen(
                [sys.executable, temp_filename],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                env=dict(os.environ, PYTHONIOENCODING='utf-8')
            )
            
            stdout, stderr = process.communicate(timeout=30)
            
            # Nettoyer le fichier temporaire
            os.unlink(temp_filename)
            
            if process.returncode == 0:
                self.execution_finished.emit({
                    'success': True,
                    'output': stdout,
                    'error': None
                })
            else:
                self.execution_finished.emit({
                    'success': False,
                    'output': stdout,
                    'error': stderr
                })
                
        except subprocess.TimeoutExpired:
            process.kill()
            self.execution_finished.emit({
                'success': False,
                'output': '',
                'error': 'Timeout: L\'exécution a pris trop de temps (>30s)'
            })
        except Exception as e:
            self.execution_finished.emit({
                'success': False,
                'output': '',
                'error': f'Erreur d\'exécution: {str(e)}'
            })

class CodeEditor(QWidget):
    """Éditeur de code Python avec coloration syntaxique"""
    
    code_executed = pyqtSignal(dict)
    
    def __init__(self):
        super().__init__()
        self.execution_thread = None
        self.init_ui()
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
        
        layout.addLayout(toolbar_layout)
        
        # Éditeur de texte
        self.text_editor = QTextEdit()
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
        
        # Appliquer la coloration syntaxique
        self.highlighter = PythonHighlighter(self.text_editor.document())
        
        layout.addWidget(self.text_editor)
        
        # Indicateur de statut
        self.status_label = QLabel("Prêt à coder")
        self.status_label.setStyleSheet("color: #888; font-size: 10px; font-style: italic;")
        layout.addWidget(self.status_label)
        
    def load_examples(self):
        """Charger les exemples de code"""
        examples = {
            "Sélectionner un exemple...": "",
            "Hello World": '''# Premier programme Python
print("Hello, PiCarX Studio!")
print("Bienvenue dans l'environnement de simulation")

# Variables
nom = "Robot PiCarX"
version = 1.0

print(f"Nom du robot: {nom}")
print(f"Version: {version}")
''',
            "Contrôle basique": '''# Contrôle basique du robot PiCarX
import time

# Simulation des commandes de base
def avancer(distance=10):
    print(f"🤖 Avancer de {distance} unités")
    
def tourner(angle=90):
    print(f"🔄 Tourner de {angle} degrés")
    
def arreter():
    print("⏹️ Arrêt du robot")

# Programme principal
print("=== Démarrage du programme ===")

# Séquence de mouvements
avancer(20)
time.sleep(1)
tourner(90)
time.sleep(1)
avancer(15)
time.sleep(1)
arreter()

print("=== Programme terminé ===")
''',
            "Lecture capteurs": '''# Simulation de lecture des capteurs
import random
import time

def lire_capteur_ultrason():
    """Simuler la lecture du capteur ultrasonique"""
    distance = random.randint(10, 200)
    return distance

def lire_capteurs_ir():
    """Simuler la lecture des capteurs IR"""
    ir_gauche = random.choice([True, False])
    ir_droit = random.choice([True, False])
    return ir_gauche, ir_droit

def afficher_etat_capteurs():
    """Afficher l'état de tous les capteurs"""
    distance = lire_capteur_ultrason()
    ir_g, ir_d = lire_capteurs_ir()
    
    print(f"📡 Distance ultrason: {distance} cm")
    print(f"🔍 IR Gauche: {'Détecté' if ir_g else 'Libre'}")
    print(f"🔍 IR Droit: {'Détecté' if ir_d else 'Libre'}")
    
    if distance < 30:
        print("⚠️ Obstacle proche!")
    
    if ir_g and ir_d:
        print("🛤️ Sur la ligne")
    elif ir_g:
        print("🛤️ Décalage à droite")
    elif ir_d:
        print("🛤️ Décalage à gauche")
    else:
        print("🛤️ Ligne non détectée")

# Boucle de lecture
print("=== Lecture des capteurs ===")
for i in range(5):
    print(f"\\n--- Lecture {i+1} ---")
    afficher_etat_capteurs()
    time.sleep(0.5)
''',
            "Suivi de ligne": '''# Algorithme de suivi de ligne
import time
import random

class RobotPiCarX:
    def __init__(self):
        self.vitesse = 30
        self.position = 0
        
    def lire_capteurs_ir(self):
        """Simuler les capteurs IR"""
        # Simulation basée sur la position
        ir_gauche = abs(self.position + 1) < 2
        ir_droit = abs(self.position - 1) < 2
        return ir_gauche, ir_droit
    
    def avancer(self):
        print("🤖 Avancer")
        
    def tourner_gauche(self):
        print("🔄 Tourner à gauche")
        self.position -= 0.5
        
    def tourner_droite(self):
        print("🔄 Tourner à droite")
        self.position += 0.5
        
    def arreter(self):
        print("⏹️ Arrêt")

def suivi_ligne(robot, duree=10):
    """Algorithme de suivi de ligne"""
    print("🛤️ Démarrage du suivi de ligne")
    
    for step in range(duree):
        ir_gauche, ir_droit = robot.lire_capteurs_ir()
        
        print(f"\\nÉtape {step+1}: IR_G={ir_gauche}, IR_D={ir_droit}")
        
        if ir_gauche and ir_droit:
            # Sur la ligne - avancer
            robot.avancer()
        elif ir_gauche and not ir_droit:
            # Décalage à droite - tourner à gauche
            robot.tourner_gauche()
        elif not ir_gauche and ir_droit:
            # Décalage à gauche - tourner à droite
            robot.tourner_droite()
        else:
            # Ligne perdue - rechercher
            print("🔍 Recherche de la ligne...")
            robot.tourner_droite()
        
        time.sleep(0.3)
    
    robot.arreter()
    print("\\n✅ Suivi de ligne terminé")

# Programme principal
robot = RobotPiCarX()
suivi_ligne(robot, 8)
''',
            "Évitement d'obstacles": '''# Algorithme d'évitement d'obstacles
import random
import time

class RobotAvoidance:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.angle = 0
        
    def lire_distance(self):
        """Simuler le capteur ultrasonique"""
        return random.randint(5, 100)
    
    def avancer(self, distance=10):
        print(f"🤖 Avancer {distance} unités")
        self.x += distance
        
    def reculer(self, distance=5):
        print(f"🔙 Reculer {distance} unités")
        self.x -= distance
        
    def tourner(self, angle=45):
        print(f"🔄 Tourner de {angle}°")
        self.angle += angle
        
    def position(self):
        return f"Position: ({self.x}, {self.y}, {self.angle}°)"

def eviter_obstacles(robot, iterations=10):
    """Algorithme d'évitement d'obstacles"""
    print("🚧 Démarrage de l'évitement d'obstacles")
    
    for i in range(iterations):
        distance = robot.lire_distance()
        print(f"\\n--- Itération {i+1} ---")
        print(f"📡 Distance mesurée: {distance} cm")
        print(robot.position())
        
        if distance > 50:
            # Chemin libre - avancer
            print("✅ Chemin libre")
            robot.avancer(15)
        elif distance > 20:
            # Obstacle modéré - avancer prudemment
            print("⚠️ Obstacle modéré")
            robot.avancer(5)
        else:
            # Obstacle proche - éviter
            print("🚨 Obstacle proche - évitement!")
            robot.reculer(10)
            
            # Choisir direction d'évitement
            direction = random.choice([-90, 90])
            robot.tourner(direction)
            robot.avancer(20)
            robot.tourner(-direction)  # Revenir à la direction originale
        
        time.sleep(0.4)
    
    print("\\n✅ Évitement d'obstacles terminé")
    print(f"Position finale: {robot.position()}")

# Programme principal
robot = RobotAvoidance()
eviter_obstacles(robot, 6)
'''
        }
        
        for name, code in examples.items():
            self.examples_combo.addItem(name)
            
        self.examples = examples
        
    def load_example(self, example_name):
        """Charger un exemple de code"""
        if hasattr(self, 'examples') and example_name in self.examples and example_name != "Sélectionner un exemple...":
            self.text_editor.setPlainText(self.examples[example_name])
            self.status_label.setText(f"Exemple chargé: {example_name}")
            
    def execute_code(self):
        """Exécuter le code Python"""
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
        
    def clear_editor(self):
        """Effacer l'éditeur"""
        self.text_editor.clear()
        self.examples_combo.setCurrentIndex(0)
        self.status_label.setText("🗑️ Éditeur effacé")
        
    def setPlainText(self, text):
        """Définir le texte de l'éditeur"""
        self.text_editor.setPlainText(text)
        
    def toPlainText(self):
        """Obtenir le texte de l'éditeur"""
        return self.text_editor.toPlainText()
        
    def clear(self):
        """Effacer l'éditeur"""
        self.clear_editor()