#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Panneau des scénarios pour PiCarX Studio
"""

from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QListWidget, 
                            QListWidgetItem, QPushButton, QLabel, QTextEdit,
                            QGroupBox, QProgressBar, QComboBox)
from PyQt6.QtCore import Qt, pyqtSignal, QTimer
from PyQt6.QtGui import QFont, QIcon

class ScenarioPanel(QWidget):
    """Panneau de gestion des scénarios d'apprentissage"""
    
    # Signaux
    scenario_selected = pyqtSignal(str)
    run_scenario = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.scenarios = {}
        self.current_scenario = None
        self.init_ui()
        self.load_scenarios()
        
    def init_ui(self):
        """Initialiser l'interface utilisateur"""
        layout = QVBoxLayout(self)
        
        # Groupe de sélection de scénarios
        selection_group = self.create_selection_group()
        layout.addWidget(selection_group)
        
        # Groupe de détails du scénario
        details_group = self.create_details_group()
        layout.addWidget(details_group)
        
        # Groupe de progression
        progress_group = self.create_progress_group()
        layout.addWidget(progress_group)
        
        layout.addStretch()
        
    def create_selection_group(self):
        """Créer le groupe de sélection"""
        group = QGroupBox("🎯 Scénarios Disponibles")
        layout = QVBoxLayout(group)
        
        # Filtre par catégorie
        filter_layout = QHBoxLayout()
        filter_layout.addWidget(QLabel("Catégorie:"))
        
        self.category_combo = QComboBox()
        self.category_combo.addItems([
            "Tous",
            "Débutant",
            "Intermédiaire", 
            "Avancé",
            "Défis"
        ])
        self.category_combo.setStyleSheet("""
            QComboBox {
                padding: 5px;
                border: 1px solid #555;
                border-radius: 3px;
                background-color: #3a3a3a;
                color: white;
            }
            QComboBox QAbstractItemView {
                background-color: #3a3a3a;
                color: white;
                selection-background-color: #0078d4;
            }
        """)
        self.category_combo.currentTextChanged.connect(self.filter_scenarios)
        filter_layout.addWidget(self.category_combo)
        
        layout.addLayout(filter_layout)
        
        # Liste des scénarios
        self.scenario_list = QListWidget()
        self.scenario_list.setStyleSheet("""
            QListWidget {
                background-color: #2a2a2a;
                border: 1px solid #555;
                border-radius: 5px;
                padding: 5px;
            }
            QListWidget::item {
                padding: 8px;
                border-bottom: 1px solid #444;
                color: white;
            }
            QListWidget::item:selected {
                background-color: #0078d4;
                border-radius: 3px;
            }
            QListWidget::item:hover {
                background-color: #3a3a3a;
                border-radius: 3px;
            }
        """)
        self.scenario_list.itemClicked.connect(self.on_scenario_selected)
        layout.addWidget(self.scenario_list)
        
        return group
        
    def create_details_group(self):
        """Créer le groupe de détails"""
        group = QGroupBox("📋 Détails du Scénario")
        layout = QVBoxLayout(group)
        
        # Titre du scénario
        self.scenario_title = QLabel("Aucun scénario sélectionné")
        self.scenario_title.setStyleSheet("font-weight: bold; font-size: 12px; color: #0078d4;")
        layout.addWidget(self.scenario_title)
        
        # Difficulté et durée
        info_layout = QHBoxLayout()
        self.difficulty_label = QLabel("Difficulté: -")
        self.difficulty_label.setStyleSheet("font-size: 10px; color: #888;")
        info_layout.addWidget(self.difficulty_label)
        
        info_layout.addStretch()
        
        self.duration_label = QLabel("Durée: -")
        self.duration_label.setStyleSheet("font-size: 10px; color: #888;")
        info_layout.addWidget(self.duration_label)
        
        layout.addLayout(info_layout)
        
        # Description
        self.scenario_description = QTextEdit()
        self.scenario_description.setMaximumHeight(100)
        self.scenario_description.setReadOnly(True)
        self.scenario_description.setStyleSheet("""
            QTextEdit {
                background-color: #2a2a2a;
                border: 1px solid #555;
                border-radius: 3px;
                padding: 5px;
                color: #ccc;
                font-size: 10px;
            }
        """)
        self.scenario_description.setPlaceholderText("Sélectionnez un scénario pour voir sa description...")
        layout.addWidget(self.scenario_description)
        
        # Objectifs
        self.objectives_label = QLabel("🎯 Objectifs:")
        self.objectives_label.setStyleSheet("font-weight: bold; font-size: 10px; margin-top: 5px;")
        layout.addWidget(self.objectives_label)
        
        self.objectives_list = QTextEdit()
        self.objectives_list.setMaximumHeight(80)
        self.objectives_list.setReadOnly(True)
        self.objectives_list.setStyleSheet(self.scenario_description.styleSheet())
        layout.addWidget(self.objectives_list)
        
        # Boutons d'action
        buttons_layout = QHBoxLayout()
        
        self.btn_load = QPushButton("📂 Charger")
        self.btn_load.setStyleSheet("""
            QPushButton {
                background-color: #0078d4;
                color: white;
                font-weight: bold;
                padding: 8px 12px;
                border: none;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #106ebe;
            }
            QPushButton:disabled {
                background-color: #666;
                color: #aaa;
            }
        """)
        self.btn_load.clicked.connect(self.load_scenario)
        self.btn_load.setEnabled(False)
        buttons_layout.addWidget(self.btn_load)
        
        self.btn_run = QPushButton("▶️ Exécuter")
        self.btn_run.setStyleSheet("""
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
            QPushButton:disabled {
                background-color: #666;
                color: #aaa;
            }
        """)
        self.btn_run.clicked.connect(self.run_current_scenario)
        self.btn_run.setEnabled(False)
        buttons_layout.addWidget(self.btn_run)
        
        layout.addLayout(buttons_layout)
        
        return group
        
    def create_progress_group(self):
        """Créer le groupe de progression"""
        group = QGroupBox("📊 Progression")
        layout = QVBoxLayout(group)
        
        # Progression globale
        progress_layout = QHBoxLayout()
        progress_layout.addWidget(QLabel("Progression globale:"))
        
        self.global_progress = QProgressBar()
        self.global_progress.setRange(0, 100)
        self.global_progress.setValue(25)
        self.global_progress.setStyleSheet("""
            QProgressBar {
                border: 1px solid #555;
                border-radius: 3px;
                text-align: center;
                background-color: #2a2a2a;
                color: white;
            }
            QProgressBar::chunk {
                background-color: #44aa44;
                border-radius: 2px;
            }
        """)
        progress_layout.addWidget(self.global_progress)
        
        self.progress_label = QLabel("25%")
        self.progress_label.setStyleSheet("font-weight: bold; color: #44aa44;")
        progress_layout.addWidget(self.progress_label)
        
        layout.addLayout(progress_layout)
        
        # Statistiques
        stats_layout = QHBoxLayout()
        
        self.completed_label = QLabel("✅ Complétés: 3")
        self.completed_label.setStyleSheet("font-size: 10px; color: #44aa44;")
        stats_layout.addWidget(self.completed_label)
        
        stats_layout.addStretch()
        
        self.total_label = QLabel("📚 Total: 12")
        self.total_label.setStyleSheet("font-size: 10px; color: #888;")
        stats_layout.addWidget(self.total_label)
        
        layout.addLayout(stats_layout)
        
        # Badges/Récompenses
        badges_label = QLabel("🏆 Badges obtenus:")
        badges_label.setStyleSheet("font-size: 10px; font-weight: bold; margin-top: 5px;")
        layout.addWidget(badges_label)
        
        self.badges_display = QLabel("🥉 Premier pas  🎯 Précision  🚀 Vitesse")
        self.badges_display.setStyleSheet("font-size: 9px; color: #ffaa44; font-style: italic;")
        self.badges_display.setWordWrap(True)
        layout.addWidget(self.badges_display)
        
        return group
        
    def load_scenarios(self):
        """Charger les scénarios disponibles"""
        self.scenarios = {
            "intro_basics": {
                "title": "🚀 Introduction aux bases",
                "category": "Débutant",
                "difficulty": "⭐ Facile",
                "duration": "5 min",
                "description": "Découvrez les commandes de base du robot PiCarX. Apprenez à le faire avancer, reculer et tourner.",
                "objectives": [
                    "Comprendre les commandes de mouvement",
                    "Faire avancer le robot en ligne droite",
                    "Effectuer des virages à gauche et à droite",
                    "Arrêter le robot proprement"
                ],
                "completed": True,
                "code_template": '''# Introduction aux bases du PiCarX
print("=== Commandes de base ===")

# Avancer
print("🤖 Avancer de 10 unités")

# Tourner
print("🔄 Tourner à droite de 90°")

# Reculer
print("🔙 Reculer de 5 unités")

print("✅ Séquence terminée!")
'''
            },
            "sensor_reading": {
                "title": "📡 Lecture des capteurs",
                "category": "Débutant", 
                "difficulty": "⭐ Facile",
                "duration": "8 min",
                "description": "Apprenez à lire et interpréter les données des différents capteurs du robot.",
                "objectives": [
                    "Lire le capteur ultrasonique",
                    "Interpréter les capteurs infrarouges",
                    "Afficher les données de l'IMU",
                    "Créer un tableau de bord des capteurs"
                ],
                "completed": True,
                "code_template": '''# Lecture des capteurs
import random

def lire_capteurs():
    distance = random.randint(10, 200)
    ir_gauche = random.choice([True, False])
    ir_droit = random.choice([True, False])
    
    print(f"📡 Distance: {distance} cm")
    print(f"🔍 IR Gauche: {ir_gauche}")
    print(f"🔍 IR Droit: {ir_droit}")

for i in range(5):
    print(f"\\n--- Lecture {i+1} ---")
    lire_capteurs()
'''
            },
            "line_following": {
                "title": "🛤️ Suivi de ligne",
                "category": "Intermédiaire",
                "difficulty": "⭐⭐ Moyen",
                "duration": "15 min", 
                "description": "Programmez le robot pour suivre une ligne jaune en utilisant les capteurs infrarouges.",
                "objectives": [
                    "Détecter la ligne avec les capteurs IR",
                    "Implémenter un algorithme de suivi",
                    "Gérer les virages et intersections",
                    "Optimiser la vitesse de suivi"
                ],
                "completed": True,
                "code_template": '''# Algorithme de suivi de ligne
def suivi_ligne():
    print("🛤️ Démarrage du suivi de ligne")
    
    for step in range(10):
        # Simulation des capteurs
        ir_gauche = step % 3 == 0
        ir_droit = step % 4 == 0
        
        if ir_gauche and ir_droit:
            print("➡️ Sur la ligne - Avancer")
        elif ir_gauche:
            print("↩️ Tourner à gauche")
        elif ir_droit:
            print("↪️ Tourner à droite")
        else:
            print("🔍 Rechercher la ligne")

suivi_ligne()
'''
            },
            "obstacle_avoidance": {
                "title": "🚧 Évitement d'obstacles",
                "category": "Intermédiaire",
                "difficulty": "⭐⭐ Moyen", 
                "duration": "20 min",
                "description": "Développez un système d'évitement d'obstacles utilisant le capteur ultrasonique.",
                "objectives": [
                    "Détecter les obstacles à distance",
                    "Planifier des trajectoires d'évitement",
                    "Naviguer autour des obstacles",
                    "Reprendre la route après évitement"
                ],
                "completed": False,
                "code_template": '''# Évitement d'obstacles
import random

def eviter_obstacles():
    print("🚧 Système d'évitement activé")
    
    for i in range(8):
        distance = random.randint(5, 100)
        print(f"📡 Distance: {distance} cm")
        
        if distance < 30:
            print("🚨 Obstacle détecté - Évitement!")
            print("🔄 Tourner à droite")
            print("➡️ Avancer")
            print("🔄 Tourner à gauche")
        else:
            print("✅ Voie libre - Avancer")

eviter_obstacles()
'''
            },
            "maze_navigation": {
                "title": "🌀 Navigation en labyrinthe",
                "category": "Avancé",
                "difficulty": "⭐⭐⭐ Difficile",
                "duration": "30 min",
                "description": "Programmez le robot pour naviguer dans un labyrinthe complexe.",
                "objectives": [
                    "Implémenter l'algorithme main droite",
                    "Cartographier le labyrinthe",
                    "Trouver la sortie optimale",
                    "Gérer les impasses"
                ],
                "completed": False,
                "code_template": '''# Navigation en labyrinthe
def navigation_labyrinthe():
    print("🌀 Entrée dans le labyrinthe")
    
    # Algorithme main droite simplifié
    directions = ["Nord", "Est", "Sud", "Ouest"]
    direction_actuelle = 0
    
    for step in range(15):
        print(f"Étape {step+1}: Direction {directions[direction_actuelle]}")
        
        # Simulation de détection de mur
        mur_droit = step % 3 == 0
        mur_devant = step % 5 == 0
        
        if not mur_droit:
            print("↪️ Tourner à droite")
            direction_actuelle = (direction_actuelle + 1) % 4
        elif not mur_devant:
            print("➡️ Avancer")
        else:
            print("↩️ Tourner à gauche")
            direction_actuelle = (direction_actuelle - 1) % 4

navigation_labyrinthe()
'''
            },
            "speed_challenge": {
                "title": "🏁 Défi de vitesse",
                "category": "Défis",
                "difficulty": "⭐⭐⭐ Difficile",
                "duration": "25 min",
                "description": "Parcourez un circuit le plus rapidement possible sans collision.",
                "objectives": [
                    "Optimiser la vitesse de déplacement",
                    "Minimiser le temps de parcours",
                    "Éviter toutes les collisions",
                    "Battre le record de temps"
                ],
                "completed": False,
                "code_template": '''# Défi de vitesse
import time

def defi_vitesse():
    print("🏁 Défi de vitesse - START!")
    start_time = time.time()
    
    # Circuit optimisé
    mouvements = [
        ("Avancer", 20),
        ("Tourner droite", 90),
        ("Avancer", 15),
        ("Tourner gauche", 90),
        ("Avancer", 25),
        ("Tourner droite", 180),
        ("Avancer", 30)
    ]
    
    for action, valeur in mouvements:
        print(f"🚀 {action}: {valeur}")
        time.sleep(0.1)  # Simulation
    
    end_time = time.time()
    temps_total = end_time - start_time
    print(f"🏆 Temps total: {temps_total:.2f} secondes")

defi_vitesse()
'''
            }
        }
        
        self.update_scenario_list()
        
    def update_scenario_list(self):
        """Mettre à jour la liste des scénarios"""
        self.scenario_list.clear()
        
        for scenario_id, scenario in self.scenarios.items():
            item = QListWidgetItem()
            
            # Icône selon le statut
            status_icon = "✅" if scenario.get("completed", False) else "⏳"
            
            # Texte de l'item
            item_text = f"{status_icon} {scenario['title']}"
            item.setText(item_text)
            item.setData(Qt.ItemDataRole.UserRole, scenario_id)
            
            # Couleur selon la difficulté
            if "Facile" in scenario['difficulty']:
                item.setForeground(Qt.GlobalColor.green)
            elif "Moyen" in scenario['difficulty']:
                item.setForeground(Qt.GlobalColor.yellow)
            else:
                item.setForeground(Qt.GlobalColor.red)
                
            self.scenario_list.addItem(item)
            
    def filter_scenarios(self, category):
        """Filtrer les scénarios par catégorie"""
        # Pour l'instant, afficher tous les scénarios
        # TODO: Implémenter le filtrage réel
        self.update_scenario_list()
        
    def on_scenario_selected(self, item):
        """Gérer la sélection d'un scénario"""
        scenario_id = item.data(Qt.ItemDataRole.UserRole)
        if scenario_id in self.scenarios:
            scenario = self.scenarios[scenario_id]
            self.current_scenario = scenario_id
            
            # Mettre à jour l'affichage
            self.scenario_title.setText(scenario['title'])
            self.difficulty_label.setText(f"Difficulté: {scenario['difficulty']}")
            self.duration_label.setText(f"Durée: {scenario['duration']}")
            self.scenario_description.setPlainText(scenario['description'])
            
            # Objectifs
            objectives_text = "\\n".join([f"• {obj}" for obj in scenario['objectives']])
            self.objectives_list.setPlainText(objectives_text)
            
            # Activer les boutons
            self.btn_load.setEnabled(True)
            self.btn_run.setEnabled(True)
            
            # Émettre le signal
            self.scenario_selected.emit(scenario_id)
            
    def load_scenario(self):
        """Charger le scénario sélectionné"""
        if self.current_scenario:
            scenario = self.scenarios[self.current_scenario]
            # TODO: Charger le code template dans l'éditeur
            print(f"Chargement du scénario: {scenario['title']}")
            
    def run_current_scenario(self):
        """Exécuter le scénario actuel"""
        if self.current_scenario:
            self.run_scenario.emit(self.current_scenario)
            
    def update_progress(self):
        """Mettre à jour la progression"""
        completed = sum(1 for s in self.scenarios.values() if s.get("completed", False))
        total = len(self.scenarios)
        percentage = int((completed / total) * 100)
        
        self.global_progress.setValue(percentage)
        self.progress_label.setText(f"{percentage}%")
        self.completed_label.setText(f"✅ Complétés: {completed}")
        self.total_label.setText(f"📚 Total: {total}")
        
    def mark_scenario_completed(self, scenario_id):
        """Marquer un scénario comme complété"""
        if scenario_id in self.scenarios:
            self.scenarios[scenario_id]["completed"] = True
            self.update_scenario_list()
            self.update_progress()
            
    def get_scenario_code(self, scenario_id):
        """Obtenir le code template d'un scénario"""
        if scenario_id in self.scenarios:
            return self.scenarios[scenario_id].get("code_template", "")
        return ""