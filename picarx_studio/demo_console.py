#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Démonstration console de PiCarX Studio
Version sans interface graphique pour tester les fonctionnalités
"""

import sys
import os
import time
import random

# Ajouter le répertoire du projet au path
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from core.config import AppConfig
from core.logger import setup_logger

class PiCarXConsoleDemo:
    """Démonstration console de PiCarX Studio"""
    
    def __init__(self):
        self.logger = setup_logger()
        self.config = AppConfig()
        self.robot_x = 400
        self.robot_y = 300
        self.robot_angle = 0
        self.simulation_running = False
        
    def show_banner(self):
        """Afficher la bannière de l'application"""
        print("=" * 60)
        print("🤖 PiCarX Studio - Démonstration Console")
        print("   Simulateur et Programmation Robotique")
        print("   Version 1.0.0")
        print("=" * 60)
        print()
        
    def show_menu(self):
        """Afficher le menu principal"""
        print("📋 Menu Principal:")
        print("1. 🎮 Démonstration de contrôle")
        print("2. 📡 Simulation des capteurs")
        print("3. 🐍 Exécution d'exemples de code")
        print("4. 🎯 Scénarios d'apprentissage")
        print("5. ⚙️  Configuration")
        print("6. 📊 Statistiques")
        print("0. 🚪 Quitter")
        print()
        
    def demo_control(self):
        """Démonstration du contrôle du robot"""
        print("🎮 Démonstration de Contrôle du Robot")
        print("-" * 40)
        
        commands = [
            ("Avancer", 20, 0),
            ("Tourner à droite", 0, 90),
            ("Avancer", 15, 0),
            ("Tourner à gauche", 0, -90),
            ("Avancer", 25, 0),
            ("Arrêt", 0, 0)
        ]
        
        for i, (action, distance, angle) in enumerate(commands, 1):
            print(f"Étape {i}: {action}")
            
            if distance > 0:
                self.robot_x += distance * 0.8
                print(f"   Position: ({self.robot_x:.1f}, {self.robot_y:.1f})")
            
            if angle != 0:
                self.robot_angle += angle
                self.robot_angle %= 360
                print(f"   Orientation: {self.robot_angle}°")
            
            time.sleep(0.8)
        
        print("✅ Séquence de contrôle terminée")
        print()
        
    def demo_sensors(self):
        """Démonstration des capteurs"""
        print("📡 Simulation des Capteurs")
        print("-" * 40)
        
        for i in range(8):
            print(f"Lecture {i+1}:")
            
            # Capteur ultrasonique
            distance = random.randint(10, 200)
            print(f"  📡 Ultrason: {distance} cm", end="")
            if distance < 30:
                print(" 🚨 PROCHE!")
            elif distance < 60:
                print(" ⚠️ Attention")
            else:
                print(" ✅ OK")
            
            # Capteurs IR
            ir_left = random.choice([True, False])
            ir_right = random.choice([True, False])
            print(f"  🔍 IR Gauche: {'✅ Détecté' if ir_left else '❌ Libre'}")
            print(f"  🔍 IR Droit: {'✅ Détecté' if ir_right else '❌ Libre'}")
            
            # État de ligne
            if ir_left and ir_right:
                print("  🛤️ Ligne: Sur la ligne")
            elif ir_left:
                print("  🛤️ Ligne: Décalage à droite")
            elif ir_right:
                print("  🛤️ Ligne: Décalage à gauche")
            else:
                print("  🛤️ Ligne: Non détectée")
            
            # IMU
            angular_vel = random.uniform(-50, 50)
            linear_acc = random.uniform(-5, 5)
            print(f"  🧭 Vitesse angulaire: {angular_vel:.2f} °/s")
            print(f"  🧭 Accélération: {linear_acc:.2f} m/s²")
            
            # Batterie
            battery = random.randint(60, 100)
            print(f"  🔋 Batterie: {battery}%", end="")
            if battery > 80:
                print(" ✅ Excellent")
            elif battery > 60:
                print(" ⚠️ Correct")
            else:
                print(" 🔋 Faible")
            
            print()
            time.sleep(1)
        
        print("✅ Simulation des capteurs terminée")
        print()
        
    def demo_code_execution(self):
        """Démonstration d'exécution de code"""
        print("🐍 Exécution d'Exemples de Code")
        print("-" * 40)
        
        examples = {
            "Hello World": '''
print("Hello, PiCarX Studio!")
print("Bienvenue dans l'environnement de simulation")
nom = "Robot PiCarX"
print(f"Nom du robot: {nom}")
''',
            "Contrôle basique": '''
def avancer(distance=10):
    print(f"🤖 Avancer de {distance} unités")
    
def tourner(angle=90):
    print(f"🔄 Tourner de {angle} degrés")

print("=== Séquence de mouvements ===")
avancer(20)
tourner(90)
avancer(15)
print("=== Terminé ===")
''',
            "Lecture capteurs": '''
import random

def lire_capteurs():
    distance = random.randint(10, 200)
    ir_gauche = random.choice([True, False])
    ir_droit = random.choice([True, False])
    
    print(f"📡 Distance: {distance} cm")
    print(f"🔍 IR Gauche: {ir_gauche}")
    print(f"🔍 IR Droit: {ir_droit}")

for i in range(3):
    print(f"\\nLecture {i+1}:")
    lire_capteurs()
'''
        }
        
        for name, code in examples.items():
            print(f"📝 Exemple: {name}")
            print("   Code:")
            for line in code.strip().split('\n'):
                print(f"   {line}")
            print("\n   Résultat:")
            try:
                exec(code)
            except Exception as e:
                print(f"   ❌ Erreur: {e}")
            print("\n" + "-" * 40)
            time.sleep(1)
        
        print("✅ Exemples d'exécution terminés")
        print()
        
    def demo_scenarios(self):
        """Démonstration des scénarios"""
        print("🎯 Scénarios d'Apprentissage")
        print("-" * 40)
        
        scenarios = [
            ("🚀 Introduction aux bases", "Débutant", "✅ Complété"),
            ("📡 Lecture des capteurs", "Débutant", "✅ Complété"),
            ("🛤️ Suivi de ligne", "Intermédiaire", "✅ Complété"),
            ("🚧 Évitement d'obstacles", "Intermédiaire", "⏳ En cours"),
            ("🌀 Navigation en labyrinthe", "Avancé", "🔒 Verrouillé"),
            ("🏁 Défi de vitesse", "Défis", "🔒 Verrouillé")
        ]
        
        print("📚 Liste des scénarios:")
        for i, (title, level, status) in enumerate(scenarios, 1):
            print(f"{i}. {title}")
            print(f"   Niveau: {level}")
            print(f"   Statut: {status}")
            print()
        
        # Progression
        completed = sum(1 for _, _, status in scenarios if "Complété" in status)
        total = len(scenarios)
        percentage = int((completed / total) * 100)
        
        print(f"📊 Progression globale: {percentage}% ({completed}/{total})")
        print("🏆 Badges obtenus: 🥉 Premier pas, 🎯 Précision, 🚀 Vitesse")
        print()
        
    def show_config(self):
        """Afficher la configuration"""
        print("⚙️ Configuration")
        print("-" * 40)
        
        print(f"📱 Application: {self.config.get('app.name', 'PiCarX Studio')}")
        print(f"🔢 Version: {self.config.get('app.version', '1.0.0')}")
        print(f"🎨 Thème: {self.config.get('app.theme', 'dark')}")
        print(f"🗂️ Répertoire de config: {self.config.config_dir}")
        print(f"📁 Répertoire de projets: {self.config.get('paths.projects_dir', 'N/A')}")
        print(f"📝 Répertoire de logs: {self.config.get('paths.logs_dir', 'N/A')}")
        print()
        
        print("🔧 Paramètres de simulation:")
        print(f"  Vitesse max: {self.config.get('simulation.max_speed', 100)}%")
        print(f"  Qualité rendu: {self.config.get('simulation.render_quality', 'high')}")
        print(f"  Physique activée: {self.config.get('simulation.physics_enabled', True)}")
        print()
        
    def show_stats(self):
        """Afficher les statistiques"""
        print("📊 Statistiques")
        print("-" * 40)
        
        stats = {
            "Sessions totales": random.randint(15, 50),
            "Temps total d'utilisation": f"{random.randint(5, 20)}h {random.randint(10, 59)}min",
            "Projets créés": random.randint(3, 12),
            "Lignes de code écrites": random.randint(200, 1500),
            "Scénarios complétés": 3,
            "Badges obtenus": 3,
            "Collisions évitées": random.randint(50, 200),
            "Distance parcourue": f"{random.randint(500, 2000)}m"
        }
        
        for key, value in stats.items():
            print(f"  {key}: {value}")
        
        print()
        print("🏆 Derniers accomplissements:")
        print("  • Scénario 'Suivi de ligne' complété")
        print("  • Badge 'Précision' obtenu")
        print("  • 10 heures d'utilisation atteintes")
        print()
        
    def run(self):
        """Lancer la démonstration"""
        self.show_banner()
        
        while True:
            self.show_menu()
            
            try:
                choice = input("Votre choix (0-6): ").strip()
                print()
                
                if choice == "0":
                    print("👋 Au revoir ! Merci d'avoir testé PiCarX Studio")
                    break
                elif choice == "1":
                    self.demo_control()
                elif choice == "2":
                    self.demo_sensors()
                elif choice == "3":
                    self.demo_code_execution()
                elif choice == "4":
                    self.demo_scenarios()
                elif choice == "5":
                    self.show_config()
                elif choice == "6":
                    self.show_stats()
                else:
                    print("❌ Choix invalide. Veuillez choisir entre 0 et 6.")
                    print()
                    
            except KeyboardInterrupt:
                print("\n\n👋 Interruption détectée. Au revoir !")
                break
            except Exception as e:
                print(f"❌ Erreur: {e}")
                print()
            
            if choice != "0":
                input("Appuyez sur Entrée pour continuer...")
                print("\n" + "=" * 60 + "\n")

def main():
    """Point d'entrée principal"""
    demo = PiCarXConsoleDemo()
    demo.run()

if __name__ == "__main__":
    main()