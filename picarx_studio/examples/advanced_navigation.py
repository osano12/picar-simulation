#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Exemple Avancé - Navigation Intelligente avec PiCarX
Démontre l'utilisation des capteurs pour une navigation autonome
"""

import random
import time
import math

class PiCarXAdvanced:
    """Classe avancée pour la navigation intelligente du PiCarX"""
    
    def __init__(self):
        self.position = {'x': 0, 'y': 0, 'theta': 0}
        self.speed = 30
        self.sensor_data = {}
        self.path_history = []
        self.obstacles_detected = []
        
    def read_sensors(self):
        """Lire tous les capteurs du robot"""
        # Simulation des capteurs
        self.sensor_data = {
            'ultrasonic': random.randint(10, 200),
            'ir_left': random.choice([True, False]),
            'ir_right': random.choice([True, False]),
            'imu': {
                'angular_velocity': {'z': random.uniform(-50, 50)},
                'linear_acceleration': {'x': random.uniform(-5, 5)}
            },
            'battery': {'percentage': random.randint(70, 100)},
            'position': self.position.copy()
        }
        return self.sensor_data
    
    def move_forward(self, distance):
        """Avancer d'une distance donnée"""
        print(f"🤖 Avancer de {distance} unités")
        
        # Calculer nouvelle position
        rad = math.radians(self.position['theta'])
        self.position['x'] += distance * math.cos(rad)
        self.position['y'] += distance * math.sin(rad)
        
        # Ajouter à l'historique
        self.path_history.append(self.position.copy())
        
        return True
    
    def turn(self, angle):
        """Tourner d'un angle donné (en degrés)"""
        print(f"🔄 Tourner de {angle}°")
        self.position['theta'] = (self.position['theta'] + angle) % 360
        return True
    
    def stop(self):
        """Arrêter le robot"""
        print("⏹️ Arrêt du robot")
        return True
    
    def analyze_environment(self):
        """Analyser l'environnement avec les capteurs"""
        sensors = self.read_sensors()
        
        analysis = {
            'obstacle_ahead': sensors['ultrasonic'] < 50,
            'obstacle_close': sensors['ultrasonic'] < 20,
            'line_detected': sensors['ir_left'] or sensors['ir_right'],
            'on_line': sensors['ir_left'] and sensors['ir_right'],
            'line_left': sensors['ir_left'] and not sensors['ir_right'],
            'line_right': not sensors['ir_left'] and sensors['ir_right'],
            'battery_low': sensors['battery']['percentage'] < 20
        }
        
        return analysis
    
    def obstacle_avoidance_behavior(self):
        """Comportement d'évitement d'obstacles"""
        print("🚧 Mode évitement d'obstacles activé")
        
        analysis = self.analyze_environment()
        
        if analysis['obstacle_close']:
            print("🚨 Obstacle très proche - Manœuvre d'urgence")
            self.stop()
            self.turn(-90)  # Tourner à gauche
            self.move_forward(20)
            self.turn(90)   # Revenir à la direction originale
            return True
        
        elif analysis['obstacle_ahead']:
            print("⚠️ Obstacle détecté - Contournement")
            self.turn(45)   # Tourner légèrement
            self.move_forward(15)
            self.turn(-45)  # Corriger la direction
            return True
        
        return False
    
    def line_following_behavior(self):
        """Comportement de suivi de ligne"""
        print("🛤️ Mode suivi de ligne activé")
        
        analysis = self.analyze_environment()
        
        if analysis['on_line']:
            print("✅ Sur la ligne - Avancer")
            self.move_forward(10)
            return True
        
        elif analysis['line_left']:
            print("↩️ Ligne à gauche - Correction")
            self.turn(-15)
            self.move_forward(5)
            return True
        
        elif analysis['line_right']:
            print("↪️ Ligne à droite - Correction")
            self.turn(15)
            self.move_forward(5)
            return True
        
        else:
            print("🔍 Ligne perdue - Recherche")
            self.turn(30)  # Chercher la ligne
            return False
    
    def exploration_behavior(self):
        """Comportement d'exploration libre"""
        print("🗺️ Mode exploration activé")
        
        # Mouvement aléatoire intelligent
        actions = [
            ('forward', 20),
            ('forward', 15),
            ('turn', 45),
            ('turn', -45),
            ('turn', 90)
        ]
        
        action, value = random.choice(actions)
        
        if action == 'forward':
            self.move_forward(value)
        elif action == 'turn':
            self.turn(value)
        
        return True
    
    def intelligent_navigation(self, mode='auto', duration=30):
        """Navigation intelligente avec sélection automatique du comportement"""
        print(f"🧠 Navigation intelligente - Mode: {mode}")
        print("=" * 50)
        
        start_time = time.time()
        step = 0
        
        while time.time() - start_time < duration:
            step += 1
            print(f"\n--- Étape {step} ---")
            
            # Lire les capteurs
            sensors = self.read_sensors()
            analysis = self.analyze_environment()
            
            # Afficher l'état actuel
            print(f"Position: ({self.position['x']:.1f}, {self.position['y']:.1f}, {self.position['theta']:.0f}°)")
            print(f"Capteurs: Ultrason={sensors['ultrasonic']}cm, IR_L={sensors['ir_left']}, IR_R={sensors['ir_right']}")
            print(f"Batterie: {sensors['battery']['percentage']}%")
            
            # Vérifier la batterie
            if analysis['battery_low']:
                print("🔋 Batterie faible - Retour à la base recommandé")
                break
            
            # Sélection du comportement selon la priorité
            behavior_executed = False
            
            # 1. Priorité maximale : évitement d'obstacles
            if analysis['obstacle_ahead']:
                behavior_executed = self.obstacle_avoidance_behavior()
            
            # 2. Priorité moyenne : suivi de ligne
            elif analysis['line_detected'] and mode in ['auto', 'line']:
                behavior_executed = self.line_following_behavior()
            
            # 3. Priorité faible : exploration
            elif mode in ['auto', 'explore']:
                behavior_executed = self.exploration_behavior()
            
            # Pause entre les actions
            time.sleep(0.5)
        
        print(f"\n🏁 Navigation terminée après {step} étapes")
        self.show_navigation_summary()
    
    def show_navigation_summary(self):
        """Afficher un résumé de la navigation"""
        print("\n📊 Résumé de Navigation")
        print("-" * 30)
        
        if self.path_history:
            total_distance = 0
            for i in range(1, len(self.path_history)):
                prev = self.path_history[i-1]
                curr = self.path_history[i]
                dx = curr['x'] - prev['x']
                dy = curr['y'] - prev['y']
                total_distance += math.sqrt(dx*dx + dy*dy)
            
            print(f"Distance parcourue: {total_distance:.1f} unités")
            print(f"Position finale: ({self.position['x']:.1f}, {self.position['y']:.1f})")
            print(f"Orientation finale: {self.position['theta']:.0f}°")
            print(f"Points de passage: {len(self.path_history)}")
        
        print(f"Obstacles détectés: {len(self.obstacles_detected)}")
        print(f"Niveau de batterie: {self.sensor_data.get('battery', {}).get('percentage', 'N/A')}%")
    
    def demo_scenarios(self):
        """Démonstration de différents scénarios"""
        scenarios = [
            ('Exploration libre', 'explore', 10),
            ('Suivi de ligne', 'line', 8),
            ('Navigation mixte', 'auto', 12)
        ]
        
        for name, mode, duration in scenarios:
            print(f"\n🎯 Scénario: {name}")
            print("=" * 60)
            
            # Réinitialiser la position
            self.position = {'x': 0, 'y': 0, 'theta': 0}
            self.path_history = []
            
            # Exécuter le scénario
            self.intelligent_navigation(mode, duration)
            
            input("\nAppuyez sur Entrée pour le scénario suivant...")

def main():
    """Fonction principale de démonstration"""
    print("🤖 PiCarX Studio - Navigation Intelligente Avancée")
    print("=" * 60)
    
    robot = PiCarXAdvanced()
    
    print("\nChoisissez un mode de démonstration:")
    print("1. 🗺️ Exploration libre (10 étapes)")
    print("2. 🛤️ Suivi de ligne (8 étapes)")
    print("3. 🧠 Navigation intelligente (12 étapes)")
    print("4. 🎯 Tous les scénarios")
    print("0. 🚪 Quitter")
    
    try:
        choice = input("\nVotre choix (0-4): ").strip()
        
        if choice == "1":
            robot.intelligent_navigation('explore', 10)
        elif choice == "2":
            robot.intelligent_navigation('line', 8)
        elif choice == "3":
            robot.intelligent_navigation('auto', 12)
        elif choice == "4":
            robot.demo_scenarios()
        elif choice == "0":
            print("👋 Au revoir !")
        else:
            print("❌ Choix invalide")
            
    except KeyboardInterrupt:
        print("\n\n👋 Interruption détectée. Au revoir !")
    except Exception as e:
        print(f"\n❌ Erreur: {e}")

if __name__ == "__main__":
    main()