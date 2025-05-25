#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PiCar X - Tutoriel 1: Premiers pas avec le robot
================================================

Ce tutoriel vous guide à travers les bases du contrôle du PiCar X:
1. Déplacement en ligne droite
2. Rotation
3. Utilisation des capteurs
4. Gestion des événements

Prérequis:
- Simulation PiCar X en cours d'exécution
- ROS 2 configuré correctement
"""

import rclpy
import time
from picar_api import PiCar


def main():
    # Initialiser ROS 2
    rclpy.init()
    
    # Créer une instance du PiCar
    print("Initialisation du PiCar X...")
    robot = PiCar()
    
    # Attendre que tous les capteurs soient initialisés
    time.sleep(2)
    
    print("\n=== Tutoriel 1: Premiers pas avec le PiCar X ===\n")
    
    # Étape 1: Déplacement en ligne droite
    print("Étape 1: Avancer de 50 cm")
    robot.drive(distance_cm=50, speed=20)
    print("✓ Déplacement terminé!")
    
    time.sleep(1)
    
    # Étape 2: Rotation
    print("\nÉtape 2: Tourner de 90 degrés")
    robot.turn(angle_deg=90)
    print("✓ Rotation terminée!")
    
    time.sleep(1)
    
    # Étape 3: Utilisation de la caméra
    print("\nÉtape 3: Orienter la caméra")
    print("  - Regarder à gauche...")
    robot.set_camera_position(pan=30, tilt=0)
    time.sleep(1)
    
    print("  - Regarder à droite...")
    robot.set_camera_position(pan=-30, tilt=0)
    time.sleep(1)
    
    print("  - Regarder en haut...")
    robot.set_camera_position(pan=0, tilt=15)
    time.sleep(1)
    
    print("  - Remettre la caméra au centre...")
    robot.set_camera_position(pan=0, tilt=0)
    print("✓ Orientation de la caméra terminée!")
    
    time.sleep(1)
    
    # Étape 4: Gestion des événements
    print("\nÉtape 4: Configuration des événements")
    
    @robot.on_ultrasonic_close(threshold_cm=20)
    def obstacle_detected():
        print("⚠️ Obstacle détecté! Distance: {:.1f} cm".format(
            robot.ultrasonic_distance * 100))
        robot.stop()
    
    @robot.on_battery_low(threshold_v=7.0)
    def battery_low():
        print("🔋 Batterie faible! Tension: {:.1f}V, Charge: {:.0f}%".format(
            robot.battery_voltage, robot.battery_percentage))
    
    print("✓ Événements configurés!")
    
    # Étape 5: Déplacement avec détection d'obstacles
    print("\nÉtape 5: Avancer jusqu'à détecter un obstacle")
    print("  Avancement en cours...")
    
    # Définir une vitesse constante
    robot.set_speed(linear_speed=15)
    
    # Continuer jusqu'à ce qu'un obstacle soit détecté
    # (Le callback obstacle_detected sera appelé automatiquement)
    try:
        # Attendre jusqu'à 10 secondes ou jusqu'à ce qu'un obstacle soit détecté
        timeout = time.time() + 10
        while time.time() < timeout and robot.ultrasonic_distance > 0.2:
            # Afficher la distance actuelle
            print("  Distance: {:.1f} cm".format(robot.ultrasonic_distance * 100), end="\r")
            time.sleep(0.1)
        
        # S'assurer que le robot est arrêté
        robot.stop()
        
        if robot.ultrasonic_distance <= 0.2:
            print("\n✓ Obstacle détecté et robot arrêté!")
        else:
            print("\n✓ Temps écoulé, aucun obstacle détecté.")
    
    except KeyboardInterrupt:
        # Arrêter le robot en cas d'interruption
        robot.stop()
        print("\nTutoriel interrompu par l'utilisateur.")
    
    # Conclusion
    print("\n=== Tutoriel terminé! ===")
    print("Vous avez appris à:")
    print("  1. Déplacer le robot en ligne droite")
    print("  2. Faire tourner le robot")
    print("  3. Orienter la caméra")
    print("  4. Configurer des événements")
    print("  5. Utiliser le capteur ultrason")
    
    # Nettoyer
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()