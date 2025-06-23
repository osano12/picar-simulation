#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PiCarX Studio - Application Desktop pour Simulation et Contrôle du Robot PiCarX
Version: 1.0.0
Auteur: Équipe PiCarX
"""

import sys
import os
import logging
from PyQt6.QtWidgets import QApplication, QMainWindow, QSplashScreen, QMessageBox
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPixmap, QIcon, QPainter, QFont, QColor

# Ajouter le répertoire parent au path pour importer les modules du projet
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ui.main_window import MainWindow
from core.config import AppConfig
from core.logger import setup_logger

class PiCarXStudio:
    """Classe principale de l'application PiCarX Studio"""
    
    def __init__(self, logger=None, args=None):
        self.app = None
        self.main_window = None
        self.splash = None
        self.logger = logger or setup_logger()
        self.args = args
        
    def create_splash_screen(self):
        """Créer l'écran de démarrage"""
        # Créer un pixmap personnalisé pour le splash screen
        splash_pixmap = QPixmap(500, 350)
        splash_pixmap.fill(QColor(30, 30, 30))
        
        # Dessiner sur le splash screen
        painter = QPainter(splash_pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        # Titre
        painter.setPen(QColor(255, 255, 255))
        painter.setFont(QFont("Arial", 24, QFont.Weight.Bold))
        painter.drawText(50, 100, "PiCarX Studio")
        
        # Sous-titre
        painter.setFont(QFont("Arial", 12))
        painter.setPen(QColor(200, 200, 200))
        painter.drawText(50, 130, "Simulateur et Programmation Robotique")
        
        # Version
        painter.setFont(QFont("Arial", 10))
        painter.setPen(QColor(150, 150, 150))
        painter.drawText(50, 160, "Version 1.0.0")
        
        # Logo/Robot simple
        painter.setPen(QColor(100, 150, 255))
        painter.setBrush(QColor(100, 150, 255))
        painter.drawRect(350, 80, 80, 120)
        
        # Roues
        painter.setBrush(QColor(80, 80, 80))
        painter.drawRect(340, 100, 15, 40)
        painter.drawRect(445, 100, 15, 40)
        painter.drawRect(340, 160, 15, 40)
        painter.drawRect(445, 160, 15, 40)
        
        painter.end()
        
        self.splash = QSplashScreen(splash_pixmap)
        self.splash.setWindowFlags(Qt.WindowType.WindowStaysOnTopHint | Qt.WindowType.FramelessWindowHint)
        self.splash.show()
        
        # Messages de chargement
        self.splash.showMessage("Initialisation de PiCarX Studio...", 
                               Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignCenter, 
                               QColor(255, 255, 255))
        
    def initialize_app(self):
        """Initialiser l'application"""
        self.app = QApplication(sys.argv)
        self.app.setApplicationName("PiCarX Studio")
        self.app.setApplicationVersion("1.0.0")
        self.app.setOrganizationName("PiCarX Team")
        
        # Style sombre par défaut
        self.app.setStyleSheet("""
            QMainWindow {
                background-color: #2b2b2b;
                color: #ffffff;
            }
            QWidget {
                background-color: #2b2b2b;
                color: #ffffff;
            }
            QGroupBox {
                font-weight: bold;
                border: 2px solid #555555;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
        """)
        
        # Créer l'écran de démarrage
        self.create_splash_screen()
        
        # Traiter les événements pour afficher le splash
        self.app.processEvents()
        
    def load_components(self):
        """Charger les composants de l'application"""
        self.splash.showMessage("Chargement des composants...", 
                               Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignCenter, 
                               QColor(255, 255, 255))
        self.app.processEvents()
        
        # Charger la configuration
        try:
            config = AppConfig()
            config.create_directories()
        except Exception as e:
            self.logger.error(f"Erreur lors du chargement de la configuration: {e}")
        
        self.splash.showMessage("Initialisation de l'interface...", 
                               Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignCenter, 
                               QColor(255, 255, 255))
        self.app.processEvents()
        
        # Créer la fenêtre principale
        try:
            self.main_window = MainWindow()
        except Exception as e:
            self.logger.error(f"Erreur lors de la création de la fenêtre principale: {e}")
            QMessageBox.critical(None, "Erreur", f"Impossible de créer l'interface: {e}")
            return False
            
        return True
        
    def show_main_window(self):
        """Afficher la fenêtre principale"""
        self.splash.showMessage("Finalisation...", 
                               Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignCenter, 
                               QColor(255, 255, 255))
        self.app.processEvents()
        
        # Timer pour fermer le splash et afficher la fenêtre principale
        QTimer.singleShot(1500, self.finish_loading)
        
    def finish_loading(self):
        """Terminer le chargement et afficher l'interface principale"""
        if self.main_window:
            self.splash.finish(self.main_window)
            self.main_window.show()
            self.logger.info("PiCarX Studio démarré avec succès")
        else:
            self.splash.close()
        
    def run(self):
        """Lancer l'application"""
        try:
            self.initialize_app()
            
            if not self.load_components():
                return 1
                
            self.show_main_window()
            
            # Lancer la boucle d'événements
            return self.app.exec()
            
        except Exception as e:
            self.logger.error(f"Erreur lors du démarrage de l'application: {e}")
            if self.splash:
                self.splash.close()
            QMessageBox.critical(None, "Erreur Critique", 
                               f"Impossible de démarrer PiCarX Studio:\n{e}")
            return 1

def main(logger=None, args=None):
    """Point d'entrée principal"""
    # Vérifier les dépendances critiques
    try:
        from PyQt6.QtWidgets import QApplication
    except ImportError:
        print("ERREUR: PyQt6 n'est pas installé.")
        print("Installez-le avec: pip install PyQt6")
        return 1
        
    studio = PiCarXStudio(logger, args)
    return studio.run()

if __name__ == "__main__":
    sys.exit(main())