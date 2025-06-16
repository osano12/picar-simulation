#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Script de lancement avec debug détaillé
"""

import sys
import traceback
from PyQt6.QtWidgets import QApplication

def main():
    """Lancement avec debug"""
    print("=== Debug Launch PiCarX Studio ===")
    
    try:
        print("1. Création de QApplication...")
        app = QApplication(sys.argv)
        print("✅ QApplication créée")
        
        print("2. Import des modules...")
        from ui.main_window import MainWindow
        print("✅ MainWindow importée")
        
        print("3. Création de MainWindow...")
        window = MainWindow()
        print("✅ MainWindow créée")
        
        print("4. Affichage de la fenêtre...")
        window.show()
        print("✅ Fenêtre affichée")
        
        print("5. Lancement de la boucle d'événements...")
        return app.exec()
        
    except Exception as e:
        print(f"❌ ERREUR: {e}")
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    sys.exit(main())