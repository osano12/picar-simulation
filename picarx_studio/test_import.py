#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test d'importation des modules pour identifier les erreurs
"""

import sys
import traceback

def test_import(module_name):
    """Tester l'importation d'un module"""
    try:
        print(f"Test d'importation: {module_name}")
        __import__(module_name)
        print(f"✅ {module_name} - OK")
        return True
    except Exception as e:
        print(f"❌ {module_name} - ERREUR: {e}")
        traceback.print_exc()
        return False

def main():
    """Tester tous les modules"""
    modules = [
        'core.config',
        'core.logger',
        'ui.simulator_widget',
        'ui.code_editor',
        'ui.sensor_panel',
        'ui.control_panel',
        'ui.scenario_panel',
        'ui.main_window'
    ]
    
    print("=== Test d'importation des modules PiCarX Studio ===")
    
    success_count = 0
    for module in modules:
        if test_import(module):
            success_count += 1
        print()
    
    print(f"Résultat: {success_count}/{len(modules)} modules importés avec succès")
    
    if success_count == len(modules):
        print("✅ Tous les modules sont OK!")
    else:
        print("❌ Certains modules ont des erreurs")

if __name__ == "__main__":
    main()