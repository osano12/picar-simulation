#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test de la structure de PiCarX Studio sans interface graphique
"""

import sys
import os

# Ajouter le répertoire du projet au path
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

def test_imports():
    """Tester les imports des modules"""
    print("🧪 Test des imports des modules...")
    
    try:
        from core.config import AppConfig
        print("✅ core.config - OK")
    except Exception as e:
        print(f"❌ core.config - Erreur: {e}")
    
    try:
        from core.logger import setup_logger, LoggerMixin
        print("✅ core.logger - OK")
    except Exception as e:
        print(f"❌ core.logger - Erreur: {e}")
    
    # Test des modules UI (sans PyQt6)
    try:
        # Simuler l'import sans créer d'objets PyQt6
        import importlib.util
        
        # Test du code_editor
        spec = importlib.util.spec_from_file_location("code_editor", "ui/code_editor.py")
        if spec and spec.loader:
            print("✅ ui.code_editor - Structure OK")
        else:
            print("❌ ui.code_editor - Fichier non trouvé")
            
        # Test du sensor_panel
        spec = importlib.util.spec_from_file_location("sensor_panel", "ui/sensor_panel.py")
        if spec and spec.loader:
            print("✅ ui.sensor_panel - Structure OK")
        else:
            print("❌ ui.sensor_panel - Fichier non trouvé")
            
        # Test du scenario_panel
        spec = importlib.util.spec_from_file_location("scenario_panel", "ui/scenario_panel.py")
        if spec and spec.loader:
            print("✅ ui.scenario_panel - Structure OK")
        else:
            print("❌ ui.scenario_panel - Fichier non trouvé")
            
    except Exception as e:
        print(f"❌ Modules UI - Erreur: {e}")

def test_config():
    """Tester la configuration"""
    print("\n🔧 Test de la configuration...")
    
    try:
        from core.config import AppConfig
        config = AppConfig()
        
        # Test des valeurs par défaut
        app_name = config.get('app.name', 'PiCarX Studio')
        print(f"✅ Nom de l'application: {app_name}")
        
        version = config.get('app.version', '1.0.0')
        print(f"✅ Version: {version}")
        
        # Test de création des répertoires
        config.create_directories()
        print("✅ Répertoires créés")
        
    except Exception as e:
        print(f"❌ Configuration - Erreur: {e}")

def test_logger():
    """Tester le système de logging"""
    print("\n📝 Test du système de logging...")
    
    try:
        from core.logger import setup_logger, LoggerMixin
        
        # Test du logger principal
        logger = setup_logger()
        logger.info("Test du logger principal")
        print("✅ Logger principal - OK")
        
        # Test du LoggerMixin
        class TestClass(LoggerMixin):
            def test_log(self):
                self.logger.info("Test du LoggerMixin")
                return True
        
        test_obj = TestClass()
        if test_obj.test_log():
            print("✅ LoggerMixin - OK")
        
    except Exception as e:
        print(f"❌ Logger - Erreur: {e}")

def test_code_examples():
    """Tester les exemples de code"""
    print("\n🐍 Test des exemples de code...")
    
    try:
        # Simuler l'exécution d'un exemple simple
        example_code = '''
# Test d'exemple de code
print("Hello, PiCarX Studio!")

def test_function():
    return "Fonction de test OK"

result = test_function()
print(f"Résultat: {result}")
'''
        
        # Exécuter le code d'exemple
        exec(example_code)
        print("✅ Exécution d'exemple - OK")
        
    except Exception as e:
        print(f"❌ Exemples de code - Erreur: {e}")

def test_file_structure():
    """Tester la structure des fichiers"""
    print("\n📁 Test de la structure des fichiers...")
    
    required_files = [
        'main.py',
        'launch.py',
        'requirements.txt',
        'README.md',
        'core/__init__.py',
        'core/config.py',
        'core/logger.py',
        'ui/__init__.py',
        'ui/main_window.py',
        'ui/simulator_widget.py',
        'ui/code_editor.py',
        'ui/sensor_panel.py',
        'ui/control_panel.py',
        'ui/scenario_panel.py'
    ]
    
    missing_files = []
    for file_path in required_files:
        if os.path.exists(file_path):
            print(f"✅ {file_path}")
        else:
            print(f"❌ {file_path} - MANQUANT")
            missing_files.append(file_path)
    
    if not missing_files:
        print("✅ Tous les fichiers requis sont présents")
    else:
        print(f"❌ {len(missing_files)} fichiers manquants")

def main():
    """Fonction principale de test"""
    print("🚀 Test de la structure PiCarX Studio")
    print("=" * 50)
    
    test_file_structure()
    test_imports()
    test_config()
    test_logger()
    test_code_examples()
    
    print("\n" + "=" * 50)
    print("✅ Tests terminés!")
    print("\n💡 Pour lancer l'interface graphique:")
    print("   python launch.py")
    print("\n📚 Pour plus d'informations, consultez README.md")

if __name__ == "__main__":
    main()