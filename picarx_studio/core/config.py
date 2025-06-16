#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Configuration de l'application PiCarX Studio
"""

import os
import json
import logging
from pathlib import Path
from typing import Dict, Any

class AppConfig:
    """Gestionnaire de configuration de l'application"""
    
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.config_dir = Path.home() / ".picarx_studio"
        self.config_file = self.config_dir / "config.json"
        self.default_config = self._get_default_config()
        self.config = self._load_config()
        
    def _get_default_config(self) -> Dict[str, Any]:
        """Configuration par défaut"""
        return {
            "app": {
                "version": "1.0.0",
                "language": "fr",
                "theme": "dark",
                "auto_save": True,
                "auto_save_interval": 300,  # 5 minutes
                "window_geometry": None,
                "window_state": None
            },
            "simulator": {
                "physics_enabled": True,
                "render_quality": "medium",  # low, medium, high
                "fps_limit": 60,
                "show_debug_info": True,
                "collision_detection": True,
                "show_sensors": True,
                "show_trail": True,
                "grid_size": 50
            },
            "robot": {
                "default_speed": 30,
                "turn_speed": 20,
                "sensor_update_rate": 10,  # Hz
                "battery_simulation": True,
                "realistic_physics": True,
                "robot_width": 40,
                "robot_height": 60
            },
            "interface": {
                "show_toolbar": True,
                "show_status_bar": True,
                "show_minimap": True,
                "panel_layout": "default",
                "font_size": 10,
                "splitter_sizes": [300, 600, 300]
            },
            "editor": {
                "syntax_highlighting": True,
                "auto_completion": True,
                "line_numbers": True,
                "word_wrap": False,
                "tab_size": 4,
                "font_family": "Consolas",
                "font_size": 11
            },
            "network": {
                "ros_enabled": False,
                "ros_master_uri": "http://localhost:11311",
                "web_interface_port": 8080,
                "api_port": 8081
            },
            "paths": {
                "projects_dir": str(Path.home() / "PiCarX_Projects"),
                "scenarios_dir": str(Path.home() / "PiCarX_Scenarios"),
                "exports_dir": str(Path.home() / "PiCarX_Exports"),
                "temp_dir": str(Path.home() / ".picarx_studio" / "temp")
            }
        }
    
    def _load_config(self) -> Dict[str, Any]:
        """Charger la configuration depuis le fichier"""
        try:
            if self.config_file.exists():
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    loaded_config = json.load(f)
                    # Fusionner avec la config par défaut pour les nouvelles clés
                    return self._merge_configs(self.default_config, loaded_config)
            else:
                # Créer le répertoire de config s'il n'existe pas
                self.config_dir.mkdir(exist_ok=True)
                self._save_config(self.default_config)
                return self.default_config.copy()
                
        except Exception as e:
            self.logger.error(f"Erreur lors du chargement de la configuration: {e}")
            return self.default_config.copy()
    
    def _merge_configs(self, default: Dict[str, Any], loaded: Dict[str, Any]) -> Dict[str, Any]:
        """Fusionner deux configurations"""
        result = default.copy()
        for key, value in loaded.items():
            if key in result and isinstance(result[key], dict) and isinstance(value, dict):
                result[key] = self._merge_configs(result[key], value)
            else:
                result[key] = value
        return result
    
    def _save_config(self, config: Dict[str, Any]):
        """Sauvegarder la configuration"""
        try:
            self.config_dir.mkdir(exist_ok=True)
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(config, f, indent=2, ensure_ascii=False)
        except Exception as e:
            self.logger.error(f"Erreur lors de la sauvegarde de la configuration: {e}")
    
    def get(self, key_path: str, default=None):
        """Récupérer une valeur de configuration avec notation pointée"""
        keys = key_path.split('.')
        value = self.config
        
        try:
            for key in keys:
                value = value[key]
            return value
        except (KeyError, TypeError):
            return default
    
    def set(self, key_path: str, value: Any):
        """Définir une valeur de configuration avec notation pointée"""
        keys = key_path.split('.')
        config = self.config
        
        # Naviguer jusqu'au dernier niveau
        for key in keys[:-1]:
            if key not in config:
                config[key] = {}
            config = config[key]
        
        # Définir la valeur
        config[keys[-1]] = value
        
        # Sauvegarder
        self.save()
    
    def save(self):
        """Sauvegarder la configuration actuelle"""
        self._save_config(self.config)
    
    def reset_to_default(self):
        """Réinitialiser à la configuration par défaut"""
        self.config = self.default_config.copy()
        self.save()
    
    def create_directories(self):
        """Créer les répertoires nécessaires"""
        directories = [
            self.get('paths.projects_dir'),
            self.get('paths.scenarios_dir'),
            self.get('paths.exports_dir'),
            self.get('paths.temp_dir')
        ]
        
        for directory in directories:
            if directory:
                try:
                    Path(directory).mkdir(parents=True, exist_ok=True)
                    self.logger.info(f"Répertoire créé/vérifié: {directory}")
                except Exception as e:
                    self.logger.error(f"Erreur lors de la création du répertoire {directory}: {e}")
    
    def get_config_dict(self) -> Dict[str, Any]:
        """Retourner une copie de la configuration complète"""
        return self.config.copy()