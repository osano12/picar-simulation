#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Système de logging pour PiCarX Studio
"""

import logging
import logging.handlers
import os
from pathlib import Path
from datetime import datetime

def setup_logger(name="PiCarXStudio", level=logging.INFO):
    """
    Configurer le système de logging
    
    Args:
        name: Nom du logger
        level: Niveau de logging
        
    Returns:
        Logger configuré
    """
    
    # Créer le répertoire de logs
    log_dir = Path.home() / ".picarx_studio" / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    
    # Nom du fichier de log avec timestamp
    log_filename = log_dir / f"picarx_studio_{datetime.now().strftime('%Y%m%d')}.log"
    
    # Créer le logger
    logger = logging.getLogger(name)
    logger.setLevel(level)
    
    # Éviter la duplication des handlers
    if logger.handlers:
        return logger
    
    # Format des messages
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    # Handler pour fichier avec rotation
    try:
        file_handler = logging.handlers.RotatingFileHandler(
            log_filename,
            maxBytes=10*1024*1024,  # 10MB
            backupCount=5,
            encoding='utf-8'
        )
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
    except Exception as e:
        print(f"Impossible de créer le fichier de log: {e}")
    
    # Handler pour console
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_formatter = logging.Formatter(
        '%(levelname)s - %(message)s'
    )
    console_handler.setFormatter(console_formatter)
    logger.addHandler(console_handler)
    
    return logger

class LoggerMixin:
    """Mixin pour ajouter facilement un logger à une classe"""
    
    @property
    def logger(self):
        if not hasattr(self, '_logger'):
            self._logger = logging.getLogger(f"{self.__class__.__module__}.{self.__class__.__name__}")
        return self._logger