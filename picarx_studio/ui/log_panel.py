#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Panneau de logs en temps réel pour PiCarX Studio
"""

import logging
import time
from datetime import datetime
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QTextBrowser, 
                            QPushButton, QLabel, QComboBox, QCheckBox, QGroupBox,
                            QFileDialog, QMessageBox)
from PyQt6.QtCore import Qt, pyqtSignal, QTimer, QThread
from PyQt6.QtGui import QFont, QColor, QTextCharFormat, QTextCursor

class LogHandler(logging.Handler):
    """Handler personnalisé pour capturer les logs"""
    
    def __init__(self, log_panel):
        super().__init__()
        self.log_panel = log_panel
        
    def emit(self, record):
        """Émettre un log vers le panneau"""
        try:
            msg = self.format(record)
            self.log_panel.add_log_message(record.levelname, msg, record.created)
        except Exception:
            self.handleError(record)

class LogPanel(QWidget):
    """Panneau de logs en temps réel"""
    
    # Signaux
    log_message_added = pyqtSignal(str, str, float)  # level, message, timestamp
    
    def __init__(self):
        super().__init__()
        self.log_messages = []
        self.max_messages = 1000
        self.auto_scroll = True
        self.show_timestamps = True
        self.current_filter = "ALL"
        
        self.init_ui()
        self.setup_logging()
        
        # Timer pour mettre à jour l'affichage
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(100)  # Mise à jour toutes les 100ms
        
    def init_ui(self):
        """Initialiser l'interface utilisateur"""
        layout = QVBoxLayout(self)
        
        # Groupe de contrôles
        controls_group = self.create_controls_group()
        layout.addWidget(controls_group)
        
        # Zone d'affichage des logs
        self.log_display = QTextBrowser()
        self.log_display.setFont(QFont("Consolas", 9))
        self.log_display.setStyleSheet("""
            QTextBrowser {
                background-color: #1a1a1a;
                color: #e0e0e0;
                border: 1px solid #555;
                border-radius: 5px;
                padding: 5px;
            }
        """)
        layout.addWidget(self.log_display)
        
        # Barre de statut
        status_layout = QHBoxLayout()
        self.status_label = QLabel("Logs en temps réel - Prêt")
        self.status_label.setStyleSheet("color: #888; font-size: 10px;")
        status_layout.addWidget(self.status_label)
        
        status_layout.addStretch()
        
        self.message_count_label = QLabel("Messages: 0")
        self.message_count_label.setStyleSheet("color: #888; font-size: 10px;")
        status_layout.addWidget(self.message_count_label)
        
        layout.addLayout(status_layout)
        
    def create_controls_group(self):
        """Créer le groupe de contrôles"""
        group = QGroupBox("📋 Contrôles des Logs")
        layout = QHBoxLayout(group)
        
        # Filtre par niveau
        layout.addWidget(QLabel("Niveau:"))
        self.level_combo = QComboBox()
        self.level_combo.addItems(["ALL", "DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"])
        self.level_combo.setStyleSheet("""
            QComboBox {
                padding: 3px;
                border: 1px solid #555;
                border-radius: 3px;
                background-color: #3a3a3a;
                color: white;
                min-width: 80px;
            }
            QComboBox QAbstractItemView {
                background-color: #3a3a3a;
                color: white;
                selection-background-color: #0078d4;
            }
        """)
        self.level_combo.currentTextChanged.connect(self.set_filter_level)
        layout.addWidget(self.level_combo)
        
        layout.addWidget(QLabel("|"))
        
        # Options d'affichage
        self.timestamp_checkbox = QCheckBox("Horodatage")
        self.timestamp_checkbox.setChecked(True)
        self.timestamp_checkbox.toggled.connect(self.toggle_timestamps)
        layout.addWidget(self.timestamp_checkbox)
        
        self.autoscroll_checkbox = QCheckBox("Défilement auto")
        self.autoscroll_checkbox.setChecked(True)
        self.autoscroll_checkbox.toggled.connect(self.toggle_autoscroll)
        layout.addWidget(self.autoscroll_checkbox)
        
        layout.addWidget(QLabel("|"))
        
        # Boutons d'action
        self.btn_clear = QPushButton("🗑️ Effacer")
        self.btn_clear.setStyleSheet("""
            QPushButton {
                background-color: #aa4444;
                color: white;
                font-weight: bold;
                padding: 6px 12px;
                border: none;
                border-radius: 3px;
            }
            QPushButton:hover {
                background-color: #bb5555;
            }
        """)
        self.btn_clear.clicked.connect(self.clear_logs)
        layout.addWidget(self.btn_clear)
        
        self.btn_export = QPushButton("💾 Exporter")
        self.btn_export.setStyleSheet("""
            QPushButton {
                background-color: #4444aa;
                color: white;
                font-weight: bold;
                padding: 6px 12px;
                border: none;
                border-radius: 3px;
            }
            QPushButton:hover {
                background-color: #5555bb;
            }
        """)
        self.btn_export.clicked.connect(self.export_logs)
        layout.addWidget(self.btn_export)
        
        self.btn_pause = QPushButton("⏸️ Pause")
        self.btn_pause.setCheckable(True)
        self.btn_pause.setStyleSheet("""
            QPushButton {
                background-color: #aa8844;
                color: white;
                font-weight: bold;
                padding: 6px 12px;
                border: none;
                border-radius: 3px;
            }
            QPushButton:hover {
                background-color: #bb9955;
            }
            QPushButton:checked {
                background-color: #cc6644;
            }
        """)
        self.btn_pause.toggled.connect(self.toggle_pause)
        layout.addWidget(self.btn_pause)
        
        layout.addStretch()
        
        return group
        
    def setup_logging(self):
        """Configurer la capture des logs"""
        # Créer un handler personnalisé
        self.log_handler = LogHandler(self)
        self.log_handler.setLevel(logging.DEBUG)
        
        # Format des messages
        formatter = logging.Formatter('%(name)s - %(levelname)s - %(message)s')
        self.log_handler.setFormatter(formatter)
        
        # Ajouter le handler au logger principal
        root_logger = logging.getLogger()
        root_logger.addHandler(self.log_handler)
        
        # Logger spécifique pour PiCarX Studio
        self.studio_logger = logging.getLogger('PiCarXStudio')
        
        # Ajouter quelques messages de test
        self.add_test_messages()
        
    def add_test_messages(self):
        """Ajouter des messages de test"""
        test_messages = [
            ("INFO", "PiCarX Studio démarré avec succès", time.time()),
            ("DEBUG", "Initialisation des composants UI", time.time() + 0.1),
            ("INFO", "Simulateur initialisé", time.time() + 0.2),
            ("WARNING", "Capteur ultrasonique non calibré", time.time() + 0.3),
            ("INFO", "Connexion au robot établie", time.time() + 0.4),
            ("DEBUG", "Boucle de simulation démarrée", time.time() + 0.5),
        ]
        
        for level, message, timestamp in test_messages:
            self.add_log_message(level, message, timestamp)
    
    def add_log_message(self, level, message, timestamp):
        """Ajouter un message de log"""
        if self.btn_pause.isChecked():
            return
            
        # Ajouter à la liste
        log_entry = {
            'level': level,
            'message': message,
            'timestamp': timestamp,
            'formatted_time': datetime.fromtimestamp(timestamp).strftime('%H:%M:%S.%f')[:-3]
        }
        
        self.log_messages.append(log_entry)
        
        # Limiter le nombre de messages
        if len(self.log_messages) > self.max_messages:
            self.log_messages.pop(0)
            
        # Mettre à jour le compteur
        self.message_count_label.setText(f"Messages: {len(self.log_messages)}")
        
    def update_display(self):
        """Mettre à jour l'affichage des logs"""
        if self.btn_pause.isChecked():
            return
            
        # Sauvegarder la position du curseur
        cursor = self.log_display.textCursor()
        at_end = cursor.atEnd()
        
        # Effacer et reconstruire l'affichage
        self.log_display.clear()
        
        # Filtrer les messages
        filtered_messages = self.filter_messages()
        
        # Afficher les messages
        for entry in filtered_messages[-200:]:  # Afficher seulement les 200 derniers
            self.append_formatted_message(entry)
            
        # Auto-scroll si activé et si on était à la fin
        if self.auto_scroll and at_end:
            cursor = self.log_display.textCursor()
            cursor.movePosition(QTextCursor.MoveOperation.End)
            self.log_display.setTextCursor(cursor)
            
    def filter_messages(self):
        """Filtrer les messages selon le niveau sélectionné"""
        if self.current_filter == "ALL":
            return self.log_messages
            
        level_priority = {
            'DEBUG': 10,
            'INFO': 20,
            'WARNING': 30,
            'ERROR': 40,
            'CRITICAL': 50
        }
        
        min_level = level_priority.get(self.current_filter, 0)
        
        return [msg for msg in self.log_messages 
                if level_priority.get(msg['level'], 0) >= min_level]
    
    def append_formatted_message(self, entry):
        """Ajouter un message formaté à l'affichage"""
        cursor = self.log_display.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.End)
        
        # Format de base
        format = QTextCharFormat()
        
        # Couleur selon le niveau
        level_colors = {
            'DEBUG': QColor(150, 150, 150),
            'INFO': QColor(100, 200, 255),
            'WARNING': QColor(255, 200, 100),
            'ERROR': QColor(255, 100, 100),
            'CRITICAL': QColor(255, 50, 50)
        }
        
        format.setForeground(level_colors.get(entry['level'], QColor(200, 200, 200)))
        
        # Construire le texte
        text_parts = []
        
        if self.show_timestamps:
            text_parts.append(f"[{entry['formatted_time']}]")
            
        text_parts.append(f"[{entry['level']}]")
        text_parts.append(entry['message'])
        
        text = " ".join(text_parts)
        
        # Insérer le texte avec le format
        cursor.insertText(text + "\n", format)
        
    def set_filter_level(self, level):
        """Définir le niveau de filtrage"""
        self.current_filter = level
        self.status_label.setText(f"Filtre: {level}")
        
    def toggle_timestamps(self, enabled):
        """Basculer l'affichage des horodatages"""
        self.show_timestamps = enabled
        
    def toggle_autoscroll(self, enabled):
        """Basculer le défilement automatique"""
        self.auto_scroll = enabled
        
    def toggle_pause(self, paused):
        """Basculer la pause"""
        if paused:
            self.btn_pause.setText("▶️ Reprendre")
            self.status_label.setText("Logs en pause")
        else:
            self.btn_pause.setText("⏸️ Pause")
            self.status_label.setText("Logs en temps réel - Actif")
            
    def clear_logs(self):
        """Effacer tous les logs"""
        self.log_messages.clear()
        self.log_display.clear()
        self.message_count_label.setText("Messages: 0")
        self.status_label.setText("Logs effacés")
        
        # Ajouter un message de confirmation
        self.add_log_message("INFO", "Logs effacés par l'utilisateur", time.time())
        
    def export_logs(self):
        """Exporter les logs vers un fichier"""
        if not self.log_messages:
            QMessageBox.information(self, "Export", "Aucun log à exporter.")
            return
            
        filename, _ = QFileDialog.getSaveFileName(
            self, "Exporter les logs", 
            f"picarx_logs_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt",
            "Fichiers texte (*.txt);;Tous les fichiers (*)"
        )
        
        if filename:
            try:
                with open(filename, 'w', encoding='utf-8') as f:
                    f.write(f"# Logs PiCarX Studio - Exporté le {datetime.now()}\n")
                    f.write(f"# Total: {len(self.log_messages)} messages\n\n")
                    
                    for entry in self.log_messages:
                        timestamp = datetime.fromtimestamp(entry['timestamp']).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                        f.write(f"[{timestamp}] [{entry['level']}] {entry['message']}\n")
                        
                QMessageBox.information(self, "Export", f"Logs exportés vers:\n{filename}")
                self.add_log_message("INFO", f"Logs exportés vers {filename}", time.time())
                
            except Exception as e:
                QMessageBox.critical(self, "Erreur", f"Impossible d'exporter les logs:\n{e}")
                self.add_log_message("ERROR", f"Erreur d'export: {e}", time.time())
    
    def log_info(self, message):
        """Ajouter un message INFO"""
        self.add_log_message("INFO", message, time.time())
        
    def log_warning(self, message):
        """Ajouter un message WARNING"""
        self.add_log_message("WARNING", message, time.time())
        
    def log_error(self, message):
        """Ajouter un message ERROR"""
        self.add_log_message("ERROR", message, time.time())
        
    def log_debug(self, message):
        """Ajouter un message DEBUG"""
        self.add_log_message("DEBUG", message, time.time())