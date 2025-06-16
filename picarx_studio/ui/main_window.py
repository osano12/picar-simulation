#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Fenêtre principale de PiCarX Studio
"""

import sys
from PyQt6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                            QSplitter, QMenuBar, QToolBar, QStatusBar, QDockWidget,
                            QTabWidget, QTextEdit, QLabel, QPushButton,
                            QMessageBox, QFileDialog, QProgressBar, QApplication)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QSettings
from PyQt6.QtGui import QIcon, QKeySequence, QPixmap, QFont, QAction

from ui.simulator_widget import SimulatorWidget
from ui.code_editor import CodeEditor
from ui.sensor_panel import SensorPanel
from ui.control_panel import ControlPanel
from ui.scenario_panel import ScenarioPanel
from core.logger import LoggerMixin
from core.config import AppConfig

class MainWindow(QMainWindow, LoggerMixin):
    """Fenêtre principale de l'application"""
    
    # Signaux
    project_changed = pyqtSignal(str)
    simulation_started = pyqtSignal()
    simulation_stopped = pyqtSignal()
    
    def __init__(self):
        super().__init__()
        self.config = AppConfig()
        self.current_project = None
        self.simulation_running = False
        self.settings = QSettings()
        
        self.init_ui()
        self.setup_connections()
        self.restore_settings()
        
        self.logger.info("Fenêtre principale initialisée")
    
    def init_ui(self):
        """Initialiser l'interface utilisateur"""
        self.setWindowTitle("PiCarX Studio - Simulateur et Programmation Robotique")
        self.setMinimumSize(1200, 800)
        
        # Widget central avec splitter
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Layout principal
        main_layout = QHBoxLayout(central_widget)
        main_splitter = QSplitter(Qt.Orientation.Horizontal)
        main_layout.addWidget(main_splitter)
        
        # Panneau gauche (contrôles et capteurs)
        left_panel = self.create_left_panel()
        main_splitter.addWidget(left_panel)
        
        # Zone centrale (simulateur et éditeur)
        center_panel = self.create_center_panel()
        main_splitter.addWidget(center_panel)
        
        # Panneau droit (scénarios et propriétés)
        right_panel = self.create_right_panel()
        main_splitter.addWidget(right_panel)
        
        # Proportions du splitter
        splitter_sizes = self.config.get('interface.splitter_sizes', [300, 600, 300])
        main_splitter.setSizes(splitter_sizes)
        
        # Créer les menus et barres d'outils
        self.create_menus()
        self.create_toolbars()
        self.create_status_bar()
        
    def create_left_panel(self):
        """Créer le panneau de gauche"""
        left_dock = QDockWidget("Contrôles", self)
        left_dock.setFeatures(QDockWidget.DockWidgetFeature.DockWidgetMovable | 
                             QDockWidget.DockWidgetFeature.DockWidgetFloatable)
        
        # Widget conteneur avec onglets
        tab_widget = QTabWidget()
        
        # Panneau de contrôle
        self.control_panel = ControlPanel()
        tab_widget.addTab(self.control_panel, "🎮 Contrôle")
        
        # Panneau des capteurs
        self.sensor_panel = SensorPanel()
        tab_widget.addTab(self.sensor_panel, "📡 Capteurs")
        
        left_dock.setWidget(tab_widget)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, left_dock)
        
        return left_dock
    
    def create_center_panel(self):
        """Créer le panneau central"""
        center_widget = QWidget()
        center_layout = QVBoxLayout(center_widget)
        
        # Splitter vertical pour simulateur et éditeur
        center_splitter = QSplitter(Qt.Orientation.Vertical)
        center_layout.addWidget(center_splitter)
        
        # Widget simulateur
        self.simulator_widget = SimulatorWidget()
        center_splitter.addWidget(self.simulator_widget)
        
        # Onglets pour les éditeurs
        editor_tabs = QTabWidget()
        
        # Éditeur de code Python
        self.code_editor = CodeEditor()
        editor_tabs.addTab(self.code_editor, "🐍 Code Python")
        
        # Console de sortie
        self.console = QTextEdit()
        self.console.setMaximumHeight(150)
        self.console.setReadOnly(True)
        self.console.setPlaceholderText("Console de sortie - Les résultats d'exécution apparaîtront ici...")
        self.console.setStyleSheet("""
            QTextEdit {
                background-color: #1e1e1e;
                color: #ffffff;
                font-family: 'Consolas', 'Monaco', monospace;
                font-size: 10pt;
                border: 1px solid #555;
            }
        """)
        editor_tabs.addTab(self.console, "📟 Console")
        
        center_splitter.addWidget(editor_tabs)
        center_splitter.setSizes([500, 250])
        
        return center_widget
    
    def create_right_panel(self):
        """Créer le panneau de droite"""
        right_dock = QDockWidget("Scénarios", self)
        right_dock.setFeatures(QDockWidget.DockWidgetFeature.DockWidgetMovable | 
                              QDockWidget.DockWidgetFeature.DockWidgetFloatable)
        
        # Panneau des scénarios
        self.scenario_panel = ScenarioPanel()
        right_dock.setWidget(self.scenario_panel)
        
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, right_dock)
        
        return right_dock
    
    def create_menus(self):
        """Créer les menus"""
        menubar = self.menuBar()
        
        # Menu Fichier
        file_menu = menubar.addMenu("&Fichier")
        
        new_action = QAction("&Nouveau Projet", self)
        new_action.setShortcut(QKeySequence.StandardKey.New)
        new_action.setStatusTip("Créer un nouveau projet")
        new_action.triggered.connect(self.new_project)
        file_menu.addAction(new_action)
        
        open_action = QAction("&Ouvrir Projet", self)
        open_action.setShortcut(QKeySequence.StandardKey.Open)
        open_action.setStatusTip("Ouvrir un projet existant")
        open_action.triggered.connect(self.open_project)
        file_menu.addAction(open_action)
        
        save_action = QAction("&Sauvegarder", self)
        save_action.setShortcut(QKeySequence.StandardKey.Save)
        save_action.setStatusTip("Sauvegarder le projet actuel")
        save_action.triggered.connect(self.save_project)
        file_menu.addAction(save_action)
        
        file_menu.addSeparator()
        
        exit_action = QAction("&Quitter", self)
        exit_action.setShortcut(QKeySequence.StandardKey.Quit)
        exit_action.setStatusTip("Quitter l'application")
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        # Menu Simulation
        sim_menu = menubar.addMenu("&Simulation")
        
        self.start_sim_action = QAction("&Démarrer Simulation", self)
        self.start_sim_action.setShortcut("F5")
        self.start_sim_action.setStatusTip("Démarrer la simulation du robot")
        self.start_sim_action.triggered.connect(self.start_simulation)
        sim_menu.addAction(self.start_sim_action)
        
        self.stop_sim_action = QAction("&Arrêter Simulation", self)
        self.stop_sim_action.setShortcut("Shift+F5")
        self.stop_sim_action.setStatusTip("Arrêter la simulation du robot")
        self.stop_sim_action.triggered.connect(self.stop_simulation)
        self.stop_sim_action.setEnabled(False)
        sim_menu.addAction(self.stop_sim_action)
        
        sim_menu.addSeparator()
        
        reset_action = QAction("&Réinitialiser Robot", self)
        reset_action.setShortcut("Ctrl+R")
        reset_action.setStatusTip("Réinitialiser la position du robot")
        reset_action.triggered.connect(self.reset_robot)
        sim_menu.addAction(reset_action)
        
        # Menu Code
        code_menu = menubar.addMenu("&Code")
        
        run_code_action = QAction("&Exécuter Code", self)
        run_code_action.setShortcut("Ctrl+F5")
        run_code_action.setStatusTip("Exécuter le code Python")
        run_code_action.triggered.connect(self.run_code)
        code_menu.addAction(run_code_action)
        
        clear_console_action = QAction("&Effacer Console", self)
        clear_console_action.setShortcut("Ctrl+L")
        clear_console_action.setStatusTip("Effacer la console de sortie")
        clear_console_action.triggered.connect(self.clear_console)
        code_menu.addAction(clear_console_action)
        
        # Menu Outils
        tools_menu = menubar.addMenu("&Outils")
        
        settings_action = QAction("&Préférences", self)
        settings_action.setStatusTip("Ouvrir les préférences")
        settings_action.triggered.connect(self.show_preferences)
        tools_menu.addAction(settings_action)
        
        # Menu Aide
        help_menu = menubar.addMenu("&Aide")
        
        about_action = QAction("&À propos", self)
        about_action.setStatusTip("À propos de PiCarX Studio")
        about_action.triggered.connect(self.show_about)
        help_menu.addAction(about_action)
    
    def create_toolbars(self):
        """Créer les barres d'outils"""
        # Barre d'outils principale
        main_toolbar = self.addToolBar("Principal")
        main_toolbar.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextUnderIcon)
        
        # Boutons de fichier
        new_btn = main_toolbar.addAction("📄 Nouveau", self.new_project)
        new_btn.setStatusTip("Créer un nouveau projet")
        
        open_btn = main_toolbar.addAction("📂 Ouvrir", self.open_project)
        open_btn.setStatusTip("Ouvrir un projet")
        
        save_btn = main_toolbar.addAction("💾 Sauvegarder", self.save_project)
        save_btn.setStatusTip("Sauvegarder le projet")
        
        main_toolbar.addSeparator()
        
        # Boutons de simulation
        self.start_btn = main_toolbar.addAction("▶️ Démarrer", self.start_simulation)
        self.start_btn.setStatusTip("Démarrer la simulation")
        
        self.stop_btn = main_toolbar.addAction("⏹️ Arrêter", self.stop_simulation)
        self.stop_btn.setStatusTip("Arrêter la simulation")
        self.stop_btn.setEnabled(False)
        
        main_toolbar.addSeparator()
        
        # Bouton de réinitialisation
        reset_btn = main_toolbar.addAction("🔄 Reset", self.reset_robot)
        reset_btn.setStatusTip("Réinitialiser le robot")
        
        main_toolbar.addSeparator()
        
        # Bouton d'exécution de code
        run_btn = main_toolbar.addAction("🐍 Exécuter", self.run_code)
        run_btn.setStatusTip("Exécuter le code Python")
    
    def create_status_bar(self):
        """Créer la barre de statut"""
        self.status_bar = self.statusBar()
        
        # Label de statut principal
        self.status_label = QLabel("Prêt - PiCarX Studio initialisé")
        self.status_bar.addWidget(self.status_label)
        
        # Barre de progression
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        self.progress_bar.setMaximumWidth(200)
        self.status_bar.addPermanentWidget(self.progress_bar)
        
        # Indicateur de simulation
        self.sim_status_label = QLabel("Simulation: Arrêtée")
        self.sim_status_label.setStyleSheet("color: #ff6666;")
        self.status_bar.addPermanentWidget(self.sim_status_label)
        
        # Indicateur de connexion
        self.connection_label = QLabel("Mode: Simulation")
        self.connection_label.setStyleSheet("color: #66ff66;")
        self.status_bar.addPermanentWidget(self.connection_label)
    
    def setup_connections(self):
        """Configurer les connexions entre les composants"""
        # Connexions du panneau de contrôle
        self.control_panel.move_command.connect(self.simulator_widget.move_robot)
        self.control_panel.stop_command.connect(self.simulator_widget.stop_robot)
        self.control_panel.speed_changed.connect(self.simulator_widget.set_max_speed)
        
        # Connexions du simulateur
        self.simulator_widget.sensor_data_updated.connect(self.sensor_panel.update_sensors)
        self.simulator_widget.robot_position_changed.connect(self.update_robot_status)
        self.simulator_widget.collision_detected.connect(self.on_collision_detected)
        
        # Connexions des scénarios
        self.scenario_panel.scenario_selected.connect(self.load_scenario)
        self.scenario_panel.run_scenario.connect(self.run_scenario)
        
        # Connexions de l'éditeur de code
        self.code_editor.code_executed.connect(self.on_code_executed)
    
    def restore_settings(self):
        """Restaurer les paramètres de l'application"""
        # Restaurer la géométrie de la fenêtre
        geometry = self.settings.value("geometry")
        if geometry:
            self.restoreGeometry(geometry)
        
        # Restaurer l'état des docks
        state = self.settings.value("windowState")
        if state:
            self.restoreState(state)
        
        # Appliquer le thème
        theme = self.config.get('app.theme', 'dark')
        self.apply_theme(theme)
    
    def apply_theme(self, theme_name):
        """Appliquer un thème à l'interface"""
        if theme_name == 'dark':
            self.setStyleSheet("""
                QMainWindow {
                    background-color: #2b2b2b;
                    color: #ffffff;
                }
                QDockWidget {
                    background-color: #3c3c3c;
                    color: #ffffff;
                    titlebar-close-icon: url(close.png);
                    titlebar-normal-icon: url(undock.png);
                }
                QDockWidget::title {
                    text-align: left;
                    background-color: #4a4a4a;
                    padding-left: 5px;
                }
                QTabWidget::pane {
                    border: 1px solid #555555;
                    background-color: #2b2b2b;
                }
                QTabBar::tab {
                    background-color: #3c3c3c;
                    color: #ffffff;
                    padding: 8px 16px;
                    margin-right: 2px;
                    border-top-left-radius: 4px;
                    border-top-right-radius: 4px;
                }
                QTabBar::tab:selected {
                    background-color: #4a4a4a;
                    border-bottom: 2px solid #0078d4;
                }
                QTabBar::tab:hover {
                    background-color: #555555;
                }
                QMenuBar {
                    background-color: #3c3c3c;
                    color: #ffffff;
                }
                QMenuBar::item:selected {
                    background-color: #4a4a4a;
                }
                QMenu {
                    background-color: #3c3c3c;
                    color: #ffffff;
                    border: 1px solid #555555;
                }
                QMenu::item:selected {
                    background-color: #4a4a4a;
                }
                QToolBar {
                    background-color: #3c3c3c;
                    border: none;
                    spacing: 3px;
                }
                QStatusBar {
                    background-color: #3c3c3c;
                    color: #ffffff;
                }
            """)
    
    # Slots pour les actions
    def new_project(self):
        """Créer un nouveau projet"""
        self.logger.info("Création d'un nouveau projet")
        self.status_label.setText("Nouveau projet créé")
        self.code_editor.clear()
        self.console.clear()
        
    def open_project(self):
        """Ouvrir un projet existant"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Ouvrir un projet", 
            self.config.get('paths.projects_dir'),
            "Projets PiCarX (*.pcx);;Fichiers Python (*.py);;Tous les fichiers (*)"
        )
        if file_path:
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                    self.code_editor.setPlainText(content)
                self.logger.info(f"Projet ouvert: {file_path}")
                self.status_label.setText(f"Projet ouvert: {file_path}")
                self.current_project = file_path
            except Exception as e:
                QMessageBox.warning(self, "Erreur", f"Impossible d'ouvrir le fichier:\n{e}")
    
    def save_project(self):
        """Sauvegarder le projet actuel"""
        if self.current_project:
            try:
                with open(self.current_project, 'w', encoding='utf-8') as f:
                    f.write(self.code_editor.toPlainText())
                self.logger.info("Projet sauvegardé")
                self.status_label.setText("Projet sauvegardé")
            except Exception as e:
                QMessageBox.warning(self, "Erreur", f"Impossible de sauvegarder:\n{e}")
        else:
            # Sauvegarder sous
            file_path, _ = QFileDialog.getSaveFileName(
                self, "Sauvegarder le projet",
                self.config.get('paths.projects_dir'),
                "Fichiers Python (*.py);;Projets PiCarX (*.pcx)"
            )
            if file_path:
                try:
                    with open(file_path, 'w', encoding='utf-8') as f:
                        f.write(self.code_editor.toPlainText())
                    self.current_project = file_path
                    self.logger.info(f"Projet sauvegardé: {file_path}")
                    self.status_label.setText(f"Projet sauvegardé: {file_path}")
                except Exception as e:
                    QMessageBox.warning(self, "Erreur", f"Impossible de sauvegarder:\n{e}")
    
    def start_simulation(self):
        """Démarrer la simulation"""
        self.simulation_running = True
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(False)
        self.start_sim_action.setEnabled(False)
        self.stop_sim_action.setEnabled(True)
        
        self.simulator_widget.start_simulation()
        self.status_label.setText("Simulation démarrée")
        self.sim_status_label.setText("Simulation: Active")
        self.sim_status_label.setStyleSheet("color: #66ff66;")
        
        self.simulation_started.emit()
        self.logger.info("Simulation démarrée")
    
    def stop_simulation(self):
        """Arrêter la simulation"""
        self.simulation_running = False
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.start_sim_action.setEnabled(True)
        self.stop_sim_action.setEnabled(False)
        
        self.simulator_widget.stop_simulation()
        self.status_label.setText("Simulation arrêtée")
        self.sim_status_label.setText("Simulation: Arrêtée")
        self.sim_status_label.setStyleSheet("color: #ff6666;")
        
        self.simulation_stopped.emit()
        self.logger.info("Simulation arrêtée")
    
    def reset_robot(self):
        """Réinitialiser le robot"""
        self.simulator_widget.reset_robot()
        self.status_label.setText("Robot réinitialisé")
        self.logger.info("Robot réinitialisé")
    
    def run_code(self):
        """Exécuter le code Python"""
        code = self.code_editor.toPlainText()
        if code.strip():
            self.console.append(">>> Exécution du code...")
            self.code_editor.execute_code(code)
        else:
            self.console.append(">>> Aucun code à exécuter")
    
    def clear_console(self):
        """Effacer la console"""
        self.console.clear()
        self.status_label.setText("Console effacée")
    
    def load_scenario(self, scenario_id):
        """Charger un scénario"""
        self.logger.info(f"Chargement du scénario: {scenario_id}")
        self.status_label.setText(f"Scénario chargé: {scenario_id}")
    
    def run_scenario(self, scenario_id):
        """Exécuter un scénario"""
        self.logger.info(f"Exécution du scénario: {scenario_id}")
        self.status_label.setText(f"Exécution: {scenario_id}")
    
    def update_robot_status(self, position):
        """Mettre à jour le statut du robot"""
        x, y, theta = position
        self.status_label.setText(f"Robot: ({x:.1f}, {y:.1f}, {theta:.1f}°)")
    
    def on_collision_detected(self, collision_type):
        """Gérer les collisions"""
        self.console.append(f"⚠️ Collision détectée: {collision_type}")
        self.status_label.setText(f"Collision: {collision_type}")
    
    def on_code_executed(self, result):
        """Gérer les résultats d'exécution de code"""
        if result['success']:
            self.console.append("✅ Code exécuté avec succès")
            if result['output']:
                self.console.append(result['output'])
        else:
            self.console.append("❌ Erreur d'exécution:")
            self.console.append(result['error'])
    
    def show_preferences(self):
        """Afficher les préférences"""
        QMessageBox.information(self, "Préférences", 
                               "Fenêtre de préférences à implémenter\n\n"
                               "Fonctionnalités prévues:\n"
                               "• Thèmes d'interface\n"
                               "• Paramètres de simulation\n"
                               "• Configuration des capteurs\n"
                               "• Raccourcis clavier")
    
    def show_about(self):
        """Afficher la boîte À propos"""
        QMessageBox.about(self, "À propos de PiCarX Studio", 
                         "<h2>PiCarX Studio v1.0.0</h2>"
                         "<p>Simulateur et environnement de programmation<br>"
                         "pour le robot PiCarX</p>"
                         "<p><b>Fonctionnalités:</b></p>"
                         "<ul>"
                         "<li>Simulation 2D interactive</li>"
                         "<li>Programmation Python intégrée</li>"
                         "<li>Capteurs simulés réalistes</li>"
                         "<li>Scénarios d'apprentissage</li>"
                         "<li>Interface moderne et intuitive</li>"
                         "</ul>"
                         "<p>© 2024 Équipe PiCarX</p>")
    
    def closeEvent(self, event):
        """Gérer la fermeture de l'application"""
        # Sauvegarder les paramètres de fenêtre
        self.settings.setValue("geometry", self.saveGeometry())
        self.settings.setValue("windowState", self.saveState())
        
        # Demander confirmation si du code non sauvegardé
        if self.code_editor.toPlainText().strip() and not self.current_project:
            reply = QMessageBox.question(self, 'Code non sauvegardé', 
                                       'Vous avez du code non sauvegardé. Voulez-vous le sauvegarder avant de quitter?',
                                       QMessageBox.StandardButton.Save | 
                                       QMessageBox.StandardButton.Discard | 
                                       QMessageBox.StandardButton.Cancel,
                                       QMessageBox.StandardButton.Save)
            
            if reply == QMessageBox.StandardButton.Save:
                self.save_project()
            elif reply == QMessageBox.StandardButton.Cancel:
                event.ignore()
                return
        
        # Arrêter la simulation si active
        if self.simulation_running:
            self.stop_simulation()
        
        # Sauvegarder la configuration
        self.config.save()
        self.logger.info("Application fermée")
        event.accept()