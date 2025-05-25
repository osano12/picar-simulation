@echo off
REM Script d'installation pour Windows pour le PiCar X Simulator

echo === Installation du PiCar X Simulator pour Windows ===
echo Ce script va installer toutes les dépendances nécessaires pour le simulateur.
echo L'installation peut prendre plusieurs minutes.
echo.

REM Vérifier si PowerShell est disponible
where powershell >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo Erreur: PowerShell n'est pas disponible sur ce système.
    exit /b 1
)

REM Vérifier si Chocolatey est installé
powershell -Command "if (-not (Get-Command choco -ErrorAction SilentlyContinue)) { Write-Host 'Installation de Chocolatey...'; Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://chocolatey.org/install.ps1')) }"

REM Installer les dépendances de base
echo Installation des dépendances de base...
call choco install -y git python3 cmake wget

REM Installer Docker pour exécuter ROS 2
echo Installation de Docker pour exécuter ROS 2...
call choco install -y docker-desktop

REM Vérifier si Docker est en cours d'exécution
docker info >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo Veuillez démarrer Docker Desktop et réessayer.
    echo Une fois Docker démarré, exécutez à nouveau ce script.
    exit /b 1
)

REM Installer les dépendances Python
echo Installation des dépendances Python...
pip3 install -U setuptools flask flask-socketio eventlet numpy opencv-python matplotlib pyyaml transforms3d

REM Configurer l'espace de travail
echo Configuration de l'espace de travail...
set WORKSPACE_DIR=%USERPROFILE%\ros2_ws
set SCRIPT_DIR=%~dp0

REM Créer les répertoires nécessaires
mkdir "%WORKSPACE_DIR%\src\picar_simulator\picar_description" 2>nul
mkdir "%WORKSPACE_DIR%\src\picar_simulator\picar_gazebo" 2>nul
mkdir "%WORKSPACE_DIR%\src\picar_simulator\picar_control" 2>nul
mkdir "%WORKSPACE_DIR%\src\picar_simulator\picar_bringup" 2>nul
mkdir "%WORKSPACE_DIR%\src\picar_simulator\web_interface\templates" 2>nul

REM Copier les fichiers du projet
echo Copie des fichiers du projet...

REM Copier les fichiers URDF et Gazebo
copy "%SCRIPT_DIR%picar_description_detailed.urdf" "%WORKSPACE_DIR%\src\picar_simulator\picar_description\"
copy "%SCRIPT_DIR%picar_gazebo_detailed.gazebo" "%WORKSPACE_DIR%\src\picar_simulator\picar_gazebo\"

REM Copier les nœuds ROS
copy "%SCRIPT_DIR%controller_node.py" "%WORKSPACE_DIR%\src\picar_simulator\picar_control\"
copy "%SCRIPT_DIR%sensor_sim_node.py" "%WORKSPACE_DIR%\src\picar_simulator\picar_control\"
copy "%SCRIPT_DIR%world_manager_node.py" "%WORKSPACE_DIR%\src\picar_simulator\picar_control\"

REM Copier les fichiers de lancement
copy "%SCRIPT_DIR%picar_simulator.launch.py" "%WORKSPACE_DIR%\src\picar_simulator\picar_bringup\"

REM Copier la configuration du bridge ROS-Unreal
copy "%SCRIPT_DIR%ros_unreal_bridge_config.yaml" "%WORKSPACE_DIR%\src\picar_simulator\"

REM Copier les templates web
xcopy /E /I /Y "%SCRIPT_DIR%web_interface\templates" "%WORKSPACE_DIR%\src\picar_simulator\web_interface\templates"

REM Créer un package.xml pour le package ROS
echo ^<?xml version="1.0"?^> > "%WORKSPACE_DIR%\src\picar_simulator\package.xml"
echo ^<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?^> >> "%WORKSPACE_DIR%\src\picar_simulator\package.xml"
echo ^<package format="3"^> >> "%WORKSPACE_DIR%\src\picar_simulator\package.xml"
echo   ^<name^>picar_simulator^</name^> >> "%WORKSPACE_DIR%\src\picar_simulator\package.xml"
echo   ^<version^>0.1.0^</version^> >> "%WORKSPACE_DIR%\src\picar_simulator\package.xml"
echo   ^<description^>Simulateur interactif pour le PiCar X^</description^> >> "%WORKSPACE_DIR%\src\picar_simulator\package.xml"
echo   ^<maintainer email="user@example.com"^>User^</maintainer^> >> "%WORKSPACE_DIR%\src\picar_simulator\package.xml"
echo   ^<license^>MIT^</license^> >> "%WORKSPACE_DIR%\src\picar_simulator\package.xml"
echo. >> "%WORKSPACE_DIR%\src\picar_simulator\package.xml"
echo   ^<buildtool_depend^>ament_cmake^</buildtool_depend^> >> "%WORKSPACE_DIR%\src\picar_simulator\package.xml"
echo   ^<buildtool_depend^>ament_cmake_python^</buildtool_depend^> >> "%WORKSPACE_DIR%\src\picar_simulator\package.xml"
echo. >> "%WORKSPACE_DIR%\src\picar_simulator\package.xml"
echo   ^<depend^>rclpy^</depend^> >> "%WORKSPACE_DIR%\src\picar_simulator\package.xml"
echo   ^<depend^>std_msgs^</depend^> >> "%WORKSPACE_DIR%\src\picar_simulator\package.xml"
echo   ^<depend^>sensor_msgs^</depend^> >> "%WORKSPACE_DIR%\src\picar_simulator\package.xml"
echo   ^<depend^>geometry_msgs^</depend^> >> "%WORKSPACE_DIR%\src\picar_simulator\package.xml"
echo   ^<depend^>nav_msgs^</depend^> >> "%WORKSPACE_DIR%\src\picar_simulator\package.xml"
echo   ^<depend^>tf2_ros^</depend^> >> "%WORKSPACE_DIR%\src\picar_simulator\package.xml"
echo   ^<depend^>gazebo_ros^</depend^> >> "%WORKSPACE_DIR%\src\picar_simulator\package.xml"
echo. >> "%WORKSPACE_DIR%\src\picar_simulator\package.xml"
echo   ^<export^> >> "%WORKSPACE_DIR%\src\picar_simulator\package.xml"
echo     ^<build_type^>ament_cmake^</build_type^> >> "%WORKSPACE_DIR%\src\picar_simulator\package.xml"
echo   ^</export^> >> "%WORKSPACE_DIR%\src\picar_simulator\package.xml"
echo ^</package^> >> "%WORKSPACE_DIR%\src\picar_simulator\package.xml"

REM Créer un script de lancement
echo @echo off > "%USERPROFILE%\launch_picar_simulator.bat"
echo docker run -it --rm ^^ >> "%USERPROFILE%\launch_picar_simulator.bat"
echo     -p 8080:8080 ^^ >> "%USERPROFILE%\launch_picar_simulator.bat"
echo     -v %USERPROFILE%\ros2_ws:/root/ros2_ws ^^ >> "%USERPROFILE%\launch_picar_simulator.bat"
echo     osrf/ros:humble-desktop ^^ >> "%USERPROFILE%\launch_picar_simulator.bat"
echo     bash -c "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 launch picar_simulator picar_simulator.launch.py" >> "%USERPROFILE%\launch_picar_simulator.bat"

REM Créer le fichier docker-compose.yml
echo version: '3' > "%SCRIPT_DIR%docker-compose.yml"
echo. >> "%SCRIPT_DIR%docker-compose.yml"
echo services: >> "%SCRIPT_DIR%docker-compose.yml"
echo   ros_gazebo: >> "%SCRIPT_DIR%docker-compose.yml"
echo     image: osrf/ros:humble-desktop >> "%SCRIPT_DIR%docker-compose.yml"
echo     container_name: picar_simulator >> "%SCRIPT_DIR%docker-compose.yml"
echo     ports: >> "%SCRIPT_DIR%docker-compose.yml"
echo       - "8080:8080" >> "%SCRIPT_DIR%docker-compose.yml"
echo     volumes: >> "%SCRIPT_DIR%docker-compose.yml"
echo       - ./:/root/picar_simulator >> "%SCRIPT_DIR%docker-compose.yml"
echo       - %USERPROFILE%\ros2_ws:/root/ros2_ws >> "%SCRIPT_DIR%docker-compose.yml"
echo     command: ^> >> "%SCRIPT_DIR%docker-compose.yml"
echo       bash -c "cd /root/picar_simulator && >> "%SCRIPT_DIR%docker-compose.yml"
echo                source /opt/ros/humble/setup.bash && >> "%SCRIPT_DIR%docker-compose.yml"
echo                source /root/ros2_ws/install/setup.bash && >> "%SCRIPT_DIR%docker-compose.yml"
echo                ros2 launch picar_simulator picar_simulator.launch.py" >> "%SCRIPT_DIR%docker-compose.yml"
echo. >> "%SCRIPT_DIR%docker-compose.yml"
echo   dev: >> "%SCRIPT_DIR%docker-compose.yml"
echo     image: osrf/ros:humble-desktop >> "%SCRIPT_DIR%docker-compose.yml"
echo     container_name: picar_simulator_dev >> "%SCRIPT_DIR%docker-compose.yml"
echo     ports: >> "%SCRIPT_DIR%docker-compose.yml"
echo       - "8080:8080" >> "%SCRIPT_DIR%docker-compose.yml"
echo     volumes: >> "%SCRIPT_DIR%docker-compose.yml"
echo       - ./:/root/picar_simulator >> "%SCRIPT_DIR%docker-compose.yml"
echo       - %USERPROFILE%\ros2_ws:/root/ros2_ws >> "%SCRIPT_DIR%docker-compose.yml"
echo     command: bash >> "%SCRIPT_DIR%docker-compose.yml"
echo     stdin_open: true >> "%SCRIPT_DIR%docker-compose.yml"
echo     tty: true >> "%SCRIPT_DIR%docker-compose.yml"

echo.
echo === Installation terminée ===
echo.
echo Pour utiliser le simulateur, double-cliquez sur:
echo   %USERPROFILE%\launch_picar_simulator.bat
echo.
echo Pour accéder à l'interface web, ouvrez un navigateur et allez à:
echo   http://localhost:8080
echo.
echo Pour utiliser Docker directement:
echo   docker-compose up ros_gazebo
echo.
echo Profitez de votre PiCar X Simulator!

pause