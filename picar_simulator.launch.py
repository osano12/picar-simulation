#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Fichier de lancement pour le simulateur PiCar X
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Chemins des fichiers
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    pkg_share = os.path.dirname(os.path.realpath(__file__))
    
    # Fichiers de description du robot
    urdf_file = os.path.join(pkg_share, 'picar_description_detailed.urdf')
    gazebo_file = os.path.join(pkg_share, 'picar_gazebo_detailed.gazebo')
    
    # Monde Gazebo
    world_file = os.path.join(pkg_share, 'workshop.world')
    
    # Configuration du bridge ROS-Unreal
    bridge_config_file = os.path.join(pkg_share, 'ros_unreal_bridge_config.yaml')
    
    # Paramètres de lancement
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_unreal = LaunchConfiguration('use_unreal')
    
    # Déclaration des arguments
    declare_mode_cmd = DeclareLaunchArgument(
        name='mode',
        default_value='standard',
        description='Mode de simulation: standard ou light')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Utiliser le temps de simulation')
    
    declare_use_unreal_cmd = DeclareLaunchArgument(
        name='use_unreal',
        default_value='true',
        description='Utiliser Unreal Engine pour le rendu visuel')
    
    # Charger le contenu des fichiers URDF et Gazebo
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()
    
    with open(gazebo_file, 'r') as f:
        gazebo_content = f.read()
    
    # Fusionner les fichiers URDF et Gazebo
    robot_description = robot_description_content.replace('</robot>', gazebo_content + '</robot>')
    
    # Lancer Gazebo
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'standard' and not ", use_unreal]))
    )
    
    # Spawn du robot dans Gazebo
    spawn_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'picar',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.02',  # Légèrement au-dessus du sol pour éviter les pénétrations
        ],
        output='screen'
    )
    
    # Publier la description du robot
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )
    
    # Nœud contrôleur
    controller_node_cmd = Node(
        package='picar_simulator',
        executable='controller_node.py',
        name='controller_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'control_rate': 50.0,
            'velocity_kp': 1.0,
            'velocity_ki': 0.1,
            'velocity_kd': 0.05,
            'steering_kp': 2.0,
            'steering_ki': 0.0,
            'steering_kd': 0.1,
            'auto_tune': False,
            'ultrasonic_threshold': 0.2,
            'line_detection_threshold': 0.05
        }]
    )
    
    # Nœud de simulation des capteurs
    sensor_sim_node_cmd = Node(
        package='picar_simulator',
        executable='sensor_sim_node.py',
        name='sensor_sim_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'ultrasonic_noise_stddev': 0.01,
            'ultrasonic_latency_min': 0.05,
            'ultrasonic_latency_max': 0.07,
            'ir_false_detection_prob': 0.05,
            'imu_gyro_drift': 0.00035,
            'imu_accel_noise': 0.02,
            'camera_noise': 0.1,
            'camera_latency': 0.1,
            'sensor_failure_prob': 0.001,
            'sensor_failure_duration_min': 1.0,
            'sensor_failure_duration_max': 5.0
        }]
    )
    
    # Nœud de gestion de l'environnement
    world_manager_node_cmd = Node(
        package='picar_simulator',
        executable='world_manager_node.py',
        name='world_manager_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'update_rate': 10.0,
            'weather_change_prob': 0.001,
            'day_night_cycle_duration': 600.0,
            'pedestrian_count': 5,
            'obstacle_count': 3
        }]
    )
    
    # Bridge ROS-Unreal
    ros_unreal_bridge_cmd = Node(
        package='ros_unreal_bridge',
        executable='bridge_node',
        name='ros_unreal_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'config_file': bridge_config_file
        }],
        condition=IfCondition(use_unreal)
    )
    
    # API Python
    picar_api_node_cmd = Node(
        package='picar_simulator',
        executable='picar_api.py',
        name='picar_api_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Interface web
    web_interface_cmd = Node(
        package='picar_simulator',
        executable='web_interface.py',
        name='web_interface_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'port': 8080,
            'host': '0.0.0.0'
        }]
    )
    
    # Message d'information
    info_message_cmd = ExecuteProcess(
        cmd=['echo', 'PiCar X Simulator démarré. Interface web disponible à http://localhost:8080'],
        output='screen'
    )
    
    # Créer la description de lancement
    ld = LaunchDescription()
    
    # Ajouter les déclarations d'arguments
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_unreal_cmd)
    
    # Ajouter les actions de lancement
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(controller_node_cmd)
    ld.add_action(sensor_sim_node_cmd)
    ld.add_action(world_manager_node_cmd)
    ld.add_action(ros_unreal_bridge_cmd)
    ld.add_action(picar_api_node_cmd)
    ld.add_action(web_interface_cmd)
    ld.add_action(info_message_cmd)
    
    return ld