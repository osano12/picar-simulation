#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Fichier de lancement pour la simulation PiCar X
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Chemins des fichiers
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    pkg_share = os.path.dirname(os.path.realpath(__file__))
    
    # Fichiers de description du robot
    urdf_file = os.path.join(pkg_share, 'picar_description.urdf')
    gazebo_file = os.path.join(pkg_share, 'picar_gazebo.gazebo')
    
    # Monde Gazebo
    world_file = os.path.join(pkg_share, 'workshop.world')
    
    # Paramètres de lancement
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Déclaration des arguments
    declare_mode_cmd = DeclareLaunchArgument(
        name='mode',
        default_value='standard',
        description='Mode de simulation: standard ou light')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Utiliser le temps de simulation')
    
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
        condition=IfCondition(PythonExpression(["'", mode, "' == 'standard'"]))
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
    
    # Nœud de contrôle du robot
    robot_controller_cmd = Node(
        package='picar_control',
        executable='picar_controller_node',
        name='picar_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'standard'"]))
    )
    
    # Interface web (simulée par un message)
    web_interface_cmd = ExecuteProcess(
        cmd=['echo', 'Interface web disponible à http://localhost:8080'],
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'standard'"]))
    )
    
    # Créer la description de lancement
    ld = LaunchDescription()
    
    # Ajouter les déclarations d'arguments
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    
    # Ajouter les actions de lancement
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(robot_controller_cmd)
    ld.add_action(web_interface_cmd)
    
    return ld