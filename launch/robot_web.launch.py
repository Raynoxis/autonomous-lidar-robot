#!/usr/bin/env python3
"""
MakersPet Mini - Web Navigation Launch File
Démarrage Nav2 + SLAM (RosBridge lancé par l'entrypoint)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Récupération des chemins de packages
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Arguments de lancement
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_mode = LaunchConfiguration('slam', default='True')

    # Chemin vers le fichier de paramètres navigation
    params_file = LaunchConfiguration('params_file',
        default='/app/config/navigation.yaml')

    # Inclusion de Nav2 Bringup avec SLAM
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': slam_mode,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='True',
            description='Enable SLAM mode'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='/app/config/navigation.yaml',
            description='Full path to the ROS2 parameters file for Nav2'
        ),
        nav2_bringup_launch,
    ])
