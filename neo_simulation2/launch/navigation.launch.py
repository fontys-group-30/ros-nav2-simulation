import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    param_dir = os.path.join(get_package_share_directory('neo_simulation2'), 'configs/' + "mpo_500", 'navigation.yaml')
    map_dir = os.path.join(get_package_share_directory('neo_simulation2'), 'maps', "neo_track1" + '.yaml')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('neo_nav2_bringup'), 'launch')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/localization_neo.launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': 'True',
                'use_multi_robots': 'False',
                'params_file': param_dir,
                'namespace': ''
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_neo.launch.py']),
            launch_arguments={
                'use_sim_time': 'True',
                'namespace': ''
            }.items(),
        )
    ])