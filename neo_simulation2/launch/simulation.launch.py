import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    my_neo_robot = "mpo_500"
    my_neo_world = "neo_track1"

    world_path = os.path.join(get_package_share_directory('neo_simulation2'), 'worlds', f'{my_neo_world}.world')
    robot_description_urdf = os.path.join(get_package_share_directory('neo_simulation2'), 'robots', my_neo_robot, f'{my_neo_robot}.urdf')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path, 'verbose': 'true'}.items()
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', my_neo_robot, '-file', robot_description_urdf],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[robot_description_urdf]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('neo_simulation2'), 'launch', 'navigation.launch.py')
            ),
        ),
        IncludeLaunchDescription (
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('neo_nav2_bringup'), 'launch', 'rviz_launch.py')
            ),
        )
    ])