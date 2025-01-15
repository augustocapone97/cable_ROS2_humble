from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Ottieni il percorso del file Xacro
    package_share_directory = get_package_share_directory('cable_conv')
    xacro_file = os.path.join(package_share_directory, 'urdf', 'cable_model.urdf')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'cable_model',
                '-file', xacro_file
            ],
            output='screen'
        ),

        Node(
            package='cable_conv',
            executable='CableController.py',
            name='cable_controller',
            output='screen'
        ),
    ])
