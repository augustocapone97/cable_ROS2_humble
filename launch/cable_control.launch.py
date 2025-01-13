from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os

def generate_launch_description():
    pkg_path = os.path.join(os.getenv('COLCON_PREFIX_PATH').split(":")[0],'cable_control', 'share', 'cable_control')

    urdf_file = os.path.join(pkg_path, 'urdf', 'flexible_cable.urdf')
    controller_config = os.path.join(pkg_path, 'config', 'cable_controllers.yaml')

    if not os.path.isfile(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")
    if not os.path.isfile(controller_config):
        raise FileNotFoundError(f"Controller config file not found: {controller_config}")

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            parameters=[controller_config],
            output='screen',
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster', "--controller-manager", "/controller_manager"],
                    # parameters=['-joints', 'joint_end_sphere'],
                    output='screen',
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['position_controller', "--controller-manager", "/controller_manager"],
                    # parameters=['-joints', 'joint_end_sphere'],
                    output='screen',
                ),
            ],
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-entity', 'flexible_cable',
                '-file', urdf_file,
            ],
            output='screen',
        ),
    ])
