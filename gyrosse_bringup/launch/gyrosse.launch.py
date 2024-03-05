import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )
    
    gyrosse_bringup_path = os.path.join(
        get_package_share_directory('gyrosse_bringup'))

    gyrosse_description_path = os.path.join(
        get_package_share_directory('gyrosse_description'))

    robot_description_content = os.path.join(gyrosse_description_path,
                              'urdf',
                              'gyrosse_description.xacro')
    

    robot_description = {"robot_description": robot_description_content}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description, 
            os.path.join(gyrosse_bringup_path, 'config/gyrosse_controllers.yaml')],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gyrosse_base_controller", "-c", "/controller_manager"]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            os.path.join(gyrosse_description_path, 'config/config.rviz'),
        ],
        output='screen',
    )

    joy_node = Node(
        package='joy', executable='joy_node', name='joy_node',
        parameters=[{
            'deadzone': 0.05,
            'autorepeat_rate': 30.0
        }],
    )

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[os.path.join(gyrosse_bringup_path, 'config/ps4.config.yaml')],
        remappings=[
            ('/cmd_vel', '/gyrosse_base_controller/cmd_vel_unstamped'),
        ]
    )

    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        gazebo,
        rviz_node,
        joy_node,
        teleop_twist_joy_node,
    ])