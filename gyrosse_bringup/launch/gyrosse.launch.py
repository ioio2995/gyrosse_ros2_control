from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("gyrosse_description"),
                    "urdf",
                    "gyrosse_description.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    controllers_config_file = PathJoinSubstitution(
        [
            FindPackageShare("gyrosse_bringup"),
            "config",
            "gyrosse_controller.yaml",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("gyrosse_description"), 
            "config", 
            "config.rviz"
        ]
    )

    joy_config_file = PathJoinSubstitution(
        [
            FindPackageShare("gyrosse_bringup"), 
            "config", 
            "ps4.config.yaml"
        ]
    )


    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
        ),
        launch_arguments={"verbose": "false"}.items(),
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description, 
            controllers_config_file],
        output="both"
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gyrosse_base_controller",  "--controller-manager", "/controller_manager"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            rviz_config_file,
        ],
        output="screen",
    )

    joy_node = Node(
        package="joy", executable="joy_node", name="joy_node",
        parameters=[{
            "deadzone": 0.05,
            "autorepeat_rate": 30.0
        }],
    )

    teleop_twist_joy_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[joy_config_file],
        remappings=[
            ("/cmd_vel", 
             "/gyrosse_base_controller/cmd_vel_unstamped"),
        ]
    )

    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        gazebo_node,
        joy_node,
        teleop_twist_joy_node,
    ])