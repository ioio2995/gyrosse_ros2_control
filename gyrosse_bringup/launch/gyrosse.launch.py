from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, 
    FindExecutable, 
    LaunchConfiguration,
    PathJoinSubstitution, 
    )
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "ifname_can",
            default_value="can0",
            description="Can address of RMD Motor.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'use_fake_hardware' parameter is true.",
        )
    ) 

    # Initialize Arguments
    ifname_can = LaunchConfiguration("ifname_can")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")

     # Get URDF via xacro
    robot_urdf_xacro = PathJoinSubstitution(
        [
            FindPackageShare("gyrosse_description"),
            "urdf",
            "gyrosse_description.xacro",
        ]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            robot_urdf_xacro,
            " ",
            "ifname_can:=",
            ifname_can,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
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

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        gazebo_node,
        joy_node,
        teleop_twist_joy_node,
    ]

    return LaunchDescription(declared_arguments + nodes)