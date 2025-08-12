import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get package directories
    moveit_config_package = "gluon_moveit_config"
    robot_description_package = "gluon_py"
    
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="gluon.urdf",
            description="URDF description file with the robot.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        )
    )
    
    # Initialize arguments
    description_file = LaunchConfiguration("description_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # Robot description
    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare(robot_description_package), "urdf", description_file]
                ),
            ]
        ),
        value_type=str
    )
    
    robot_description = {"robot_description": robot_description_content, "use_sim_time": use_sim_time}
    
    # Robot description semantic (SRDF)
    robot_description_semantic_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="cat")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare(moveit_config_package), "config", "gluon.srdf"]
                ),
            ]
        ),
        value_type=str
    )
    
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    # Joint state publisher GUI
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[robot_description],
    )
    
    # Move group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"use_sim_time": use_sim_time},
            # Load additional MoveIt parameters
            os.path.join(
                FindPackageShare(moveit_config_package).find(moveit_config_package),
                "config",
                "kinematics.yaml",
            ),
            os.path.join(
                FindPackageShare(moveit_config_package).find(moveit_config_package),
                "config",
                "joint_limits.yaml",
            ),
        ],
    )
    
    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "moveit.rviz"]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            {"use_sim_time": use_sim_time},
        ],
    )
    
    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        move_group_node,
        rviz_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes_to_start)