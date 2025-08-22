from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directories
    gluon_moveit_config_package_dir = get_package_share_directory('gluon_moveit_config')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Include demo from gluon_moveit_config (includes robot description, move_group, and rviz)
    gluon_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gluon_moveit_config_package_dir, 'launch', 'demo.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Actuator controller node
    actuator_controller_node = Node(
        package='actuatorcontroller_ros2',
        executable='innfos_actuator',
        name='innfos_actuator',
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        # Launch the demo from gluon_moveit_config
        gluon_demo_launch,
        # Launch our actuator controller
        actuator_controller_node
    ])