import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'my_robot_description'
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Process the URDF file
    pkg_path = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_path,'urdf','robot.urdf.xacro')
    
    robot_description_config = Command(['xacro ', xacro_file])
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    # Create a joint_state_publisher node (bez GUI)
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'rate': 30}]  # uproszczone parametry
    )
    
    # Create a RViz node
    rviz_config_file = os.path.join(pkg_path, 'config', 'display.rviz')
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        node_joint_state_publisher,
        node_robot_state_publisher,
        node_rviz
    ])
