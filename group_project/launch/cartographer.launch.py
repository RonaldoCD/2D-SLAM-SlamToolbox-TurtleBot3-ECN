import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespace = LaunchConfiguration('namespace', default='waffle2')
    # map_frame = LaunchConfiguration('map_frame', default=[namespace, '/map'])
    odom_frame = LaunchConfiguration('odom_frame', default=[namespace, '/odom'])
    base_link = LaunchConfiguration('base_link', default=[namespace, '/base_footprint'])
    scan_topic = LaunchConfiguration('scan_topic', default=[namespace, '/scan'])

    cartographer_config_dir = os.path.join(get_package_share_directory('group_project'), 'config')
    cartographer_config_file = "cartographer.lua"

    rviz_file = "gp_rviz_cart.rviz"
    rviz_file_path = os.path.join(get_package_share_directory('group_project'), 'rviz', rviz_file)
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'namespace',
            default_value='waffle2',
            description='Namespace'),

        Node(package='cartographer_ros', 
            executable='cartographer_node', 
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', cartographer_config_file],
            remappings=[
                # ('/map', map_frame),
                ('/odom', odom_frame),
                ('/base_footprint', base_link),
                ('/scan', scan_topic),                
            ]),

        Node(package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', '0.01', '-publish_period_sec', '1.0'],
            ),

        Node(package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + rviz_file_path])
    ])

