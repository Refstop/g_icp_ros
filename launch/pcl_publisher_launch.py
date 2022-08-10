import os

# from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false'),
    rviz_config_dir = "/home/sparo/dev_ws/src/g_icp_ros/rviz/pcl_publisher.rviz"
    return LaunchDescription([
        Node(
            package='g_icp_ros',
            node_executable='pcl_publisher_node',
            node_name = "pcl_publisher",
            output='screen'
        ),
        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])