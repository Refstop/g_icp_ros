import os

# from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false'),
    # rviz_config_dir = os.path.join(get_package_share_directory('icp_ros'), 'rviz', 'icp_ros.rviz')
    # example_dir = os.path.join(get_package_share_directory('icp_ros'), 'example', '2d_lidars_scan')
    rviz_config_dir = "/home/sparo/dev_ws/src/g_icp_ros/rviz/g_icp_ros.rviz"
    # example_dir = "/home/sparo/dev_ws/src/g_icp_ros/example/2d_lidars_scan"
    return LaunchDescription([
        # launch.actions.ExecuteProcess(
        #     cmd=['ros2', 'bag', 'play', example_dir],
        #     output='screen'
        # ),
        Node(
            package='g_icp_ros',
            node_executable='g_icp_ros_node',
            node_name = "g_icp_ros",
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