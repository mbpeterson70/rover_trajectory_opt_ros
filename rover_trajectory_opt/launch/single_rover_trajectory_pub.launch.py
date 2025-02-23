from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('rover_trajectory_opt'),
        # 'rover_trajectory_opt',
        'cfg',
        'single_rover_trajectory_pub.yaml'
        )

    trajectory_publisher_node = Node(
        name = 'trajectory_publisher_node',
        package = 'rover_trajectory_opt',
        executable = 'trajectory_publisher_node',
        output = 'screen',
        parameters = [config]
    )   

    ld = LaunchDescription()
    ld.add_action(trajectory_publisher_node)

    return ld
    