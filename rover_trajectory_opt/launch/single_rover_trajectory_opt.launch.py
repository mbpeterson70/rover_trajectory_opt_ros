from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # trajectory_path = PathJoinSubstitution([
    #     FindPackageShare('rover_trajectory_opt_ros'), 
    #     'rover_trajectory_opt', 
    #     'cfg', 
    #     'single_rover_trajectory_opt.yaml'
    # ])

    # trajectory_path = PathJoinSubstitution([
    #     FindPackageShare('rover_trajectory_opt'), 
    #     # 'rover_trajectory_opt', 
    #     'cfg', 
    #     'single_redrover_trajectory_opt.yaml'
    # ])

    config = os.path.join(
        get_package_share_directory('rover_trajectory_opt'),
        # 'rover_trajectory_opt',
        'cfg',
        'single_redrover_trajectory_opt.yaml'
        )

    trajectory_generator_node = Node(
        name = 'trajectory_generator_node',
        package = 'rover_trajectory_opt',
        executable = 'trajectory_generator_node',
        output = 'screen',
        parameters = [config]
    )   

    ld = LaunchDescription()
    ld.add_action(trajectory_generator_node)

    return ld
    