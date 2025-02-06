from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node

import os

def generate_launch_description():
    # Declare launch arguments
    # veh_arg = DeclareLaunchArgument(
    #     'veh',
    #     default_value=EnvironmentVariable('VEHTYPE', default_value='RR'),
    #     description='Vehicle type (default: RR from VEHTYPE environment variable)'
    # )
    # num_arg = DeclareLaunchArgument(
    #     'num',
    #     default_value=EnvironmentVariable('VEHNUM', default_value='01'),
    #     description='Vehicle number (default: 01 from VEHNUM environment variable)'
    # )

    # Create namespace for the vehicle
    # namespace = [LaunchConfiguration('veh'), LaunchConfiguration('num')]
    namespace = f"{os.environ.get('VEHTYPE')}{os.environ.get('VEHNUM')}"
    namespace = 'RedRover'

    # Path to the parameter file
    # param_file_path = ParameterFile(
    #     path=[LaunchConfiguration('pkg_share'), '/cfg/mpc_params.yaml'],
    #     allow_substs=True
    # )

    # Define the node
    mpc_node = Node(
        package='rover_trajectory_opt',
        executable='mpc.py',
        namespace=namespace,
        remappings=[('/cmd_vel_auto', '/cmd_vel')],
        name='mpc_node',
        output='screen',
        # parameters=[param_file_path]
    )

    # Group node under the namespace
    group_action = GroupAction([mpc_node])

    # Launch description
    return LaunchDescription([
        group_action
    ])
