from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

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

    config = os.path.join(
        get_package_share_directory('rover_trajectory_opt'),
        # 'rover_trajectory_opt',
        'cfg',
        'mpc_params.yaml'
        )

    # Path to the parameter file
    # param_file_path = ParameterFile(
    #     path=[LaunchConfiguration('pkg_share'), '/cfg/mpc_params.yaml'],
    #     allow_substs=True
    # )

    # Define the node
    mpc_node = Node(
        package='rover_trajectory_opt',
        executable='mpc_node',
        namespace=namespace,
        remappings=[(f'/{namespace}/cmd_vel_auto', f'/{namespace}/cmd_vel')],
        name='mpc_node',
        output='screen',
        parameters=[config]
    )

    # Group node under the namespace
    group_action = GroupAction([mpc_node])

    # Launch description
    return LaunchDescription([
        group_action
    ])
