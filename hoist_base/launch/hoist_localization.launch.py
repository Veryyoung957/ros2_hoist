import launch
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
import launch_ros
import launch_ros.descriptions
import os
from launch import LaunchDescription
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    pkg_share = get_package_share_directory('hoist_base')
    default_model_path = os.path.join(pkg_share, 'urdf/hoist.urdf')
    navigation2_launch_dir = os.path.join(get_package_share_directory('hoist_navigation'), 'launch')
    nav2_map_dir = os.path.join(pkg_share, 'map', 'test2.yaml')
    slam_toolbox_localization_file_dir = os.path.join(pkg_share, 'config', 'mapper_params_localization.yaml')
    slam_toolbox_map_dir = PathJoinSubstitution([pkg_share, 'map', 'test2'])
    # nav2_params_file_dir = os.path.join(pkg_share, 'config', 'nav2_params_real.yaml')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    slam_toolbox_mapping_file_dir = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    # start_mapping = launch_ros.actions.Node(
    #     parameters=[
    #         slam_toolbox_mapping_file_dir,
    #         {'use_sim_time': True}
    #     ],
    #     package='slam_toolbox',
    #     executable='async_slam_toolbox_node',
    #     name='slam_toolbox',
    #     output='screen'
    # )
    # start_navigation2 = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'localization_amcl_launch.py')),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #         'map': nav2_map_dir,
    #         'params_file': params_file}.items()
    # )
    start_navigation2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'navigation_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'map': nav2_map_dir,
                              'params_file': params_file,
                              'use_lifecycle_mgr': 'false',
                              'map_subscribe_transient_local': 'true'}.items())
    start_localization = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir,
                                                       'localization_launch.py')),
            launch_arguments={'map': nav2_map_dir,
                              'use_sim_time': use_sim_time,
                              'params_file': params_file,
                              'use_lifecycle_mgr': 'false'}.items())
    # start_localization = launch_ros.actions.Node(
    #       parameters=[
    #         slam_toolbox_localization_file_dir ,
    #         {'use_sim_time': use_sim_time,
    #                 'map_file_name': slam_toolbox_map_dir,
    #                 'map_start_pose': [0.0, 0.0, 0.0]}
    #       ],
    #       package='slam_toolbox',
    #       executable='localization_slam_toolbox_node',
    #       name='slam_toolbox',
    #       output='screen'
    #     )
    # map_server = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'map_server_launch.py')),
    #         launch_arguments={
    #             'use_sim_time': use_sim_time,
    #             'map': nav2_map_dir,
    #             'params_file': params_file,
    #             'container_name': 'nav2_container'}.items())

    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    # ld.add_action(start_mapping)
    ld.add_action(start_navigation2)
    ld.add_action(start_localization)
    # ld.add_action(map_server)
    return ld