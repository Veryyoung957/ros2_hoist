import launch
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
import launch_ros
import launch_ros.descriptions
import os
from launch import LaunchDescription
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    pkg_share = get_package_share_directory('hoist_base')
    default_model_path = os.path.join(pkg_share, 'urdf/hoist.urdf')
    navigation2_launch_dir = os.path.join(get_package_share_directory('hoist_navigation'), 'launch')
    nav2_params_file_dir = os.path.join(pkg_share, 'config', 'nav2_params_real.yaml')
    model = launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                        description='Absolute path to robot urdf file')
    use_sim_time = launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                        description='Flag to enable use_sim_time')
    
    slam_toolbox_mapping_file_dir = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    start_mapping = launch_ros.actions.Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_toolbox_mapping_file_dir,
            {'use_sim_time': LaunchConfiguration('use_sim_time'),}
        ]
    )
    start_navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'bringup_hoist_navigation.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # 'map': nav2_map_dir,
            'params_file': nav2_params_file_dir}.items()
    )
    ld = LaunchDescription()
    ld.add_action(model)
    ld.add_action(use_sim_time)
    ld.add_action(start_mapping)
    ld.add_action(start_navigation2)
    return ld