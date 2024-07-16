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
    # world = LaunchConfiguration('world')
    # nav2_map_dir = PathJoinSubstitution([pkg_share, 'map', world]), ".yaml"
    nav2_params_file_dir = os.path.join(pkg_share, 'config', 'nav2_params_real.yaml')
    # slam_toolbox_map_dir = PathJoinSubstitution([pkg_share, 'map', world])
    # slam_toolbox_localization_file_dir = os.path.join(pkg_share, 'config', 'mapper_params_localization.yaml')
    # default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    model = launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                        description='Absolute path to robot urdf file')
    # launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
        #                                description='Absolute path to rviz config file'),
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='328',
        description='Select world (map file, pcd file, world file share the same name prefix as the this parameter)')
    use_sim_time = launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                        description='Flag to enable use_sim_time')
    
    slam_toolbox_mapping_file_dir = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    rplidar = get_package_share_directory('rplidar_ros')
    hoist_base = launch_ros.actions.Node(
        package='hoist_base',
        executable='hoist_base',
    )
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': launch_ros.descriptions.ParameterValue( launch.substitutions.Command(['xacro ',os.path.join(pkg_share,'urdf/hoist.urdf')]), value_type=str)  }]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path], #Add this line
    )
    
    # rviz_node = launch_ros.actions.Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', LaunchConfiguration('rvizconfig')],
    # )
    robot_localization_node = launch_ros.actions.Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    start_mapping = launch_ros.actions.Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_toolbox_mapping_file_dir,
            {'use_sim_time': LaunchConfiguration('use_sim_time'),}
        ]
    )
    lidar_start = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(rplidar,'launch','rplidar_a2m8_launch.py')),
                launch_arguments= {
                    'serial_port': '/dev/ttyUSB1'
                }.items()
                
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
    ld.add_action(hoist_base)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(lidar_start)
    ld.add_action(start_mapping)
    ld.add_action(robot_localization_node)
    ld.add_action(start_navigation2)
    ld.add_action(declare_world_cmd)
    return ld