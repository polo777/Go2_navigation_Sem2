import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():
    launch_rviz = LaunchConfiguration('rviz', default=False)
    launch_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value=launch_rviz,
        description='Use RViz to monitor results')
    
    rviz_config_file = LaunchConfiguration('rviz_config_file', default='/root/ros2_ws/src/go2_bringup/config/go2_super_odom_2.rviz')
    rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file', default_value=rviz_config_file,
        description='Config file for rviz visualisation')
    
    # super_odom_config_path = get_share_file(
    #     package_name="super_odometry",
    #     file_name="config/livox_mid360_nav2.yaml")
    super_odom_config_path = "/root/ros2_ws/src/go2_bringup/config/livox_mid360_nav2.yaml"
    
    nav2_params_file = LaunchConfiguration('params_file', default='/root/ros2_ws/src/go2_bringup/config/nav2_params_go2.yaml')
    nav2_params_file_cmd = DeclareLaunchArgument(
        'params_file', default_value=nav2_params_file,
        description='Full path to Nav2 params YAML file')
    
    slam_params_file = LaunchConfiguration('slam_params_file', default='/root/ros2_ws/src/go2_bringup/config/slam_toolbox_params.yaml')
    slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file', default_value=slam_params_file,
        description='Full path to SLAM Toolbox params YAML file')

    super_odometry_dir = get_package_share_directory('super_odometry')
    super_odometry_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(super_odometry_dir, 'launch', 'livox_mid360.launch.py')
        ),
        launch_arguments={
            'tf_remap_dest': TextSubstitution(text='/tf_dummy'),
            'config_file': super_odom_config_path,
        }.items()
    )

    pointcloud_to_laserscan_dir = get_package_share_directory('pointcloud_to_laserscan')
    pointcloud_to_laserscan_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pointcloud_to_laserscan_dir, 'launch', 'go2_pointcloud_to_laserscan.launch.py')
        )
    )

    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': TextSubstitution(text='false')
        }.items()
    )

    # slam_toolbox_localise_node = Node(
    #     package='slam_toolbox',
    #     executable='async_slam_toolbox_node',
    #     name='slam_toolbox',
    #     output='screen',
    #     parameters=[{
    #         # ---------- Core mode ----------
    #         'use_sim_time': False,
    #         'slam_mode': 'localization',          # <-- THIS IS THE KEY
    #         'map_file_name': '',            # used only when use_posegraph=true
    #         'map_start_pose': [0.0, 0.0, 0.0],     # optional initial guess

    #         # ---------- Pose-graph loading ----------
    #         'load_map_from_file': False,         # true → posegraph, false → pgm+yaml
    #         'pgm_file_name': '/root/ros2_ws/src/go2_bringup/config/robotics_lab_v1.pgm',
    #         'yaml_file_name': '/root/ros2_ws/src/go2_bringup/config/robotics_lab_v1.yaml',

    #         # ---------- Common localisation tweaks ----------
    #         'odom_frame': 'odom',
    #         'map_frame':  'map',
    #         'base_frame': 'base_footprint',
    #         'scan_topic': '/scan',
    #         'minimum_travel_distance': 0.1,
    #         'minimum_travel_heading': 0.1,

    #         # ---------- Optional: tighter filter ----------
    #         'particle_size': 200,
    #         'resample_interval': 1,
    #         'transform_publish_period': 0.02,
    #     }],
    # )

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params_file,
            'map': '/root/ros2_ws/src/go2_bringup/config/robotics_lab_v1.yaml',  # Empty: Disables map_server (use SLAM's dynamic /map)
        }.items()
    )

    # footprint_transform_publisher = Node(
    #     package='go2_nav2',
    #     executable='footprint_transform_publisher'
    # )

    # transform_remapper = Node(
    #     package='go2_nav2',
    #     executable='transform_remapper'
    # )

    # tf_publishing = Node(
    #     package='go2_nav2',
    #     executable='odom_tf_broadcaster'
    # )

    tf_tree_publisher_node = Node(
        package='go2_nav2',
        executable='tf_tree_publisher_node'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(launch_rviz)
    )

    ld = LaunchDescription()

    # Nodes and LaunchFiles
    ld.add_action(super_odometry_cmd)
    ld.add_action(rviz_node)
    # ld.add_action(tf_publishing)
    # ld.add_action(footprint_transform_publisher)
    # ld.add_action(transform_remapper)
    ld.add_action(tf_tree_publisher_node)
    ld.add_action(pointcloud_to_laserscan_cmd)
    ld.add_action(slam_toolbox_cmd)
    # ld.add_action(slam_toolbox_localise_node)
    ld.add_action(nav2_cmd)

    # Launch Arguments
    ld.add_action(launch_rviz_cmd)
    ld.add_action(rviz_config_file_cmd)
    ld.add_action(nav2_params_file_cmd)
    ld.add_action(slam_params_file_cmd)

    return ld