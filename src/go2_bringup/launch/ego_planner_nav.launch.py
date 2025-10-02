import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    launch_rviz = LaunchConfiguration('rviz', default=False)
    launch_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value=launch_rviz,
        description='Use RViz to monitor results')
    
    rviz_config_file = LaunchConfiguration('rviz_config_file', default='/home/unitree/ros2_ws/src/go2_bringup/config/go2_test2.rviz')
    rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file', default_value=rviz_config_file,
        description='Config file for rviz visualisation')

    # super_odometry_dir = get_package_share_directory('super_odometry')
    # super_odometry_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(super_odometry_dir, 'launch', 'livox_mid360.launch.py')
    #     ),
    #     # launch_arguments={
    #     #     '<argument>': '<value>'
    #     # }.items()
    # )

    ego_planner_dir = get_package_share_directory('ego_planner')
    ego_planner_cmd = TimerAction(
            period=10.0,
            actions = [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ego_planner_dir, 'launch', 'go2_run.launch.py')
            ),
            # launch_arguments={
            #     '<argument>': '<value>'
            # }.items()
            )]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(launch_rviz)
    )

    ld = LaunchDescription()

    # Nodes and LaunchFiles
    # ld.add_action(super_odometry_cmd)
    ld.add_action(rviz_node)
    ld.add_action(ego_planner_cmd)

    # Launch Arguments
    ld.add_action(launch_rviz_cmd)
    ld.add_action(rviz_config_file_cmd)

    return ld