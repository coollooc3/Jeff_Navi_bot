"""Encoder odom + independent world pose + SLAM + raw recording; manual driving."""
from datetime import datetime, timezone
import os
from pathlib import Path
import socket
import uuid

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from leap_localization_lab.assets import prepare_assets


def generate_launch_description():
    if os.environ.get('ROS_DOMAIN_ID') != '178' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Lab requires ROS_DOMAIN_ID=178 and ROS_LOCALHOST_ONLY=1')
    with socket.socket() as probe:
        probe.settimeout(.2)
        if probe.connect_ex(('127.0.0.1', 11365)) == 0:
            raise RuntimeError('Close the previous Gazebo session before starting the lab')
    tb3 = Path(get_package_share_directory('turtlebot3_gazebo'))
    gazebo = Path(get_package_share_directory('gazebo_ros'))
    mapping = Path(get_package_share_directory('leap_mapping'))
    slam = Path(get_package_share_directory('slam_toolbox'))
    system_models = Path('/usr/share/gazebo-11/models')
    if not (system_models/'ground_plane/model.sdf').is_file():
        raise RuntimeError('Gazebo 11 system models are missing')
    if not Path('/opt/ros/humble/lib/libgazebo_ros_state.so').is_file():
        raise RuntimeError('Gazebo world-state plugin is missing')
    session = Path.home()/'leap_nav_data/localization'/(
        datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:6])
    prepare_assets(tb3/'models/turtlebot3_burger/model.sdf',
                   tb3/'worlds/turtlebot3_world.world', session)

    def include(path, arguments=None, **kwargs):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                        launch_arguments=(arguments or {}).items(), **kwargs)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        LogInfo(msg=f'LAB DATA: {session}'),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        SetEnvironmentVariable('GAZEBO_MASTER_URI', 'http://127.0.0.1:11365'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.pathsep.join([
            str(tb3/'models'), str(system_models), os.environ.get('GAZEBO_MODEL_PATH', '')])),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', os.pathsep.join([
            '/usr/share/gazebo-11', os.environ.get('GAZEBO_RESOURCE_PATH', '')])),
        include(gazebo/'launch/gzserver.launch.py',
                {'world': str(session/'lab.world'), 'server_required': 'true', 'seed': '42'}),
        include(gazebo/'launch/gzclient.launch.py', condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'use_sim_time': True,
                          'robot_description': (tb3/'urdf/turtlebot3_burger.urdf').read_text()}]),
        Node(package='gazebo_ros', executable='spawn_entity.py', arguments=[
            '-entity', 'burger', '-file', str(session/'burger_encoder.sdf'),
            '-x', '-2.0', '-y', '-0.5', '-z', '0.01', '-Y', '0.0'], output='screen'),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             parameters=[str(slam/'config/mapper_params_online_async.yaml'),
                         str(mapping/'config/mapping.yaml')], output='screen'),
        Node(package='leap_localization_lab', executable='world_pose',
             parameters=[{'use_sim_time': True}], output='screen'),
        ExecuteProcess(cmd=['ros2', 'bag', 'record', '--use-sim-time', '-s', 'sqlite3', '-o', str(session/'bag'),
                            '/odom', '/lab/ground_truth', '/scan', '/tf', '/tf_static', '/clock'],
                       output='screen'),
        Node(package='rviz2', executable='rviz2', name='localization_lab_view',
             arguments=['-d', str(mapping/'rviz/mapping.rviz')],
             parameters=[{'use_sim_time': True}], output='screen',
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
