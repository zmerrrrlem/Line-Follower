#!/usr/bin/env python3
"""
Spawn the Line_Follower robot into a Gazebo Sim world (ROS 2 Jazzy).

Expected layout (the launch file lives in Line_Follower/launch/):

    Line_Follower/
    ├── Untitled.urdf
    ├── meshes/
    │   ├── base_link.stl
    │   ├── left_wheel_1.stl
    │   └── right_wheel_1.stl
    └── launch/
        └── spawn_line_follower.launch.py

How it works:
  * robot_state_publisher publishes /robot_description and TF from the URDF.
  * The relative mesh paths in the URDF are rewritten to absolute file:// URIs,
    so Gazebo finds the STL files no matter where it is started from.
  * The same URDF string is sent to Gazebo through the ros_gz_sim `create`
    node (Gazebo converts URDF -> SDF itself), so there is no intermediate
    .sdf file that can go stale or race with the spawn.
  * /clock is bridged to ROS so use_sim_time works.

Usage:
    source /opt/ros/jazzy/setup.bash
    ros2 launch spawn_line_follower.launch.py
    ros2 launch spawn_line_follower.launch.py world:=empty.sdf x:=1.0 y:=0.5

Arguments:
    world       : World file for Gazebo (default: empty.sdf). Can be a file
                  name Gazebo knows about or an absolute path to your own world.
    world_name  : Name of the <world> inside that file (default: empty).
                  Must match the name in the SDF, otherwise spawning fails.
    robot_name  : Name of the spawned entity (default: line_follower)
    x, y, z     : Spawn position (default: 0 0 0.1)
    yaw         : Spawn yaw in radians (default: 0)
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # ------------------------------------------------------------------
    # Paths (relative to this file, so the folder can be moved freely)
    # ------------------------------------------------------------------
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    robot_dir = os.path.dirname(launch_dir)  # Line_Follower/
    urdf_file = os.path.join(robot_dir, 'Untitled.urdf')
    controller_file = os.path.join(robot_dir, 'turn_controller.py')

    if not os.path.isfile(urdf_file):
        raise FileNotFoundError(f'URDF not found: {urdf_file}')
    if not os.path.isdir(os.path.join(robot_dir, 'meshes')):
        raise FileNotFoundError(f'meshes/ folder not found in: {robot_dir}')

    # Read the URDF and make the mesh paths absolute.
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    robot_description = robot_description.replace(
        'filename="meshes/', f'filename="file://{robot_dir}/meshes/'
    )

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_sim_launch = os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    args = [
        DeclareLaunchArgument('world', default_value='empty.sdf',
                              description='Gazebo world file (name or absolute path)'),
        DeclareLaunchArgument('world_name', default_value='empty',
                              description='Name of the <world> element inside the world file'),
        DeclareLaunchArgument('robot_name', default_value='line_follower',
                              description='Name of the spawned robot entity'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn X position'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn Y position'),
        DeclareLaunchArgument('z', default_value='0.02', description='Spawn Z position'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw (rad)'),
        DeclareLaunchArgument('turn_controller', default_value='true',
                              description='Start turn_controller.py (it drives the robot via /cmd_vel)'),
        DeclareLaunchArgument('forward_speed', default_value='0.0',
                              description='Forward speed used by the turn controller (m/s)'),
    ]

    # ------------------------------------------------------------------
    # Gazebo Sim (-r = start running, -v 3 = info-level logging)
    # ------------------------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), TextSubstitution(text=' -r -v 3')],
        }.items(),
    )

    # ------------------------------------------------------------------
    # ROS side: robot_description + TF, and the simulation clock
    # ------------------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ],
    )

    # Gazebo <-> ROS bridge: clock, drive command, odometry, joint states, TF
    #   '['  = Gazebo -> ROS,   ']' = ROS -> Gazebo
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
    )

    # ------------------------------------------------------------------
    # Spawn the robot (URDF string -> Gazebo converts it to SDF)
    # ------------------------------------------------------------------
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_line_follower',
        output='screen',
        arguments=[
            '-world', LaunchConfiguration('world_name'),
            '-string', robot_description,
            '-name', LaunchConfiguration('robot_name'),
            '-allow_renaming', 'true',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    # Give Gazebo a few seconds to bring up the world before spawning.
    delayed_spawn = TimerAction(period=4.0, actions=[spawn_robot])

    # ------------------------------------------------------------------
    # Turn controller: drives forward and turns on demand (/turn_deg).
    # Started after the robot has spawned and /odom is being published.
    # Disable with:  turn_controller:=false
    # ------------------------------------------------------------------
    turn_controller = ExecuteProcess(
        cmd=[
            'python3', controller_file,
            '--ros-args',
            '-p', 'use_sim_time:=true',
            '-p', ['forward_speed:=', LaunchConfiguration('forward_speed')],
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('turn_controller')),
    )
    delayed_controller = TimerAction(period=8.0, actions=[turn_controller])

    return LaunchDescription(
        args + [gazebo, robot_state_publisher, clock_bridge, delayed_spawn, delayed_controller]
    )