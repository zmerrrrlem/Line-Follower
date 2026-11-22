#!/usr/bin/env python3
"""
Spawn the Line_Follower robot into a Gazebo Sim world (ROS 2 Jazzy).

Expected layout (the launch file lives in Line_Follower/launch/):

    Line_Follower/
    ├── Untitled.urdf
    ├── meshes/            (base_*.stl, left_wheel_1.stl, right_wheel_1.stl)
    ├── worlds/
    │   ├── line_track.sdf
    │   └── track.png
    ├── config/
    │   ├── twist_mux.yaml
    │   └── line_pid.yaml
    ├── scripts/
    │   ├── line_sensor_node.py
    │   ├── line_pid.py
    │   └── turn_node.py
    └── launch/
        └── spawn_line_follower.launch.py

How it works:
  * robot_state_publisher publishes /robot_description and TF from the URDF.
  * The relative mesh paths in the URDF are rewritten to absolute file:// URIs,
    so Gazebo finds the STL files no matter where it is started from.
  * The same URDF string is sent to Gazebo through the ros_gz_sim `create`
    node (Gazebo converts URDF -> SDF itself), so there is no intermediate
    .sdf file that can go stale or race with the spawn.
  * /clock, /cmd_vel, /odom, /joint_states, /tf and the line camera image are
    bridged between Gazebo and ROS.
  * line_sensor_node.py turns the camera image into /line_sensors and /line_position.
  * Velocity sources never publish to /cmd_vel directly. twist_mux chooses one:
        /cmd_vel_turn   (turn_node.py,   priority 100, only while turning)
        /cmd_vel_manual (teleop,         priority  50)
        /cmd_vel_line   (line_pid.py,    priority  10)
    and publishes the winner on /cmd_vel (see config/twist_mux.yaml).

Usage:
    source /opt/ros/jazzy/setup.bash
    ros2 launch /full/path/to/spawn_line_follower.launch.py
    ros2 launch /full/path/to/spawn_line_follower.launch.py line_pid:=false turn_node:=false

Requires:  sudo apt install ros-jazzy-twist-mux

Arguments:
    world       : World file for Gazebo (default: worlds/line_track.sdf). Can be a file
                  name Gazebo knows about or an absolute path to your own world.
                  NOTE: the world needs the Sensors system for the line camera.
    world_name  : Name of the <world> inside that file (default: line_track).
                  Must match the name in the SDF, otherwise spawning fails.
    robot_name  : Name of the spawned entity (default: line_follower)
    x, y, z     : Spawn position (default: 0 -0.6 0.02, on the bottom straight of the track)
    yaw         : Spawn yaw in radians (default: 0, sensor bar facing +x)
    line_sensors    : start scripts/line_sensor_node.py (default: true)
    line_pid        : start scripts/line_pid.py (default: true; it waits for /line_follow_enable)
    turn_node       : start scripts/turn_node.py (default: true; it is silent until /turn_deg)
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
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
    scripts_dir = os.path.join(robot_dir, 'scripts')
    config_dir = os.path.join(robot_dir, 'config')
    sensor_file = os.path.join(scripts_dir, 'line_sensor_node.py')
    pid_file = os.path.join(scripts_dir, 'line_pid.py')
    turn_file = os.path.join(scripts_dir, 'turn_node.py')
    mux_config = os.path.join(config_dir, 'twist_mux.yaml')
    pid_config = os.path.join(config_dir, 'line_pid.yaml')
    worlds_dir = os.path.join(robot_dir, 'worlds')
    default_world = os.path.join(worlds_dir, 'line_track.sdf')

    # Let Gazebo find the track texture (and anything else next to the world file).
    resource_path = os.pathsep.join(
        [p for p in [worlds_dir, robot_dir, os.environ.get('GZ_SIM_RESOURCE_PATH', '')] if p]
    )
    set_gz_resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path)

    if not os.path.isfile(urdf_file):
        raise FileNotFoundError(f'URDF not found: {urdf_file}')
    if not os.path.isdir(os.path.join(robot_dir, 'meshes')):
        raise FileNotFoundError(f'meshes/ folder not found in: {robot_dir}')
    for required in (sensor_file, pid_file, turn_file, mux_config, pid_config):
        if not os.path.isfile(required):
            raise FileNotFoundError(f'Missing file: {required}')

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
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Gazebo world file (name or absolute path)'),
        DeclareLaunchArgument('world_name', default_value='line_track',
                              description='Name of the <world> element inside the world file'),
        DeclareLaunchArgument('robot_name', default_value='line_follower',
                              description='Name of the spawned robot entity'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn X position'),
        DeclareLaunchArgument('y', default_value='-0.6', description='Spawn Y position'),
        DeclareLaunchArgument('z', default_value='0.02', description='Spawn Z position'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw (rad)'),
        DeclareLaunchArgument('line_sensors', default_value='true',
                              description='Start line_sensor_node.py (/line_sensors, /line_position)'),
        DeclareLaunchArgument('line_pid', default_value='true',
                              description='Start line_pid.py (waits for /line_follow_enable)'),
        DeclareLaunchArgument('turn_node', default_value='true',
                              description='Start turn_node.py (silent until /turn_deg)'),
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
            '/line_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
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
    # twist_mux: /cmd_vel_turn, /cmd_vel_manual, /cmd_vel_line -> /cmd_vel
    # ------------------------------------------------------------------
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[mux_config],
        remappings=[('cmd_vel_out', '/cmd_vel')],
    )

    # ------------------------------------------------------------------
    # Behaviour nodes (they publish to their own topic, never to /cmd_vel)
    # ------------------------------------------------------------------
    turn_node = ExecuteProcess(
        cmd=['python3', turn_file, '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('turn_node')),
    )
    line_pid = ExecuteProcess(
        cmd=['python3', pid_file, '--ros-args', '--params-file', pid_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('line_pid')),
    )
    delayed_behaviours = TimerAction(period=8.0, actions=[turn_node, line_pid])

    # ------------------------------------------------------------------
    # Line sensor array: camera image -> /line_sensors + /line_position
    # Disable with:  line_sensors:=false
    # ------------------------------------------------------------------
    line_sensor_node = ExecuteProcess(
        cmd=['python3', sensor_file, '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('line_sensors')),
    )
    delayed_sensors = TimerAction(period=6.0, actions=[line_sensor_node])

    return LaunchDescription(
        [set_gz_resource_path]
        + args
        + [gazebo, robot_state_publisher, clock_bridge, twist_mux, delayed_spawn,
           delayed_sensors, delayed_behaviours]
    )
