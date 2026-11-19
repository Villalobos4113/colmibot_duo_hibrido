"""COLMIBOT con locomoción diferencial SOLO en Gazebo Harmonic + RViz 2 (C-007).

Igual que sim_stationary.launch.py, pero con urdf/colmibot_motion.urdf.xacro (colisión
collision_mode:=mobile y DiffDrive) y un segundo puente, colmibot_motion_bridge
(config/bridge_motion.yaml): /colmibot_sim/cmd_vel ROS->GZ, /odom y /tf (odom->base_footprint)
GZ->ROS. No inicia joystick, teleop, firmware ni nada que publique comandos: el movimiento solo
llega por /colmibot_sim/cmd_vel (por ejemplo, con `ros2 run colmibot_gazebo motion_probe`).

Uso:
  ros2 launch colmibot_gazebo sim_motion.launch.py
  ros2 launch colmibot_gazebo sim_motion.launch.py gui:=false rviz:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('colmibot_gazebo')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    use_camera = LaunchConfiguration('use_camera')
    use_meshes = LaunchConfiguration('use_meshes')

    # collision_mode:=mobile se pide explícitamente (también es el defecto del xacro de motion).
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg, 'urdf', 'colmibot_motion.urdf.xacro']),
                 ' collision_mode:=mobile use_camera:=', use_camera, ' use_meshes:=', use_meshes]),
        value_type=str,
    )
    # -s: solo servidor (sin ventana de Gazebo) salvo gui true/True/1, como IfCondition.
    gz_args = PythonExpression(
        ["'-r -v 3 ' + ('' if '", gui, "'.lower() in ('true', '1') else '-s ') + 'empty.sdf'"])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Ventana de Gazebo'),
        DeclareLaunchArgument('rviz', default_value='true', description='Abrir RViz'),
        DeclareLaunchArgument('use_camera', default_value='false',
                              description='Cámara simulada (requiere render)'),
        DeclareLaunchArgument('use_meshes', default_value='true',
                              description='false: geometría simplificada si faltan las mallas CAD'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'colmibot', '-z', '0.0'],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='colmibot_bridge',
            parameters=[{'config_file': PathJoinSubstitution(
                [pkg, 'config', 'bridge_stationary.yaml'])}],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='colmibot_motion_bridge',
            parameters=[{'config_file': PathJoinSubstitution([pkg, 'config', 'bridge_motion.yaml'])}],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='colmibot_camera_bridge',
            parameters=[{'config_file': PathJoinSubstitution([pkg, 'config', 'bridge_camera.yaml'])}],
            condition=IfCondition(use_camera),
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'sim_motion.rviz'])],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(rviz),
            output='screen',
        ),
    ])
