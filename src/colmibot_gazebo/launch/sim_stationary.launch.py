"""COLMIBOT estacionario en Gazebo Harmonic + RViz 2 (C-006).

Patrón de arranque observado en Mingyar2/colmibot_ws@0aed972 (gazebo_colmi.launch.py), reescrito
sin joystick, teleop, cmd_vel, odometría ni TF desde Gazebo.

Uso:
  ros2 launch colmibot_gazebo sim_stationary.launch.py
  ros2 launch colmibot_gazebo sim_stationary.launch.py gui:=false rviz:=false
  ros2 launch colmibot_gazebo sim_stationary.launch.py use_camera:=true
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

    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg, 'urdf', 'colmibot_sim.urdf.xacro']),
                 ' use_camera:=', use_camera, ' use_meshes:=', use_meshes]),
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
            name='colmibot_camera_bridge',
            parameters=[{'config_file': PathJoinSubstitution([pkg, 'config', 'bridge_camera.yaml'])}],
            condition=IfCondition(use_camera),
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'sim.rviz'])],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(rviz),
            output='screen',
        ),
    ])
