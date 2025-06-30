import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # Parámetros configurables
    description_file = LaunchConfiguration("description_file", default="robot.urdf.xacro")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # Rutas al paquete
    rover_description_pkg = FindPackageShare("rover_description")


    # Procesar el URDF desde el XACRO

    xacro_cmd = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ", PathJoinSubstitution([rover_description_pkg, "robots", description_file])
    ])
    robot_description = ParameterValue(xacro_cmd, value_type=str)

    # Nodo robot_state_publisher con todos los toques de estilo
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name='robot_state_publisher',
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": robot_description,
        }]
    )    
    
    #joint state publisher
    joint_state_pub_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen"
    )
    
    # RViz opcional (comenta si no lo necesitas)
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["-d", PathJoinSubstitution([rover_description_pkg, "rviz", "viewit.rviz"])]
    )
    

    return LaunchDescription([
        DeclareLaunchArgument("description_file", default_value="robot.urdf.xacro"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        robot_state_pub,
        joint_state_pub_gui,
        rviz        
    ])
