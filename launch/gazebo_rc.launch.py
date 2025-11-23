import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Obtener el directorio del paquete
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    # Ruta al archivo URDF
    urdf_file = os.path.join(pkg_dir, 'urdf', 'rc_model.urdf')

    # Leer el contenido del URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Reemplazar $(find rc_model) con la ruta real
    robot_desc = robot_desc.replace('$(find rc_model)', pkg_dir)

    # Parámetros
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Nodo robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )

    # Incluir Gazebo (GZ Sim)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Nodo para spawnear el robot en Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'rc_model',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    # Bridge entre ROS2 y Gazebo para clock y joint states
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )

    # Bridge para joint commands (ROS2 -> Gazebo)
    bridge_joints = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/rc_model/joint/prismatic_x/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/rc_model/joint/prismatic_y/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen'
    )

    # Nodo de control GUI para mover los joints
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # No necesitamos relay por ahora, los joints se controlan manualmente con la GUI

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity,
        bridge_clock,
        bridge_joints,
        joint_state_publisher_gui
    ])
