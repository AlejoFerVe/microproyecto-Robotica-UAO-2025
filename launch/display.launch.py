import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

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

    # Nodo de control GUI para mover los joints
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'display.rviz')]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui,
        rviz_node
    ])
