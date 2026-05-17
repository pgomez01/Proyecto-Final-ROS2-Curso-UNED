import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    
    pkg_name = 'drone_control'
    pkg_share= get_package_share_directory(pkg_name)
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'drone_config.rviz')
    xacro_file = os.path.join(pkg_share, 'urdf', 'drone.urdf.xacro')
    world_path = os.path.join(pkg_share,'worlds', 'walls.sdf')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # Nodo Robot State Publisher: Lee el URDF y publica la geometría 
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    # Lanzar Gazebo (Mundo vacío por ahora)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'),'launch','gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )
     
    # Invocar el dron dentro de Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic','robot_description','-entity', 'dron_uned','-z', '0.2'],
        output='screen'
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        output='screen'
    )
    # Nodo Base (C++)
    base_node = Node(
        package='drone_control',
        executable='base_node',
        name='base_node'
        )
    # Nodo Controller (C++)
    controller_node = Node(
        package='drone_control',
        executable='controller_node',
        name='controller_node'
    )
    # Nodo de Seguridad
    security_node = Node(
        package='drone_control',
        executable = 'security_node',
        name='security_node'

    )
    # TF Estática: Sensor LIDAR a 10cm sobre el dron [cite: 27]
    # Argumentos: x y z yaw pitch roll frame_id child_frame_id
    """Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x','0',
                        '--y','0',
                        '--z','0.1',
                        '--roll', '0',
                        '--pitch', '0',
                        '--yaw', '0',
                        '--frame-id','base_link',
                        '--child-frame-id','lidar_link']
    ),"""
    #Nodo de Rviz2 con configuración cargada
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )
    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        base_node,
        controller_node,
        security_node,
        rviz2
    ])
