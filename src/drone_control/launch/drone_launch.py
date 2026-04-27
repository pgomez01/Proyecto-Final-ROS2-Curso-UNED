import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_share= get_package_share_directory('drone_control')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'drone_config.rviz')
    return LaunchDescription([
        # Nodo Base (C++)
        Node(
            package='drone_control',
            executable='base_node',
            name='drone_base'
        ),
        # Nodo Controller (C++)
        Node(
            package='drone_control',
            executable='controller_node',
            name='drone_controller'
        ),

        # TF Estática: Sensor LIDAR a 10cm sobre el dron [cite: 27]
        # Argumentos: x y z yaw pitch roll frame_id child_frame_id
        Node(
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
        ),
        #Nodo de Rviz2 con configuración cargada
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path]
            
        )
    ])