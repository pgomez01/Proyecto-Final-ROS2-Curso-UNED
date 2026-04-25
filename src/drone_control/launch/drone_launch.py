from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo Base (C++)
        Node(
            package='drone_control',
            executable='base_node',
            name='drone_base'
        ),
        # TF Estática: Sensor LIDAR a 10cm sobre el dron [cite: 27]
        # Argumentos: x y z yaw pitch roll frame_id child_frame_id
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['-xx','0',
                          '--y','0',
                          '--z','0.1',
                          '--roll', '0','pitch', '0',
                          '--yaw','0',
                          '--frame_id','base_link',
                          '--child-frame-id', 'lidar_link']
        )
    ])