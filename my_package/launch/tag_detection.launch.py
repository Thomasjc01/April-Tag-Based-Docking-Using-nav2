
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Image rectification node
        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_node',
            remappings=[
                ('image', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info'),
                ('image_rect', '/camera/image_rect')
            ]
        ),
        
        # AprilTag detection node
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_detector',
            remappings=[
                ('image_rect', '/camera/image_rect'),
                ('camera_info', '/camera/camera_info')
            ],
            parameters=[{
                'image_transport': 'raw',
                'family': '36h11',
                'size': 0.162,
                'max_hamming': 0,
                'z_up': True
            }]
        )
    ])