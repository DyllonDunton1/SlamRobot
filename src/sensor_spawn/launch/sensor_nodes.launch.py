from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # LiDAR node
        Node(
            package='hls_lfcd_lds_driver',
            executable='hlds_laser_publisher',
            name='hlds_laser_publisher',
            parameters=[{'port': '/dev/ttyUSB0'}],
            output='screen'
        ),
        # Static transform: odom → base_footprint
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
            output='screen'
        ),
        # Static transform: base_footprint → base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
            output='screen'
        ),
        # Static transform: base_link → laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),

        # Camera node
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='usb_camera',
            parameters=[{'video_device': '/dev/video0'}],
            output='screen'
        ),
        # SLAM Toolbox (online async mode)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        )
    ])
