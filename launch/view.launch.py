import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_config_dir = os.path.join(get_package_share_directory('rviz_color_code'), 'rviz')
    rviz_config_file = os.path.join(rviz_config_dir, 'view.rviz')

    return LaunchDescription([
        Node(
            package='rviz_color_code',
            executable='color_code',
            parameters=[
                {'sphere_radius': 0.2},
                {'space_length': 0.2},
                {'column_size': 31},
                {'division_number': 10}
            ]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])
