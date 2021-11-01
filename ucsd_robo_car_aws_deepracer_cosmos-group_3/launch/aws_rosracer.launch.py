import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('ucsd_robo_car_aws_deepracer_cosmos'),
        'config',
        'deepracer_config.yaml')

    single_node_example = Node(
        package='ucsd_robo_car_aws_deepracer_cosmos',
        executable='single_node_example',
        parameters=[config])

    ld.add_action(single_node_example)
    
    return ld
