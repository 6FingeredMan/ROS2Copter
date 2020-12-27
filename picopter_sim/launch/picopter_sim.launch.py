from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simulator_pkg',
            #namespace='sim',
            executable='simulator',
            name='simulator1'
        ),
        Node(
            package='navigator_pkg',
            #namespace='sim',
            executable='navigator',
            name='navigator1'
        ),
        Node(
            package='autopilot_pkg',
            #namespace='sim',
            executable='autopilot',
            name='autopilot1'
        ),
        Node(
            package='ahrs_sim_pkg',
            #namespace='sim',
            executable='ahrs_sim',
            name='ahrs_sim1'
        )
    ])