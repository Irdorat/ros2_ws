from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package= 'turtlesim',
            namespace= 'turtlesim1',
            executable= 'turtlesim_node',
            name= 'sim',
            arguments= ['--ros-args', '--log-level', 'info']
        ),
        Node(
            package= 'turtlesim',
            namespace= 'turtlesim2',
            executable= 'turtlesim_node',
            name= 'sim',
            arguments= ['--log-level', 'warn']
        ),
        Node(
            package= 'turtlesim',
            namespace= 'mimic',
            executable= 'mimic',
            remappings= [
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ])
    ])
