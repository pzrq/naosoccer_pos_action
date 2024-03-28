from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nao_pos_server',
            #namespace='turtlesim2',
            executable='nao_pos_action_server',
            name='nao_pos_action_server_node'
        ),
        Node(
            package='nao_pos_server',
            #namespace='turtlesim2',
            executable='nao_pos_action_client',
            name='nao_pos_action_client_node'
        ),
        Node(
            package='nao_pos_server',
            #namespace='turtlesim2',
            executable='nao_pos_publisher',
            name='nao_pos_publsher_node'
        ),
    ])