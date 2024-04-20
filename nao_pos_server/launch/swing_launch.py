from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nao_pos_server',
            #namespace='turtlesim2',
            executable='nao_pos_action_server',
            name='nao_pos_action_server_legs',
            remappings=[
                ('nao_pos_action/_action/feedback', 'nao_pos_action_legs/_action/feedback'),
                ('nao_pos_action/_action/status', 'nao_pos_action_legs/_action/status'),
                ('nao_pos_action/_action/cancel_goal', 'nao_pos_action_legs/_action/cancel_goal'),
                ('nao_pos_action/_action/get_result', 'nao_pos_action_legs/_action/get_result'),
                ('nao_pos_action/_action/send_goal', 'nao_pos_action_legs/_action/send_goal'),
            ],
        ),
        Node(
            package='nao_pos_server',
            #namespace='turtlesim2',
            executable='nao_pos_action_client',
            name='nao_pos_action_client_legs',
            remappings=[
                ('nao_pos_action/_action/feedback', 'nao_pos_action_legs/_action/feedback'),
                ('nao_pos_action/_action/status', 'nao_pos_action_legs/_action/status'),
                ('nao_pos_action/_action/cancel_goal', 'nao_pos_action_legs/_action/cancel_goal'),
                ('nao_pos_action/_action/get_result', 'nao_pos_action_legs/_action/get_result'),
                ('nao_pos_action/_action/send_goal', 'nao_pos_action_legs/_action/send_goal'),
                ('action_req', 'action_req_legs'),
            ],
        ),
        Node(
            package='nao_pos_server',
            #namespace='turtlesim2',
            executable='nao_pos_publisher',
            name='nao_pos_publisher_legs',
            remappings=[
                ('action_req', 'action_req_legs'),
            ],
        ),
    ])