from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'target_frame', default_value='turtle1',
            description='Target frame name.'
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            output='screen'
        ),
        
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='broadcaster3',
            parameters=[
                {'turtlename': 'turtle2'}
            ]
        ),
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='broadcaster2',
            parameters=[
                {'turtlename': 'turtle3'}
            ]
        ),
        
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_message_broadcaster',
            name='message_broadcaster',
        ),
        
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_message_broadcaster_1',
            name='message_broadcaster_1',
        ),
        
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_message_broadcaster_2',
            name='message_broadcaster_2',
        ),
        
    ])
