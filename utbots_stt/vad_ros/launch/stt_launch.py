from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vad_ros',
            executable='vad_node',
            name='vad_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'vad_timeout': 5_000,
                 'vad_threshold':0.5,
                 'vad_verbose':True,

                  }
            ]
        ),
        Node(
            package='whisper_ros',
            executable='whisper_node',
            name='whisper_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'whisper_verbose':True,

                  }
            ]
        )

    ])