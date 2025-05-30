from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='vad_ros',
        #     executable='vad_node',
        #     name='vad_node',
        #     output='screen',
        #     emulate_tty=True,
        #     parameters=[
        #         {'vad_timeout': 5_000,
        #          'vad_threshold':0.5,
        #          'vad_verbose':True,

        #           }
        #     ]
        # ),
        Node(
            package='ros_tts',
            executable='tts_node',
            name='tts_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                #'speaking_mode':True,
                 #'speaking_mode':'en',
                 'package_path':"/home/ehg2004/utbots_ws/src/utbots_voice/utbots_tts/ros_tts/",
                 'model_name': "tts_models/en/ljspeech/tacotron2-DDC",
                 'use_cuda':False,
                 'verbose':True,
                  }
            ]
        )

    ])        
# self.declare_parameter('speaking_mode', "optimus" , ParameterDescriptor(description='speaking_mode -> string'))
#         self.declare_parameter('language_mode', "en" , ParameterDescriptor(description='language_mode string -> ("en" or "pt-br")'))
#         self.declare_parameter('use_cuda', False , ParameterDescriptor(description='use_cuda -> Bool'))
#         self.declare_parameter('verbose', False , ParameterDescriptor(description='verbose -> Bool'))
