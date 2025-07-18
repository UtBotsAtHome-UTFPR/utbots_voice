from launch import LaunchDescription
from launch_ros.actions import Node

# ros2 launch vad_ros stt_launch.py whisper_sync_start:='true' 
# vad_timeout:='1_000' vad_threshold:='0.75'  whisper_def_model:='openai/whisper-large-v3-turbo' 
# whisper_startup:=True verbose:=true

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import OpaqueFunction

def stt_launch_setup(context, *args, **kwargs):
    #GLOBAL
    verbose = LaunchConfiguration('verbose',default="false")#.perform(context)
    #VAD
    vad_timeout = int(LaunchConfiguration('vad_timeout',default='3_000').perform(context))
    vad_threshold = float(LaunchConfiguration('vad_threshold',default='0.5').perform(context))
    disable_denoiser = LaunchConfiguration('disable_denoiser',default='false')#.perform(context)
    #WHISPER
    whisper_startup = LaunchConfiguration('whisper_startup',default='true')#.perform(context)
    whisper_sync_start = LaunchConfiguration('whisper_sync_start',default='false')#.perform(context)
    whisper_stt_timeout = float(LaunchConfiguration('whisper_stt_timeout',default='15.0').perform(context))
    whisper_def_model = LaunchConfiguration('whisper_def_model',default='openai/whisper-large-v3-turbo')#.perform(context)
    #NEED TO BE EVALUATED:
    whisper_cb_timer = float(LaunchConfiguration('whisper_cb_timer',default='0.1').perform(context))
        
    return([
        Node(
            package='vad_ros',
            executable='vad_node',
            name='vad_node',
            output='screen',
            emulate_tty=True,
            parameters=[
            {
                'vad_timeout': vad_timeout if (vad_timeout) < 5_000 and (vad_timeout) > 0.0 else 1_500,
                'vad_threshold': vad_threshold if (vad_threshold) < 1.0 and (vad_threshold) > 0.0 else 0.75,
                'vad_verbose':verbose,
                'disable_denoiser':disable_denoiser,
                }
            ]
        ),
        Node(
            package='whisper_ros',
            executable='whisper_full_node',
            name='whisper_node',
            output='screen',
            emulate_tty=True,
            parameters=[
            {
                'whisper_verbose':verbose,
                'enable_synchronous_startup':whisper_sync_start,
                'timer_period':whisper_cb_timer,
                'whisper_model':whisper_def_model,                
                'whisper_startup':whisper_startup,
                'wait_timeout':whisper_stt_timeout,
                }
            ]
        )
        ])


def generate_launch_description():
    return LaunchDescription([
        #GLOBAL,
        DeclareLaunchArgument('verbose',default_value="false"),
        #VAD,
        DeclareLaunchArgument('vad_timeout',default_value='1_500'),
        DeclareLaunchArgument('vad_threshold',default_value='0.75'),
        DeclareLaunchArgument('disable_denoiser',default_value='false'),
        #WHISPER,
        DeclareLaunchArgument('whisper_startup',default_value='true'),
        DeclareLaunchArgument('whisper_sync_start',default_value='false'),
        DeclareLaunchArgument('whisper_stt_timeout',default_value='15.0'),
        DeclareLaunchArgument('whisper_def_model',default_value='openai/whisper-large-v3-turbo'),
        #NEED TO BE EVALUATED:,
        DeclareLaunchArgument('whisper_cb_timer',default_value='0.1'),

        OpaqueFunction(function=stt_launch_setup),
    ])