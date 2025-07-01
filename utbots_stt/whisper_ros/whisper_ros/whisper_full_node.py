#!/usr/bin/env python3
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from rclpy.action import ActionServer
from utbots_msgs.msg import StringArray
from utbots_srvs.srv import LoadModel


import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
# from utbots_srvs.srv import ManageModel  # Replace with your service definition

import numpy as np
from ament_index_python.packages import get_package_share_directory
from whisper_ros.whisper_class import *
from rcl_interfaces.msg import ParameterDescriptor
from utbots_actions.action import Transcription

import threading

class WhisperTranscriber(Node):
    def __init__(self):
        super().__init__('whisper_transcriber')

        self.declare_parameter('whisper_verbose', False , ParameterDescriptor(description='whisper_verbose in Bool'))
        self.verbose = self.get_parameter('whisper_verbose').get_parameter_value().bool_value
        self.declare_parameter('enable_synchronous_startup', False , ParameterDescriptor(description='start with sync transcription'))
        self.sync_whisper = self.get_parameter('enable_synchronous_startup').get_parameter_value().bool_value

        self.declare_parameter('timer_period', 0.1 , ParameterDescriptor(description='timer_period'))
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value

        self.declare_parameter('wait_timeout', 15.0 , ParameterDescriptor(description='wait_timeout'))
        self.wait_timeout = self.get_parameter('wait_timeout').get_parameter_value().double_value


        self.declare_parameter('whisper_model', "openai/whisper-tiny.en" , ParameterDescriptor(description='whisper_model'))
        # self.startup_model = self.get_parameter('whisper_model').get_parameter_value().string_value
        
        self.declare_parameter('whisper_startup', True , ParameterDescriptor(description='whisper_startup'))
        # self.startup = self.get_parameter('whisper_startup').get_parameter_value().string_value

        # WhisperASR_parameter={
        #     "brand": "Ford",
        #     "model": "Mustang",
        #     "year": 1964
        # }

        # Initialize variables
        
        if(self.get_parameter('whisper_startup').get_parameter_value().bool_value):
            self.model = WhisperASR(model=self.get_parameter('whisper_model').get_parameter_value().string_value)
            self.model_loaded = True
        else:
            self.model = None
            self.model_loaded = True

        self.package_share_directory = get_package_share_directory('whisper_ros')


        # Create callback groups for parallel execution
        # self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.audio_cb_group = MutuallyExclusiveCallbackGroup()

        #register new message during action
        self.latest_msg_audio = None
        self.msg_event = threading.Event()

        # Timer for synchronous processing
        
        # Create publisher for transcriptions
        self.transcription_pub = self.create_publisher(
            String,
            'whispered',
            10
        )

        # Create subscriber for audio data
        self.audio_sub = self.create_subscription(
            Int16MultiArray,  # Replace with your actual audio message type
            'audio',
            self.audio_callback,
            10,
            # callback_group=self.audio_cb_group
        )

        # Service to enable/disable synchronous processing
        self.srv_enable = self.create_service(
            SetBool,
            '/utbots/voice/enable_transcription',
            self.enable_transcription_cb,
            callback_group=self.audio_cb_group

        )
        
        # Action server initialization
        self._action_server = ActionServer(
            self,
            Transcription,
            '/utbots/transcription',
            self.execute_callback,
            callback_group=self.audio_cb_group
        )
        
        #
        # self.sync_whisper=False
        # Timer for synchronous processing
        self.timer = self.create_timer(
            self.timer_period,
            self.sync_whisper_cb,
            callback_group=self.audio_cb_group,
            )

        
        #Create service for model management
        self.model_service = self.create_service(
            LoadModel,
            'whisper_model',
            self.manage_model_callback,
            callback_group=self.audio_cb_group,
        )
        
        self.get_logger().info("Whisper Transcriber Node initialized")
    
    def enable_transcription_cb(self, request, response):
        self.sync_whisper = request.data
        response.success = True
        response.message = "sync_whisper enabled" if self.sync_whisper else "sync_whisper disabled"
        return response

    def manage_model_callback(self, request, response):
        """Service callback for model management"""
        if request.load_model:
            response.success = self.load_model(request.data)
        else:
            response.success = self.unload_model()
        return response
   
    def audio_callback(self, msg):
        self.latest_msg_audio = np.frombuffer(msg.data, dtype=np.int16) 
        self.msg_event.set()        

    def sync_whisper_cb(self):
        if(self.sync_whisper & (not isinstance(self.latest_msg_audio,type(None)))):
            if not self.model_loaded:
                self.get_logger().warn("Model not loaded, skipping audio processing")
                return
            try:
                self.msg_event.clear()
                transcription = self.model.transcribe(self.latest_msg_audio)["text"]
                #limpa ultimo audio e eventos
                self.latest_msg_audio=None
                self.msg_event.clear()
                
                transcript_msg = String()
                transcript_msg.data = transcription
                self.transcription_pub.publish(transcript_msg)
                if(self.verbose):
                    self.get_logger().info(f"Transcription: {transcription}")
            except Exception as e:
                self.get_logger().error(f"Error processing audio: {str(e)}")

    async def execute_callback(self, goal_handle):
        
        self.get_logger().info('Goal received. Waiting for new message...')
        
        result = Transcription.Result()

        self.msg_event.clear()
        # audio = goal_handle.request.audio
        got_msg = self.msg_event.wait(timeout=self.wait_timeout)
        if not got_msg:
            self.get_logger().info('No new message within timeout.')
            goal_handle.succeed()
            return result
        
        if(not isinstance(self.latest_msg_audio,type(None))):
            if not self.model_loaded:
                self.get_logger().warn("Model not loaded, skipping audio processing")
                goal_handle.abort()
                return result
            try:
                self.msg_event.clear()
                transcription = self.model.transcribe(self.latest_msg_audio)["text"]
                #limpa ultimo audio e eventos
                self.latest_msg_audio=None
                self.msg_event.clear()
                
                transcript_msg = String()
                transcript_msg.data = transcription
                result.text=transcript_msg
                if(self.verbose):
                    self.get_logger().info(f"Transcription: {transcription}")
                goal_handle.succeed()
                return result
            except Exception as e:
                self.get_logger().error(f"Error processing audio: {str(e)}")
                goal_handle.abort()
                return result
   
    def load_model(self, model_name):
        """Load the whisper model"""
        if(isinstance(self.model,WhisperASR)):
            return False
        try:
            self.get_logger().info(f"Loading whisper model: {model_name}")
            self.model = WhisperASR(model=model_name,verbose=self.verbose)
            self.model_loaded = True
            if(self.verbose):
                self.get_logger().info("Model loaded successfully")
            return True
        except Exception as e:
            if(self.verbose):
                self.get_logger().error(f"Failed to load model: {str(e)}")
            self.model_loaded = False
            return False
    
    def unload_model(self):
        """Unload the whisper model"""
        if(not (isinstance(self.model,WhisperASR))):
            return False
        try:
            self.get_logger().info("Unloading whisper model")
            del self.model
            self.model = None
            self.model_loaded = False
            self.get_logger().info("Model unloaded successfully")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to unload model: {str(e)}")
            return False

from rclpy.executors import MultiThreadedExecutor
def main(args=None):
    rclpy.init(args=args)
    
    transcriber_node = WhisperTranscriber()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(transcriber_node,executor)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        if transcriber_node.model_loaded:
            transcriber_node.unload_model()
        transcriber_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()