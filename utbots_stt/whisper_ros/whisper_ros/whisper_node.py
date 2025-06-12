#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
# from utbots_srvs.srv import ManageModel  # Replace with your service definition
from utbots_srvs.srv import LoadModel


import pywhispercpp as whisper
import numpy as np
from ament_index_python.packages import get_package_share_directory
from whisper_ros.whisper_class import *
from rcl_interfaces.msg import ParameterDescriptor


class WhisperTranscriber(Node):
    def __init__(self):
        super().__init__('whisper_transcriber')

        self.declare_parameter('whisper_verbose', False , ParameterDescriptor(description='whisper_verbose in Bool'))
        self.vebose = self.get_parameter('whisper_verbose').get_parameter_value().bool_value

        
        # Initialize variables
        # self.model = None
        self.model_loaded = True
        
        self.package_share_directory = get_package_share_directory('whisper_ros')

        self.model = WhisperASR()
        
        # Create callback groups for parallel execution
        # self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.audio_cb_group = MutuallyExclusiveCallbackGroup()
        
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
            callback_group=self.audio_cb_group
        )
        
        #Create service for model management
        self.model_service = self.create_service(
            LoadModel,
            'manage_whisper_model',
            self.manage_model_callback,
            callback_group=self.service_cb_group
        )
        
        self.get_logger().info("Whisper Transcriber Node initialized")
    
    def load_model(self, model_name):
        """Load the whisper model"""
        if(isinstance(self.model,WhisperASR)):
            return False
        try:
            self.get_logger().info(f"Loading whisper model: {model_name}")
            self.model = WhisperASR(model_name)
            self.model_loaded = True
            if(self.vebose):
                self.get_logger().info("Model loaded successfully")
            return True
        except Exception as e:
            if(self.vebose):
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
    
    def manage_model_callback(self, request, response):
        """Service callback for model management"""
        if request.load_model:
            response.success = self.load_model(request.data)
        else:
            response.success = self.unload_model()
        return response
    
    def audio_callback(self, msg):
        """Process incoming audio data and transcribe"""
        if not self.model_loaded:
            
            self.get_logger().warn("Model not loaded, skipping audio processing")
            return
        
        try:
            # Convert audio data to numpy array of int16
            # Assuming msg.data is your int16 array
            # np.frombuffer
            msg_=msg.data

            # self.get_logger().info(f"Type:{msg_} ")

            audio_np = np.frombuffer(msg_, dtype=np.int16)#.squeeze()
            
            # Transcribe the audio
            results = self.model.transcribe(audio_np)
            # self.get_logger().info(type(results))
            if(self.vebose):
                self.get_logger().debug(results["text"])
            # results=results["text"]
            # self.get_logger().info(type(results))

            # transcription = ' '.join([segment for segment in results])
            transcription=results["text"]
            # Publish the transcription
            transcript_msg = String()
            transcript_msg.data = transcription
            self.transcription_pub.publish(transcript_msg)
            if(self.vebose):
                self.get_logger().debug(f"Transcription: {transcription}")
        except Exception as e:
            self.get_logger().error(f"Error processing audio: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    
    transcriber_node = WhisperTranscriber()
    
    try:
        rclpy.spin(transcriber_node)
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