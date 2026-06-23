#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ros_tts.tts_module_small import SpeechSynthModule
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer
from rcl_interfaces.msg import ParameterDescriptor
import time
from utbots_actions.action import TextToSpeech
from std_srvs.srv import SetBool
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import asyncio
import os

# Use o virtual environment para imports específicos do TTS
import sys
VENV_PYTHON = "/home/joao/tts_env/bin/python3"
if os.path.exists(VENV_PYTHON):
    # Adiciona o site-packages do venv ao path
    VENV_SITE_PACKAGES = "/home/joao/tts_env/lib/python3.10/site-packages"
    if VENV_SITE_PACKAGES not in sys.path:
        sys.path.insert(0, VENV_SITE_PACKAGES)

class CoquiTTSActionServer(Node):

    def __init__(self):
        super().__init__('coqui_tts_server')
        
        self.declare_parameter('model_name', "tts_models/en/ljspeech/tacotron2-DDC", ParameterDescriptor(description='speaking_mode -> string'))
        self.declare_parameter('use_cuda', False, ParameterDescriptor(description='use_cuda -> Bool'))
        self.declare_parameter('verbose', False, ParameterDescriptor(description='verbose -> Bool'))
        #self.declare_parameter('language','en',ParameterDescriptor(description='language -> string'))#ingles
        self.declare_parameter('language','pt-br',ParameterDescriptor(description='language -> string'))#portugues
        
        cb_group = MutuallyExclusiveCallbackGroup()

        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        
        # Get package path dynamically
        package_share_dir = get_package_share_directory('ros_tts')
        self.package_share_directory = os.path.join(package_share_dir, '../../../../src/utbots_voice/utbots_tts/ros_tts/')
        
        self.tts_module = SpeechSynthModule(
            package_path=self.package_share_directory,
            model_name=self.get_parameter('model_name').get_parameter_value().string_value,
            use_cuda=self.get_parameter('use_cuda').get_parameter_value().bool_value,
            verbose=self.verbose,
            language=self.get_parameter('language').get_parameter_value().string_value,
        )
        
        self.disable_vad_cli = self.create_client(
            SetBool,
            '/utbots/disable_vad',
            callback_group=cb_group
        )
        
        self._action_server = ActionServer(
            self,
            TextToSpeech,
            '/utbots/tts',
            self.execute_callback,
            callback_group=cb_group
        )
        
        self.get_logger().info("[TTS] Synthesizer ok")

    async def send_request(self, disable_vad: bool):
        while not self.disable_vad_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /utbots/disable_vad...')
            await asyncio.sleep(1.0)

        req = SetBool.Request()
        req.data = disable_vad
        future = self.disable_vad_cli.call_async(req)
        await future
        result = future.result()
        if result:
            self.get_logger().info(f"Service response: {result.message}")
        return result

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        try:
            text = str(goal_handle.request.text.data)

            if self.disable_vad_cli.service_is_ready():
                self.get_logger().info(f"Sending request to disable VAD")
                await self.send_request(True)

            self.tts_module.speak(text)

            if self.disable_vad_cli.service_is_ready():
                self.get_logger().info(f"Sending request to enable VAD")
                await self.send_request(False)

            goal_handle.succeed()
        except Exception as e:
            self.get_logger().error(f"Error processing Goal: {str(e)}")
            goal_handle.abort()

        return TextToSpeech.Result()

from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    rclpy.init(args=args)
    coqui_ActionServer = CoquiTTSActionServer()
    rclpy.spin(coqui_ActionServer, MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == '__main__':
    main()