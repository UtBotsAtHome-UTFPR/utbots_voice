import rclpy
from rclpy.node import Node
PATH="/home/ehg2004/utbots_ws/src/utbots_voice/utbots_tts/ros_tts/"
from std_msgs.msg import String
from  ros_tts.tts_module_small import SpeechSynthModule
from ament_index_python.packages import get_packages_with_prefixes
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
import time
from utbots_actions.action import TextToSpeech

class CoquiTTSActionServer(Node):

    def __init__(self):
        super().__init__('coqui_tts_server')
        # self.package_share_directory = get_package_share_directory('ros_tts')
        
        # self.declare_parameter('speaking_mode', "optimus" , ParameterDescriptor(description='speaking_mode -> string'))
        # self.declare_parameter('language_mode', "en" , ParameterDescriptor(description='language_mode string -> ("en" or "pt-br")'))
        self.declare_parameter('model_name', "tts_models/en/ljspeech/tacotron2-DDC" , ParameterDescriptor(description='speaking_mode -> string'))
        self.declare_parameter('use_cuda', False , ParameterDescriptor(description='use_cuda -> Bool'))
        self.declare_parameter('verbose', False , ParameterDescriptor(description='verbose -> Bool'))
        self.declare_parameter('package_path', PATH , ParameterDescriptor(description='verbose -> string'))

        self.verbose=self.get_parameter('verbose').get_parameter_value().bool_value
        self.package_share_directory = self.get_parameter('package_path').get_parameter_value().string_value

        # self.tts_module = SpeechSynthModule(self.package_share_directory,
        #                                     param_speakingMode=self.get_parameter('speaking_mode').get_parameter_value().string_value,
        #                                     param_languageMode=self.get_parameter('language_mode').get_parameter_value().string_value,
        #                                     param_use_cuda=self.get_parameter('use_cuda').get_parameter_value().bool_value,
        #                                     verbose=self.verbose
        #                                     )
        self.tts_module = SpeechSynthModule(
                package_path=self.package_share_directory,
                model_name=self.get_parameter('model_name').get_parameter_value().string_value,
                use_cuda=self.get_parameter('use_cuda').get_parameter_value().bool_value,
                verbose=self.verbose
                )


        time.sleep(10)  # Pause execution for 5 seconds
        # if(self.verbose):
        #     self.get_logger().info("[TTS] Verbose enabled")
        #     self.get_logger().info("[TTS] Model path: {}".format(self.tts_module.param_model_path))
        #     self.get_logger().info("[TTS] Model config path: {}".format(
        #         self.tts_module.param_config_path))
        #     self.get_logger().info("[TTS] Vocoder path: {}".format(self.tts_module.param_vocoder_path))
        #     self.get_logger().info("[TTS] Vocoder config path: {}".format(
        #         self.tts_module.param_vocoder_config_path))
        #     self.get_logger().info("[TTS] Speakers file path: {}".format(
        #         self.tts_module.param_speakers_file_path))
        #     self.get_logger().info("[TTS] Use CUDA: {}".format(self.tts_module.param_use_cuda))
        #     self.get_logger().info("[TTS] Speaking mode: {}".format(
        #         self.tts_module.param_speakingMode))
        #     self.get_logger().info("[TTS] Language mode: {}".format(
        #         self.tts_module.param_languageMode))
        #     self.get_logger().info("[TTS] Index CSV: {}".format(self.tts_module.csvPath))
        self.get_logger().info("[TTS] Synthesizer ok")

        
        self._action_server = ActionServer(
            self,
            TextToSpeech,
            'tts',
            self.execute_callback)

    def execute_callback(self, goal_handle):
            
        self.get_logger().info('Executing goal...')
        text = goal_handle.request.text
        # self.param_istalking = rospy.set_param("/vad_node/is_robot_talking", True) 
        self.set_parameters([ rclpy.parameter.Parameter(
                    'is_robot_talking',
                    rclpy.Parameter.Type.BOOL,
                    True
                )])
        self.tts_module.speak(text)
        self.set_parameters([ rclpy.parameter.Parameter(
                        'is_robot_talking',
                    rclpy.Parameter.Type.BOOL,
                    False
                    )])
        result = TextToSpeech.Result()
        return result

def main(args=None):
    rclpy.init(args=args)

    coqui_ActionServer = CoquiTTSActionServer()

    rclpy.spin(coqui_ActionServer)
    rclpy.shutdown()


if __name__ == '__main__':
    main()