#!/usr/bin/env python3
# ROS
import rospy
from std_msgs.msg import String

# TTS
from pathlib import Path
from TTS.config import load_config
from TTS.utils.manage import ModelManager
from TTS.utils.synthesizer import Synthesizer

# Sound
from playsound import playsound

# Reference: https://github.com/coqui-ai/TTS/blob/dev/TTS/server/server.py
# TODO: testar modelos de https://github.com/Edresson/TTS-Portuguese-Corpus

''' 
    PARAMETERS NEEDED TO BE SET FOR TTS
        # Model
        self.param_model_path = None # .pth model path
        self.param_config_path = None # .json config path

        # Vocoder - If not defined, uses GL as vocoder. Need to install vocoder library before (WaveRNN)
        self.param_vocoder_path = None # .pth model path
        self.param_vocoder_config_path = None # .json config path
        
        # Speakers file (for multi-speaker model)
        self.param_speakers_file_path = None # .json file path

        # Language ID
        self.param_language_name = None

        # Target speaker wav
        self.param_target_speaker_wav = None

        # In case you want CUDA
        self.param_use_cuda = False
'''

class SpeechSynthesisNode:

    # Init
    def __init__(self):
        rospy.init_node('tts', anonymous=True)
        rospy.loginfo("[TTS] Coqui TTS node init")

        # TODO avoid this hardcoding
        self.yourtts_model = "/home/driver/.local/share/tts/tts_models--multilingual--multi-dataset--your_tts/model_file.pth"
        self.yourtts_config = "/home/driver/.local/share/tts/tts_models--multilingual--multi-dataset--your_tts/config.json"
        self.tacotron_model = "/home/driver/.local/share/tts/tts_models--en--ljspeech--tacotron2-DDC/model_file.pth"
        self.tacotron_config = "/home/driver/.local/share/tts/tts_models--en--ljspeech--tacotron2-DDC/config.json"
        self.hifigan_v2_model = "/home/driver/.local/share/tts/vocoder_models--en--ljspeech--hifigan_v2/model_file.pth"
        self.hifigan_v2_config = "/home/driver/.local/share/tts/vocoder_models--en--ljspeech--hifigan_v2/config.json"
        self.optimus_wav = "./optimus.wav"
        self.pavarotti_wav = "./pavarotti.wav"
        self.rocky_wav = "./rocky.wav"
        self.bock_happy_wav = "./bock_happy.wav"
        self.bock_sad_wav = "./bock_sad.wav"
        self.bock_mad_wav = "./bock_mad.wav"

        # Parameters (TODO get them from launch)
        speakingMode = "rocky"
        languageMode = "en" # en, pt-br
        self.ConfigTTSParameters(speakingMode, languageMode)

        # Loads model and configs
        self.synthesizer = Synthesizer(
            tts_checkpoint = self.param_model_path,
            tts_config_path = self.param_config_path,
            tts_speakers_file = self.param_speakers_file_path,
            tts_languages_file = None,
            vocoder_checkpoint = self.param_vocoder_path,
            vocoder_config = self.param_vocoder_config_path,
            encoder_checkpoint = "",
            encoder_config = "",
            use_cuda = self.param_use_cuda,)
        rospy.loginfo("[TTS] Synthesizer ok")

        use_multi_speaker = hasattr(self.synthesizer.tts_model, "num_speakers") and (
            self.synthesizer.tts_model.num_speakers > 1 or self.synthesizer.tts_speakers_file is not None)
        rospy.loginfo("[TTS] use_multi_speaker: {}".format(use_multi_speaker))

        speaker_manager = getattr(self.synthesizer.tts_model, "speaker_manager", None)
        rospy.loginfo("[TTS] speaker_manager: {}".format(speaker_manager))
        
        use_gst = self.synthesizer.tts_config.get("use_gst", False)
        rospy.loginfo("[TTS] use_gst: {}".format(use_gst))

        # Subscribers
        self.sub_text = rospy.Subscriber("/tts/text", String, self.Callback)

        # Says hello
        if languageMode == "en":
            self.Say("Hello! I am Apollo, your service robot. Call me if you need anything.")
        elif languageMode == "pt-br":
            self.Say("Olá! Eu sou o Apollo, o seu robô de serviço. Me chame se precisar.")
        else:
            self.Say("Undefined language mode.")

        # Loop
        self.loopRate = rospy.Rate(30)
        self.MainLoop()

    def ConfigTTSParameters(self, speakingMode, language_name):
        self.param_use_cuda = False

        if speakingMode == "tacotron":
            self.SetTTSParameters(self.tacotron_model, self.tacotron_config, self.hifigan_v2_model, self.hifigan_v2_config, None, language_name, None)
        elif speakingMode == "optimus":
            self.SetTTSParameters(self.yourtts_model, self.yourtts_config, None, None, None, language_name, self.optimus_wav)
        elif speakingMode == "pavarotti":
            self.SetTTSParameters(self.yourtts_model, self.yourtts_config, None, None, None, language_name, self.pavarotti_wav)
        elif speakingMode == "rocky":
            self.SetTTSParameters(self.yourtts_model, self.yourtts_config, None, None, None, language_name, self.rocky_wav)
        elif speakingMode == "bock_happy":
            self.SetTTSParameters(self.yourtts_model, self.yourtts_config, None, None, None, language_name, self.bock_happy_wav)
        elif speakingMode == "bock_sad":
            self.SetTTSParameters(self.yourtts_model, self.yourtts_config, None, None, None, language_name, self.bock_sad_wav)
        elif speakingMode == "bock_mad":
            self.SetTTSParameters(self.yourtts_model, self.yourtts_config, None, None, None, language_name, self.bock_mad_wav)

        rospy.loginfo("[TTS] Model path: {}".format(self.param_model_path))
        rospy.loginfo("[TTS] Model config path: {}".format(self.param_config_path))
        rospy.loginfo("[TTS] Vocoder path: {}".format(self.param_vocoder_path))
        rospy.loginfo("[TTS] Vocoder config path: {}".format(self.param_vocoder_config_path))
        rospy.loginfo("[TTS] Speakers file path: {}".format(self.param_speakers_file_path))
        rospy.loginfo("[TTS] Use CUDA: {}".format(self.param_use_cuda))        

    def SetTTSParameters(self, model_path, config_path, vocoder_path, vocoder_config_path, speakers_file_path, language_name, target_speaker_wav):
        self.param_model_path = model_path
        self.param_config_path = config_path
        self.param_vocoder_path = vocoder_path
        self.param_vocoder_config_path = vocoder_config_path
        self.param_speakers_file_path = speakers_file_path
        self.param_language_name = language_name
        self.param_target_speaker_wav = target_speaker_wav

    # Callback for received text
    def Callback(self, msg):
        rospy.loginfo("[TTS] Callback: text is '{}'".format(msg.data))
        self.Say(msg.data)

    # Converts text input to audio output
    def Say(self, text):

        # Generates waveforms
        rospy.loginfo("[TTS] Generating .wav for {}".format(text))
        wavs = self.synthesizer.tts(
            text = text,
            speaker_name = "",
            language_name = self.param_language_name,
            speaker_wav = self.param_target_speaker_wav,
            style_wav = None,
            style_text = None,
            reference_wav = None,
            reference_speaker_name = None)

        # Saves .wav file
        wavPath = "/tmp/speech.wav"
        rospy.loginfo("[TTS] Saving .wav at {}'".format(wavPath))
        self.synthesizer.save_wav(wavs, wavPath)
        
        # Plays .wav
        rospy.loginfo("[TTS] Playing audio from {}".format(wavPath))
        playsound(wavPath)

        rospy.loginfo("[TTS] Played audio")

    # Main loop
    def MainLoop(self):
        rospy.loginfo("[TTS] Looping...")
        while rospy.is_shutdown() == False:
            self.loopRate.sleep()    

if __name__ == "__main__":
    SpeechSynthesisNode()