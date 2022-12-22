#!/usr/bin/env python3
# ROS
import rospy
from std_msgs.msg import String
import rospkg

# TTS
from pathlib import Path
from TTS.config import load_config
from TTS.utils.manage import ModelManager
from TTS.utils.synthesizer import Synthesizer

# Other
from playsound import playsound
import pandas
from os import system, path
import shutil

# Reference: https://github.com/coqui-ai/TTS/blob/dev/TTS/server/server.py

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

        # Gets path of this package
        self.packagePath = rospkg.RosPack().get_path('utbots_voice')
        rospy.loginfo("[TTS] Package path: {}".format(self.packagePath))

        # Sets paths to models and sample wavs
        self.SetPathsToModels()
        self.SetPathsToSampleWavs()

        # Parameters
        #   - English-only speaking modes: tacotron
        #   - Multilingual speaking modes: optimus, pavarotti, rocky, bock_happy, bock_sad, bock_mad
        #   - Language modes: en, pt-br
        self.param_speakingMode = rospy.get_param(
            '~speaking_mode', default='optimus')
        self.param_languageMode = rospy.get_param(
            '~language_mode', default="en")
        rospy.loginfo("[TTS] Speaking mode: {}".format(
            self.param_speakingMode))
        rospy.loginfo("[TTS] Language mode: {}".format(
            self.param_languageMode))
        self.ConfigTTSParameters(
            self.param_speakingMode, self.param_languageMode)

        # Synthesizer
        self.ConfigSynthesizer()
        rospy.sleep(5)

        # Opens csv used for caching wavs
        self.csvPath = self.packagePath + "/resources/wav/indexed/index.csv"
        rospy.loginfo("[TTS] Index CSV: {}".format(self.csvPath))

        # Fixes csv wav filenames
        self.DeleteRowsWithoutWavs()
        self.ReorderWavNames()

        # Subscribers
        self.sub_text = rospy.Subscriber(
            "/voice/tts/text", String, self.Callback)

        # Publishers
        self.pub_finishedAudio = rospy.Publisher(
            '/voice/tts/speech/finished', String, queue_size=1)

        # Says hello
        if self.param_languageMode == "en":
            self.TextToSpeech("Hello there.")
        elif self.param_languageMode == "pt-br":
            self.TextToSpeech("Olá, como vai?")

        # Loop
        self.loopRate = rospy.Rate(30)
        self.MainLoop()

    def SetPathsToModels(self):
        self.yourtts_model = "~/.local/share/tts/tts_models--multilingual--multi-dataset--your_tts/model_file.pth"
        self.yourtts_config = "~/.local/share/tts/tts_models--multilingual--multi-dataset--your_tts/config.json"
        self.tacotron_model = "~/.local/share/tts/tts_models--en--ljspeech--tacotron2-DDC/model_file.pth"
        self.tacotron_config = "~/.local/share/tts/tts_models--en--ljspeech--tacotron2-DDC/config.json"
        self.hifigan_v2_model = "~/.local/share/tts/vocoder_models--en--ljspeech--hifigan_v2/model_file.pth"
        self.hifigan_v2_config = "~/.local/share/tts/vocoder_models--en--ljspeech--hifigan_v2/config.json"

    def SetPathsToSampleWavs(self):
        # Paths to wavs
        self.optimus_wav = self.packagePath + "/resources/wav/samples/optimus.wav"
        self.pavarotti_wav = self.packagePath + "/resources/wav/samples/pavarotti.wav"
        self.rocky_wav = self.packagePath + "/resources/wav/samples/rocky.wav"
        self.bock_happy_wav = self.packagePath + \
            "/resources/wav/samples/bock_happy.wav"
        self.bock_sad_wav = self.packagePath + "/resources/wav/samples/bock_sad.wav"
        self.bock_mad_wav = self.packagePath + "/resources/wav/samples/bock_mad.wav"

    def ConfigSynthesizer(self):
        # Loads model and configs
        self.synthesizer = Synthesizer(
            tts_checkpoint=self.param_model_path,
            tts_config_path=self.param_config_path,
            tts_speakers_file=self.param_speakers_file_path,
            tts_languages_file=None,
            vocoder_checkpoint=self.param_vocoder_path,
            vocoder_config=self.param_vocoder_config_path,
            encoder_checkpoint="",
            encoder_config="",
            use_cuda=self.param_use_cuda)
        rospy.loginfo("[TTS] Synthesizer ok")

        use_multi_speaker = hasattr(self.synthesizer.tts_model, "num_speakers") and (
            self.synthesizer.tts_model.num_speakers > 1 or self.synthesizer.tts_speakers_file is not None)
        rospy.loginfo("[TTS] use_multi_speaker: {}".format(use_multi_speaker))

        speaker_manager = getattr(
            self.synthesizer.tts_model, "speaker_manager", None)
        rospy.loginfo("[TTS] speaker_manager: {}".format(speaker_manager))

        use_gst = self.synthesizer.tts_config.get("use_gst", False)
        rospy.loginfo("[TTS] use_gst: {}".format(use_gst))

    def ConfigTTSParameters(self, speakingMode, language_name):
        self.param_use_cuda = False

        if speakingMode == "tacotron":
            self.SetTTSParameters(self.tacotron_model, self.tacotron_config,
                                  self.hifigan_v2_model, self.hifigan_v2_config, None, language_name, None)
        elif speakingMode == "optimus":
            self.SetTTSParameters(self.yourtts_model, self.yourtts_config,
                                  None, None, None, language_name, self.optimus_wav)
        elif speakingMode == "pavarotti":
            self.SetTTSParameters(self.yourtts_model, self.yourtts_config,
                                  None, None, None, language_name, self.pavarotti_wav)
        elif speakingMode == "rocky":
            self.SetTTSParameters(self.yourtts_model, self.yourtts_config,
                                  None, None, None, language_name, self.rocky_wav)
        elif speakingMode == "bock_happy":
            self.SetTTSParameters(self.yourtts_model, self.yourtts_config,
                                  None, None, None, language_name, self.bock_happy_wav)
        elif speakingMode == "bock_sad":
            self.SetTTSParameters(self.yourtts_model, self.yourtts_config,
                                  None, None, None, language_name, self.bock_sad_wav)
        elif speakingMode == "bock_mad":
            self.SetTTSParameters(self.yourtts_model, self.yourtts_config,
                                  None, None, None, language_name, self.bock_mad_wav)

        rospy.loginfo("[TTS] Model path: {}".format(self.param_model_path))
        rospy.loginfo("[TTS] Model config path: {}".format(
            self.param_config_path))
        rospy.loginfo("[TTS] Vocoder path: {}".format(self.param_vocoder_path))
        rospy.loginfo("[TTS] Vocoder config path: {}".format(
            self.param_vocoder_config_path))
        rospy.loginfo("[TTS] Speakers file path: {}".format(
            self.param_speakers_file_path))
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
        self.TextToSpeech(msg.data)

    # Deletes rows that do not have corresponding wav files
    def DeleteRowsWithoutWavs(self):
        rospy.loginfo("[TTS] Deleting rows without corresponding wavs...")
        csvContent = self.GetCsvContent(shouldPrint=False)
        for index, row in csvContent.iterrows():
            oldWav = row["wav"]
            oldWavExists = path.isfile(self.GetWavPath(oldWav))
            if oldWavExists is False:
                csvContent = csvContent.drop(index=[index])
                csvContent.to_csv(self.csvPath, index=False, sep="|")

    # Reorders wav filenames
    def ReorderWavNames(self):
        rospy.loginfo("---")

        rospy.loginfo("[TTS] Reordering wav filenames")
        csvContent = self.GetCsvContent(shouldPrint=False)

        wavs = csvContent['wav'].tolist()
        newWavs = ["{}.wav".format(x) for x in range(0, len(wavs))]

        # Moves all wavs to /tmp/
        for wav in wavs:
            tmpPath = "/tmp/{}".format(wav)
            rospy.loginfo("[TTS] Moving {} to {}".format(
                self.GetWavPath(wav), tmpPath))
            shutil.move(self.GetWavPath(wav), tmpPath)

        # Moves wavs with new names back to their folder
        i = 0
        for wav in wavs:
            tmpPath = "/tmp/{}".format(wav)
            rospy.loginfo("[TTS] Moving {} to {}".format(
                tmpPath, self.GetWavPath(newWavs[i])))
            shutil.move(tmpPath, self.GetWavPath(newWavs[i]))
            i = i + 1

        # Drops old column and replaces it with new wav names
        csvContent.drop("wav", axis=1, inplace=True)
        csvContent["wav"] = newWavs
        csvContent.to_csv(self.csvPath, index=False, sep="|")

        rospy.loginfo("---")

    # Returns csv as a pandas dataframe
    def GetCsvContent(self, shouldPrint):
        csvContent = pandas.read_csv(self.csvPath, delimiter='|')
        if shouldPrint is True:
            rospy.loginfo("[TTS] csv content:")
            print(csvContent)
        return csvContent

    # Converts text input to audio output
    def TextToSpeech(self, text):
        text = text.replace("'", "").replace("\"", "")

        csvContent = self.GetCsvContent(shouldPrint=False)
        NewWav = True

        # Finds out if file is already indexed. If it is, Will play it before resynthesizing speech
        for index, row in csvContent.iterrows():
            if row["engine"] == "coqui" and row["voice"] == self.param_speakingMode and row["phrase"] == text and row["language"] == self.param_language_name:
                NewWav = False
                rospy.loginfo("[TTS] Wav already indexed!")
                self.PlayWav(row["wav"])

        # If wav file is not indexed yet, it will create a new wav, index it in a new row in the csv, and play the wav
        if NewWav == True:
            rospy.loginfo("[TTS] Wav not indexed yet")
            rows = csvContent.shape[0]  # number of rows in the csv
            wav = "{}.wav".format(rows)  # file name of new .wav
            self.MakeNewRowInCsv(
                text, self.param_speakingMode, wav, csvContent)
            self.SaveSpeechToWav(wav, text)
            self.PlayWav(wav)
        print("---\n")

    # Appends row to csv
    def MakeNewRowInCsv(self, text, voice, wav, csvContent):
        # Defines new row
        newRow = pandas.DataFrame({
            "engine": ["coqui"],
            "voice": [voice],
            "phrase": [text],
            "language": [self.param_languageMode],
            "wav": [wav]
        })

        # Appends new row and saves the new csv
        csvContent = csvContent.append(newRow).reset_index(drop=True)
        csvContent.to_csv(self.csvPath, index=False, sep="|")
        rospy.loginfo("[TTS] Saved new dataframe to {}".format(self.csvPath))
        csvContent = self.GetCsvContent(shouldPrint=False)

    # Saves speech generated to wav file
    def SaveSpeechToWav(self, wav, text):
        # Generates waveforms
        rospy.loginfo("[TTS] Generating .wav for {}".format(text))
        generatedWav = self.synthesizer.tts(
            text=text,
            speaker_name="",
            language_name=self.param_language_name,
            speaker_wav=self.param_target_speaker_wav,
            style_wav=None,
            style_text=None,
            reference_wav=None,
            reference_speaker_name=None)

        # Saves .wav file
        wavPath = self.GetWavPath(wav)
        rospy.loginfo("[TTS] Saving .wav at {}'".format(wavPath))
        self.synthesizer.save_wav(generatedWav, wavPath)

    # Plays wav file (returns True if succeeds)
    def PlayWav(self, wav):
        rospy.loginfo("[TTS] Playing {}".format(wav))
        playsound(self.GetWavPath(wav))
        rospy.loginfo("[TTS] Played audio")
        self.pub_finishedAudio.publish(String("ok"))

    # Gets full wav path
    def GetWavPath(self, wav):
        return "{}/resources/wav/indexed/{}".format(self.packagePath, wav)

    # Main loop
    def MainLoop(self):
        rospy.loginfo("[TTS] Looping...")
        while rospy.is_shutdown() == False:
            self.loopRate.sleep()


if __name__ == "__main__":
    SpeechSynthesisNode()
