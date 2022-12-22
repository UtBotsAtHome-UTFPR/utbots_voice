#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import rospkg

from os import system, path
import shutil
import pandas
from playsound import playsound


class SpeechSynthesisNode:

    # Init
    def __init__(self):
        rospy.init_node('tts', anonymous=True)
        rospy.loginfo("[TTS] Mimic TTS node init")

        # Parameters
        self.param_voice = rospy.get_param(
            '~voice', default="en_US/hifi-tts_low")
        rospy.loginfo("[TTS] Voice: {}".format(self.param_voice))

        # Gets path of this package
        self.packagePath = rospkg.RosPack().get_path('utbots_voice')
        rospy.loginfo("[TTS] Package path: {}".format(self.packagePath))

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
        self.TextToSpeech("Hello there.")
        self.TextToSpeech("I am Apollo.")

        # Loop
        self.loopRate = rospy.Rate(30)
        self.MainLoop()

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
        print("\n--- TEXT TO SPEECH")
        text = text.replace("'", "").replace("\"", "")

        csvContent = self.GetCsvContent(shouldPrint=False)
        NewWav = True

        # Finds out if file is already indexed. If it is, Will play it before resynthesizing speech
        for index, row in csvContent.iterrows():
            if row["engine"] == "mimic" and row["voice"] == self.param_voice and row["phrase"] == text and row["language"] == "en_US":
                NewWav = False
                rospy.loginfo("[TTS] Wav already indexed!")
                self.PlayWav(row["wav"])

        # If wav file is not indexed yet, it will create a new wav, index it in a new row in the csv, and play the wav
        if NewWav == True:
            rospy.loginfo("[TTS] Wav not indexed yet")
            rows = csvContent.shape[0]  # number of rows in the csv
            wav = "{}.wav".format(rows)  # file name of new .wav
            self.MakeNewRowInCsv(text, self.param_voice, wav, csvContent)
            self.SaveSpeechToWav(wav, text)
            self.PlayWav(wav)
        print("---\n")

    # Appends row to csv
    def MakeNewRowInCsv(self, text, voice, wav, csvContent):
        # Defines new row
        newRow = pandas.DataFrame({
            "engine": ["mimic"],
            "voice": [voice],
            "phrase": [text],
            "language": ["en_US"],
            "wav": [wav]
        })

        # Appends new row and saves the new csv
        csvContent = csvContent.append(newRow).reset_index(drop=True)
        csvContent.to_csv(self.csvPath, index=False, sep="|")
        rospy.loginfo("[TTS] Saved new dataframe to {}".format(self.csvPath))
        csvContent = self.GetCsvContent(shouldPrint=False)

    # Saves speech generated to wav file
    def SaveSpeechToWav(self, wav, text):
        command = "mimic3 --voice {} '{}' > {}".format(
            self.param_voice, text, self.GetWavPath(wav))
        rospy.loginfo("[TTS] System call: {}".format(command))
        system(command)

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
        rospy.loginfo("[TTS] Looping")
        while rospy.is_shutdown() == False:
            self.loopRate.sleep()


if __name__ == "__main__":
    SpeechSynthesisNode()
