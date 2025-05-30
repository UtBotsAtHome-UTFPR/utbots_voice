from TTS.api import TTS
from playsound import playsound
import pandas as pd
from pathlib import Path
import time
PATH="/home/ehg2004/utbots_ws/src/utbots_voice/utbots_tts/ros_tts/"

class SpeechSynthModule:
    def __init__(self,
                 package_path: str = PATH,
                 model_name: str = "tts_models/en/ljspeech/tacotron2-DDC",
                 use_cuda: bool = True,
                 verbose: bool = True):
        """
        package_path: root path to your ROS package
        model_name: Coqui model identifier or local path
        use_cuda: whether to use GPU acceleration
        verbose: whether to greet on init
        """
        self.package_path = Path(package_path)
        self.verbose = verbose
        self.use_cuda = use_cuda
        
        # Initialize Coqui TTS
        self.tts = TTS(model_name=model_name, progress_bar=False, gpu=self.use_cuda)
        
        # Prepare cache CSV
        self.csv_path = self.package_path / "resources/audios/indexed/index.csv"
        if not self.csv_path.exists():
            pd.DataFrame(columns=["engine","voice","phrase","language","wav"]) \
              .to_csv(self.csv_path, index=False, sep="|")
        self.cache = pd.read_csv(self.csv_path, sep="|")
        
        # Optional greeting
        if self.verbose:
            time.sleep(1)
            self.speak("Hello there.")

    def save_cache(self):
        self.cache.to_csv(self.csv_path, index=False, sep="|")

    def speak(self, text: str, language: str = "en"):
        """
        Synthesize (or retrieve) and play a phrase.
        text: phrase to speak
        language: ISO code (e.g., 'en', 'pt-br')
        """
        clean_text = text.replace("'", "").replace('"', "")
        # Check cache
        matched = self.cache[
            (self.cache.engine == "coqui") &
            (self.cache.voice == self.tts.model_name) &
            (self.cache.phrase == clean_text) &
            (self.cache.language == language)
        ]
        if not matched.empty:
            wav_file = matched.iloc[0].wav
            playsound(str(self.package_path / "resources/audios/indexed" / wav_file))
            return
        
        # Not cached: generate
        idx = len(self.cache)
        wav_name = f"{idx}.wav"
        out_path = self.package_path / "resources/audios/indexed" / wav_name
        out_path.parent.mkdir(parents=True, exist_ok=True)

        # Synthesize to file
        self.tts.tts_to_file(text=clean_text, file_path=str(out_path), speaker_wav=None)

        # Update cache
        new_row = { 
            "engine": "coqui",
            "voice": self.tts.model_name,
            "phrase": clean_text,
            "language": language,
            "wav": wav_name
        }
        self.cache = pd.concat([self.cache, pd.DataFrame([new_row])], ignore_index=True)
        self.save_cache()

        # Play
        playsound(str(out_path))

# from speech_synth_module import SpeechSynthModule  # Adjust import if needed
import os

def main():
    # Set the absolute path to the package directory
    # package_path = os.path.abspath(".")
    package_path=PATH
    # Initialize the speech synthesis module
    tts_module = SpeechSynthModule(
    )

    # Example phrases to test
    test_phrases = [
        "Hello there.",
        "This is a test of the TTS system.",
        "Let's synthesize some more speech.",
        "Monkey, banana, elephant"
    ]

    for phrase in test_phrases:
        print(f"Synthesizing: {phrase}")
        tts_module.speak(phrase)

if __name__ == "__main__":
    main()
