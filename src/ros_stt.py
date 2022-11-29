#!/usr/bin/env python3
from os import system
import librosa
import soundfile
from scipy.io import wavfile
import noisereduce as nr

# Configurable parameters
wav_input = "../resources/wav/optimus.wav"
language = "pt" # en, pt

# Fixed parameters
wav_resampled = "../resources/wav/stt_resampled.wav"
wav_reduced_noise = "../resources/wav/stt_reduced_noise.wav"
whisper_main = "../whisper.cpp/main"

# Determines model based on language
if language == "en":
    model = "../resources/models/ggml-base.en.bin"
else:
    model = "../resources/models/ggml-base.bin"
print("[STT] Model: {}".format(model))

# Reads input wav
print("[STT] Reading input wav from {}".format(wav_input))
input_data, input_rate = librosa.load(wav_input, sr=16000)

# Converts audio to 16KHz sample rate (requirement of whisper.cpp)
print("[STT] Writing resampled wav to {}".format(wav_resampled))
soundfile.write(wav_resampled, input_data, 16000, 'PCM_16')

# Reads resampled wav
print("[STT] Reading resampled wav from {}".format(wav_resampled))
resampled_rate, resampled_data = wavfile.read(wav_resampled)

# Performs noise reduction
print("[STT] Performing noise reduction")
reduced_noise = nr.reduce_noise(y=resampled_data, sr=resampled_rate, stationary=False)

# Stores new wav
print("[STT] Writing reduced noise wav to {}".format(wav_reduced_noise))
wavfile.write(wav_reduced_noise, resampled_rate, reduced_noise)

# Runs whisper.cpp main for the reduced noise file
command = "{} -m {} -f {} -l {}".format(whisper_main, model, wav_reduced_noise, language)
print("[STT] System command: {}".format(command))
system(command)