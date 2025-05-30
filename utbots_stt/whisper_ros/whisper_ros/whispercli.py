#!/usr/bin/env python3
"""
Real-time speech transcription using OpenAI Whisper large-v3-turbo via Hugging Face Transformers.
Reads audio from the default microphone, chunks it into configurable lengths, and transcribes on the fly.

Dependencies:
  pip install transformers sounddevice numpy torch

Usage:
  python realtime_whisper_transcription.py --chunk-length 5.0

Options:
  --chunk-length : length of audio chunk in seconds (float or list of floats)
  --sample-rate  : audio sampling rate (default: 16000)
"""
import argparse
import queue
import threading
import time
import numpy as np
import sounddevice as sd
from transformers import AutoProcessor, AutoModelForSpeechSeq2Seq, pipeline

def audio_producer(q: queue.Queue, sample_rate: int, chunk_length: float):
    """
    Records audio in chunks and pushes numpy arrays to the queue.
    """
    def callback(indata, frames, time_info, status):
        if status:
            print(f"Recording status: {status}")
        # convert to mono if stereo
        data = indata
        if data.ndim > 1:
            data = np.mean(data, axis=1)
        q.put(data.copy())

    with sd.InputStream(samplerate=sample_rate, channels=1, callback=callback,
                        blocksize=int(sample_rate * chunk_length)):
        while True:
            time.sleep(0.1)


def transcription_consumer(q: queue.Queue, n_models: int, chunk_length: float,
                           processor, model):
    """
    Consumes audio chunks, aggregates until chunk_length is reached, then transcribes.
    """
    pipe = pipeline(
        task="automatic-speech-recognition",
        model=model,
        tokenizer=processor,
        feature_extractor=processor,
        chunk_length_s=chunk_length,
        device=0 if torch.cuda.is_available() else -1,
        generate_kwargs={"max_new_tokens": 200}
    )

    buffer = np.array([], dtype=np.float32)
    while True:
        data = q.get()
        buffer = np.concatenate((buffer, data))
        # if buffer has enough samples
        if len(buffer) >= n_models:
            # take first chunk
            audio_chunk = buffer[:n_models]
            buffer = buffer[n_models:]
            # run ASR
            result = pipe(audio_chunk, chunk_length_s=chunk_length)
            print(result["text"], flush=True)


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--chunk-length", type=float, default=5.0,
                   help="Length of audio chunks in seconds.")
    p.add_argument("--sample-rate", type=int, default=16000,
                   help="Sampling rate for microphone.")
    return p.parse_args()

if __name__ == "__main__":
    args = parse_args()

    # Load processor and model
    print("Loading model and processor...")
    processor = AutoProcessor.from_pretrained("openai/whisper-large-v3-turbo")
    model = AutoModelForSpeechSeq2Seq.from_pretrained("openai/whisper-large-v3-turbo")

    # calculate samples per chunk
    samples_per_chunk = int(args.chunk_length * args.sample_rate)

    # create thread-safe queue
    audio_queue = queue.Queue()

    # start producer thread
    producer = threading.Thread(
        target=audio_producer,
        args=(audio_queue, args.sample_rate, args.chunk_length),
        daemon=True
    )
    producer.start()

    # start consumer (transcription)
    transcription_consumer(
        audio_queue,
        samples_per_chunk,
        args.chunk_length,
        processor,
        model
    )
