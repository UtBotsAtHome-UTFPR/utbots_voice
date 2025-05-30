# from pickle import TRUE
from time import time,time_ns
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

import rclpy.time
from std_msgs.msg import Int16MultiArray
import io
import numpy as np
import torch
torch.set_num_threads(1)
import torchaudio
import matplotlib
import matplotlib.pylab as plt
import pyaudio
from pydub import AudioSegment
from scipy.signal import decimate
from rnnoise_wrapper import RNNoise
from datasets import load_dataset
from rcl_interfaces.msg import ParameterDescriptor
import ctypes
from queue import Queue
from threading import Lock

class AudioPublisher(Node):
# class AudioPublisher():

    def __init__(self):
        super().__init__('audio_publisher')
        self.publisher_ = self.create_publisher(Int16MultiArray, 'audio', 10) 
               
        self.declare_parameter('vad_timeout', 300 , ParameterDescriptor(description='vad_timeout in ms'))
        self.declare_parameter('vad_threshold', 0.5 , ParameterDescriptor(description='vad_threshold in float'))
        self.declare_parameter('vad_verbose', False , ParameterDescriptor(description='vad_verbose in Bool'))

        self.vad_thresh = self.get_parameter('vad_threshold').get_parameter_value().double_value
        
        self.timeout = 1_000_000 * self.get_parameter('vad_timeout').get_parameter_value().integer_value
        self.vebose = self.get_parameter('vad_verbose').get_parameter_value().bool_value

        self.i = 0
        self.model, self.utils = torch.hub.load(repo_or_dir='snakers4/silero-vad',
                              model='silero_vad',
                              force_reload=False)
        
        
        (self.get_speech_timestamps,
        self.save_audio,
        self.read_audio,
        self.VADIterator,
        self.collect_chunks) = self.utils

        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.SAMPLE_RATE = 48000
        self.CHUNK_SIZE = int(self.SAMPLE_RATE / 10)#whisper: 10 ms frame, 

        self.audio = pyaudio.PyAudio()

        # Audio processing queue
        self.audio_queue = Queue(maxsize=100)  # Adjust size as needed
        self.queue_lock = Lock()

        # self.num_samples = 512
        self.num_samples = 480*4
        #480*3/512=2,8 -> 3

        #480= 2**5 * 3 * 5  
        #512*3= 2**9 * 3
        self.denoiser = RNNoise('librnnoise_default.so.0.4.1')
        


        self.stream = self.audio.open(format=self.FORMAT,
                            channels=self.CHANNELS,
                            rate=self.SAMPLE_RATE,
                            input=True,
                            frames_per_buffer=self.CHUNK_SIZE,
                            stream_callback=self.audio_callback
                            )
        self.stream.start_stream()

        self.get_logger().info('Audio stream started.')

        self.t_last = time_ns()
        self.t0 = time_ns()

        self.is_speaking = False

        self.data = np.array([], dtype=np.int16)

        # self.i=0
        # self.test_whisper()
    
    def audio_callback(self, in_data, frame_count, time_info, status):
        # Convert bytes to message and publish

        t_now = time_ns()  # Nanosecond timestamp
        audio_int16 = np.frombuffer(in_data, dtype=np.int16)
    
        with self.queue_lock:
            self.audio_queue.put((t_now, audio_int16))
            # print("AAA")



        return (None, pyaudio.paContinue)


    def process_audio_from_queue(self):
        # Convert bytes to message and publish
        if self.audio_queue.empty():
            return
        timestamp, audio_data = self.audio_queue.get(timeout=0.1)   
        t_now = timestamp

        audio_int16 = audio_data
        speech_presence=self.evaluate_speech_presence(audio_int16)


        if(speech_presence == True):
            #concatenate
            self.data = np.concatenate([self.data, audio_int16])
            
            if(self.is_speaking == False):
                
                self.t_last=t_now

                self.is_speaking = True
                if(self.vebose):
                    self.get_logger().info(f"Runtime: {(t_now - self.t0)/(1_000_000)}")

        elif(self.is_speaking == True ) : #speech_presence == False
            if(self.vebose):
                self.get_logger().info(f"Speaker absence: {(t_now - self.t_last)/(1_000_000)}")
            if(t_now - self.t_last >= self.timeout ):

                self.is_speaking = False

                filtered=self.remove_noise(self.data)
                
                filtered_decimate=self.decimate_cast(filtered)

                if(self.vebose):
                    self.write2file(filtered_decimate)


                msg = Int16MultiArray()
                # print(f"Len: {len(filtered)} :{type(filtered)}: {type(filtered[0])}")

                msg.data = filtered_decimate.tolist()
                # if(self.TEST_WSP==True):
                #     msg.data = self.sample.tolist()


                self.publisher_.publish(msg)
                self.get_logger().info('Publishing')

                #limpa buffer   

                self.data = np.array([], dtype=np.int16)

        # return (None, pyaudio.paContinue)

    def evaluate_speech_presence(self, audio_int16):
        if(len(audio_int16)>=512*3):
            # print("greater")
            audio_float32 = self.int2float(audio_int16)
            # decimated_audio=audio_float32
            decimated_audio=decimate(x=audio_float32,q=3,zero_phase=True) 

            decimated_audio = decimated_audio[0:512].copy()

            v=self.vad_evaluate(decimated_audio)
            return v > self.vad_thresh
        return False 
    
    def decimate_cast(self, audio_int16_48khz):
        audio_float32 = self.int2float(audio_int16_48khz)
        # decimated_audio=audio_float32
        decimated_audio=decimate(x=audio_float32,q=3,zero_phase=True)
        return self.float2int(decimated_audio)

        
    

    def vad_evaluate(self, audio_float32):
        # audio_float32 = self.int2float(audio_int16)
        new_confidence = self.model(torch.from_numpy(np.ascontiguousarray(audio_float32)), 16000).item()
        if(self.vebose):
            self.get_logger().info(f"Confidence: {new_confidence*100:.2f}")
        return new_confidence
    
    # Taken from utils_vad.py
    def validate(model,
                inputs: torch.Tensor):
        with torch.no_grad():
            outs = model(inputs)
        return outs

    # Provided by Alexander Veysov
    def int2float(self,sound):
        abs_max = np.abs(sound).max()
        sound = sound.astype('float32')
        if abs_max > 0:
            sound = sound / 32768.0
        sound = sound.squeeze()  # depends on the use case
        return sound
    
    # def float2int(sound):
    #     abs_max = np.abs(sound).max()
    #     # sound = sound.astype('float32')
    #     sound *= 32768
    #     round(sound)
    #     sound = sound.astype('int16')
    #     sound = sound.squeeze()  # depends on the use case
    #     return sound
    
    def float2int(self,sound):
        """Convert float32 audio array (-1.0 to 1.0) to int16"""
        if not ((sound.dtype == np.float32) or (sound.dtype == np.float64)):
            raise ValueError("Input must be f32 or f64 array")
        
        # sound = np.clip(sound, -1.0, 1.0)  # Prevent overflow
        sound = np.round(sound * 32768.0)  # Scale then round
        return sound.astype(np.int16, copy=False).squeeze()

    def write2file (self,audio):
        import wave
        WAVE_OUTPUT_FILENAME = f"/home/ehg2004/utbots_ws/.tmp/voice{self.i}.wav"
        self.i=self.i+1
        wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
        wf.setnchannels(self.CHANNELS)
        wf.setsampwidth(self.audio.get_sample_size(self.FORMAT))
        wf.setframerate(self.SAMPLE_RATE)
        wf.writeframes(audio.tobytes())
        wf.close()
        self.get_logger().info(f"Wrote to: {WAVE_OUTPUT_FILENAME}")


    def test_whisper(self):
        if(isinstance(self.dataset,None)):
            self.dataset = load_dataset("distil-whisper/librispeech_long",
                            "clean",
                            split="validation",
                            # cache_dir="../.hf-cache/datasets/"
                            )
            self.sample = self.dataset[0]["audio"]["array"]
            self.TEST_WSP=True
        
    # def timer_callback(self):
    #     msg = String()
    #     msg.data = 'Hello World: %d' % self.i
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.data)
    #     self.i += 1

    # def remove_noise(self, audio_np):
    #     floated=self.int2float(audio_np)
    #     offset = 0
    #     frames = []
    #     frame_width=480

    #     while offset + frame_width <= len(audio_np):
    #         frames.append(audio_np[offset:offset + frame_width])
    #         offset += frame_width

    #     denoised_frames = [self.filter_frame(frame) for frame in frames][1]
    #     # denoised_frames = [frame_with_prob[1] for frame_with_prob in denoised_frames_with_probability]
    #     denoised = np.concatenate( denoised_frames, axis=0 )
    #     return(self.float2int(denoised))
    

    # def remove_noise(self, audio_np):
    #     """Process entire audio signal through RNNoise"""
    #     if len(audio_np) == 0:
    #         return audio_np
        
    #     # Convert to float32 if needed
    #     if audio_np.dtype != np.float32:
    #         # audio_np = self.int2float(audio_np.copy())
    #         audio_np = audio_np.copy()#.astype(ctypes.c_float)

        
    #     # Process in frames
    #     frame_size = 480
    #     frames = [audio_np[i:i+frame_size] 
    #             for i in range(0, len(audio_np), frame_size)]
        
    #     # Process each frame (keep last incomplete frame as-is)
    #     processed = []
    #     for frame in frames[:-1]:
    #         _, processed_frame = self.filter_frame(frame)
    #         processed.append(processed_frame.astype(np.int16))
        
    #     # Add last frame (might be shorter than 480)
    #     if len(frames[-1]) > 0:
    #         processed.append(frames[-1])
        
    #     return (np.concatenate(processed))

    def remove_noise(self, audio_np):
        """Process entire audio signal through RNNoise"""
        if len(audio_np) == 0:
            return audio_np
        
        # Convert to proper 16-bit PCM bytes
        if audio_np.dtype != np.int16:
            # audio_np = np.clip(audio_np * 32767, -32768, 32767).astype(np.int16)
            audio_np=self.float2int(audio_np)
        audio_bytes = audio_np.tobytes()

        # Process in frames (RNNoise expects 960 bytes = 480 samples @ 16-bit)
        frame_size = 960  # 480 samples * 2 bytes
        frames = [audio_bytes[i:i+frame_size] 
                for i in range(0, len(audio_bytes), frame_size)]
        
        # Process each frame
        # print("Processed:")
        processed = []
        for i,frame in enumerate(frames):
            if len(frame) < frame_size:
                frame += b'\x00' * (frame_size - len(frame))  # Pad if needed
            _, processed_frame = self.filter_frames(frame)
            if len(processed_frame) > 0:
                processed.append(processed_frame)
                # print(f"Len: {i} : {len(processed_frame)} : {type(processed_frame)}")

        
        # return np.frombuffer(b''.join(processed), dtype=np.int16)
        return np.concatenate(processed)
        
    # def filter_frame(self, frame):
        

    #     frame_buf = frame.astype(ctypes.c_float) #np.ndarray((480,), 'h', frame).astype(ctypes.c_float)
    #     frame_buf_ptr = frame_buf.ctypes.data_as(ctypes.POINTER(ctypes.c_float))

    #     vad_probability = self.rnnoise_lib.rnnoise_process_frame(self.rnnoise_obj, frame_buf_ptr, frame_buf_ptr)
    #     np_buf = np.frombuffer(frame_buf, "float32")
    #     return np_buf

    # def filter_frame(self, frame):
    #     """Process single frame with RNNoise.
    #     Returns tuple: (vad_probability, processed_frame)"""
    #     if len(frame) != 480:
    #         raise ValueError("RNNoise requires exactly 480 samples per frame")
        
    #     # Convert input to contiguous float32 buffer
    #     # frame_buf = np.ascontiguousarray(frame, dtype=np.float32).astype(ctypes.c_float)

    #     # frame_ptr = frame_buf.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
    #     frame_buf = np.ndarray((480,), 'h', frame).astype(ctypes.c_float)
    #     frame_ptr = frame_buf.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
    #     # Process in-place (RNNoise writes output to same buffer)
    #     vad_prob = self.denoiser.rnnoise_lib.rnnoise_process_frame(
    #         self.denoiser.rnnoise_obj, frame_ptr, frame_ptr)
        
    #     return vad_prob, (frame_buf.astype(ctypes.c_short)).copy()  # Return probability and processed frame


    def filter_frames(self, frame_bytes):
        """Wrapper for RNNoise's filter_frame"""
        # Let the RNNoise wrapper handle the conversion
        vad_prob, processed_bytes = self.denoiser.filter_frame(frame_bytes)
        return vad_prob, np.frombuffer(processed_bytes, dtype=np.int16)

def main(args=None):
    rclpy.init(args=args)
    node = AudioPublisher()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)  # Non-blocking
            node.process_audio_from_queue()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()