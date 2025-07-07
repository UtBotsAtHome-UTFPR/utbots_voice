import torch
from transformers import AutoModelForSpeechSeq2Seq, AutoProcessor, pipeline
from datasets import load_dataset
import gc
import numpy as np
TINY="openai/whisper-tiny.en"
LV3_t="openai/whisper-large-v3-turbo"
CHUNK_LENGHT=15
BATCH=8
TIMESTAMPS=False
SAMPLE_R=16_000

DEFAULT_PAR = {
                # "max_new_tokens": 448,
                # "num_beams": 1,
                "condition_on_prev_tokens": False,
                # "compression_ratio_threshold": 2.4,
                # "compression_ratio_threshold": 1.35,  # zlib compression ratio threshold (in token space)
                # "temperature": (0.0, 0.2, 0.4, 0.6, 0.8, 1.0),
                "temperature": 0.0,
                # "logprob_threshold": -1.0,
                # "no_speech_threshold": 0.8,
                "return_timestamps": False,
                "language": "english",          # Force English language
                "task": "transcribe",      # or "transcribe"
                # "no_repeat_ngram_size": 3 # Optional: avoid repeating phrases   
                }

    #             generate_kwargs={
    #                 "language": "english",          # Force English language
    #                 "task": "transcribe",      # or "translate"
    #                 "beam_size": 5,            # Optional: beam search
    #                 "temperature": 0.0,        # Optional: decoding temperature
    #                 "no_repeat_ngram_size": 3 # Optional: avoid repeating phrases

class WhisperASR:
    def __init__(self,model=LV3_t,parameters=DEFAULT_PAR,load_def=True,verbose=False):
        self.model_name = model
        self.model = None
        self.processor = None
        self.pipe = None
        self.verbose=verbose
        self.generate_kwargs=None
        self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
        self.torch_dtype = torch.float16 if torch.cuda.is_available() else torch.float32
        if(load_def==True):
            self.load_model(model,parameters)

    def load_model(self, model_id=LV3_t,parameters=DEFAULT_PAR):
        """Load model into memory"""
        try:
            self.model = AutoModelForSpeechSeq2Seq.from_pretrained(
                model_id,
                torch_dtype=self.torch_dtype,
                low_cpu_mem_usage=True,
                use_safetensors=True,
                # attn_implementation="flash_attention_2"
            ).to(self.device)
            
            self.processor = AutoProcessor.from_pretrained(model_id)
            self.pipe = pipeline(
                "automatic-speech-recognition",
                model=self.model,
                tokenizer=self.processor.tokenizer,
                feature_extractor=self.processor.feature_extractor,
                device=self.device,
                torch_dtype=self.torch_dtype,
            )

            self.generate_kwargs=parameters
            
            print("Model successfully loaded")
        except Exception as e:
            print(f"Error loading model: {str(e)}")
            self.unload_model()
    
    def unload_model(self):
        """Completely unload model from memory"""
        try:
            # Delete pipeline first
            if self.pipe is not None:
                del self.pipe
                self.pipe = None
            
            # Delete model and processor
            if self.model is not None:
                del self.model
                self.model = None
                
            if self.processor is not None:
                del self.processor
                self.processor = None
            
            # Clear GPU cache if available
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
            self.generate_kwargs=DEFAULT_PAR
            # Run garbage collection
            gc.collect()
            print("Model successfully unloaded")
        except Exception as e:
            print(f"Error unloading model: {str(e)}")
    
    def transcribe(self, audio):
        """Run transcription if model is loaded"""
        if self.pipe is None:
            print("Error: Model not loaded")
            return None
        
        try:
            audio_padded = self.audio_padding(audio=audio)

            result=self.pipe(
                            audio_padded,
                            # return_timestamps=TIMESTAMPS,  # Enable timestamps if needed
                            # batch_size=BATCH,  # Adjust based on your memory
                            # chunk_length_s=CHUNK_LENGHT
                            # generate_kwargs = {
                            #     # "max_new_tokens": 448,
                            #     "num_beams": 1,
                            #     "condition_on_prev_tokens": False,
                            #     "compression_ratio_threshold": 1.35,  # zlib compression ratio threshold (in token space)
                            #     "temperature": (0.0),#(0.0, 0.2, 0.4, 0.6, 0.8, 1.0),
                            #     "logprob_threshold": -1.0,
                            #     "no_speech_threshold": 0.6,
                            #     "return_timestamps": False,
                            #     "language": "english",          # Force English language
                            #     "task": "translate",      # or "transcribe"
                            #     # "beam_size": 5,            # Optional: beam search
                            #     "temperature": 0.0,        # Optional: decoding temperature
                            #     "no_repeat_ngram_size": 3 # Optional: avoid repeating phrases
                            #     },
                            generate_kwargs = self.generate_kwargs

                            )
            print(result["text"])
            return result
        except Exception as e:
            print(f"Transcription error: {str(e)}")
            return None

    def audio_padding(self,audio):
        zero_padding = 30
        #SAMPLE_R : 16kHz
        if audio.shape[-1] < SAMPLE_R * zero_padding:
            pad_length = SAMPLE_R * zero_padding - audio.shape[-1]
            if(self.verbose):
                print("audio padded!")
            return(np.pad(audio, (0, pad_length), mode="constant"))
        return audio