import torch
from transformers import AutoModelForSpeechSeq2Seq, AutoProcessor, pipeline
from datasets import load_dataset
import gc
import numpy as np
import os
from huggingface_hub import snapshot_download, login
from pathlib import Path
TINY="openai/whisper-tiny.en"
LV3_t="openai/whisper-large-v3-turbo"
CHUNK_LENGHT=15
BATCH=8
TIMESTAMPS=False
SAMPLE_R=16_000

# Local model storage directory
LOCAL_MODEL_DIR = os.path.expanduser("~/whisper_models")

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
        self.local_model_dir = LOCAL_MODEL_DIR
        
        # Create local model directory if it doesn't exist
        os.makedirs(self.local_model_dir, exist_ok=True)
        
        if(load_def==True):
            self.load_model(model,parameters)

    def download_model_locally(self, model_id, force_download=False):
        """Download model to local directory using huggingface_hub"""
        try:
            # Create model-specific directory
            model_local_path = os.path.join(self.local_model_dir, model_id.replace("/", "_"))
            
            # Check if model already exists locally
            if os.path.exists(model_local_path) and not force_download:
                if self.verbose:
                    print(f"Model {model_id} already exists locally at {model_local_path}")
                return model_local_path
            
            print(f"Downloading model {model_id} to {model_local_path}...")
            
            # Download the entire model repository
            snapshot_download(
                repo_id=model_id,
                local_dir=model_local_path,
                local_dir_use_symlinks=False,
                resume_download=True
            )
            
            print(f"Model {model_id} successfully downloaded to {model_local_path}")
            return model_local_path
            
        except Exception as e:
            print(f"Error downloading model {model_id}: {str(e)}")
            return None

    def get_local_model_path(self, model_id):
        """Get the local path for a model"""
        model_local_path = os.path.join(self.local_model_dir, model_id.replace("/", "_"))
        return model_local_path if os.path.exists(model_local_path) else None

    def load_model(self, model_id=LV3_t,parameters=DEFAULT_PAR):
        """Load model into memory from local directory"""
        try:
            # First check if model exists locally
            local_path = self.get_local_model_path(model_id)
            
            if local_path is None:
                # Model doesn't exist locally, download it
                if self.verbose:
                    print(f"Model {model_id} not found locally, downloading...")
                local_path = self.download_model_locally(model_id)
                
                if local_path is None:
                    raise Exception(f"Failed to download model {model_id}")
            else:
                if self.verbose:
                    print(f"Using local model at {local_path}")
            
            # Load model from local path
            self.model = AutoModelForSpeechSeq2Seq.from_pretrained(
                local_path,
                torch_dtype=self.torch_dtype,
                low_cpu_mem_usage=True,
                use_safetensors=True,
                local_files_only=True  # Force using local files only
                # attn_implementation="flash_attention_2"
            ).to(self.device)
            
            self.processor = AutoProcessor.from_pretrained(
                local_path,
                local_files_only=True  # Force using local files only
            )
            
            self.pipe = pipeline(
                "automatic-speech-recognition",
                model=self.model,
                tokenizer=self.processor.tokenizer,
                feature_extractor=self.processor.feature_extractor,
                device=self.device,
                torch_dtype=self.torch_dtype,
            )

            self.generate_kwargs=parameters
            
            print(f"Model {model_id} successfully loaded from local storage")
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
            result=self.pipe(audio_padded,
                            generate_kwargs = self.generate_kwargs)
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
    
    def list_local_models(self):
        """List all locally downloaded models"""
        if not os.path.exists(self.local_model_dir):
            return []
        
        local_models = []
        for item in os.listdir(self.local_model_dir):
            item_path = os.path.join(self.local_model_dir, item)
            if os.path.isdir(item_path):
                # Convert back to original model name format
                model_name = item.replace("_", "/")
                local_models.append(model_name)
        return local_models

    def remove_local_model(self, model_id):
        """Remove a locally downloaded model"""
        import shutil
        
        local_path = self.get_local_model_path(model_id)
        if local_path and os.path.exists(local_path):
            try:
                shutil.rmtree(local_path)
                print(f"Successfully removed local model: {model_id}")
                return True
            except Exception as e:
                print(f"Error removing local model {model_id}: {str(e)}")
                return False
        else:
            print(f"Local model {model_id} not found")
            return False

    def get_local_model_info(self, model_id):
        """Get information about a locally stored model"""
        local_path = self.get_local_model_path(model_id)
        if local_path and os.path.exists(local_path):
            try:
                # Get directory size
                total_size = sum(os.path.getsize(os.path.join(dirpath, filename))
                               for dirpath, dirnames, filenames in os.walk(local_path)
                               for filename in filenames)
                
                # Convert to MB
                size_mb = total_size / (1024 * 1024)
                
                return {
                    "model_id": model_id,
                    "local_path": local_path,
                    "size_mb": round(size_mb, 2),
                    "exists": True
                }
            except Exception as e:
                print(f"Error getting model info: {str(e)}")
                return None
        else:
            return {
                "model_id": model_id,
                "local_path": None,
                "size_mb": 0,
                "exists": False
            }