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
                
                # Mantém 'True' para áudios longos (como discutido)
                "return_timestamps": True,
                
                # --- CORRIGIDO PARA INGLÊS ---
                #"language": "english",          # Ativado para forçar Inglês
                "language": "portuguese",
                "task": "transcribe",           # Ativado
                
                # "no_repeat_ngram_size": 3 # Optional: avoid repeating phrases   
                }

                #generate_kwargs={
                #    "language": "english",
                #    "task": "transcribe",
                #}

class WhisperASR:
    def __init__(self, model="openai/whisper-tiny.en", verbose=False):
        self.verbose = verbose
        if(self.verbose):
            print("Loading model...")

        self.par = DEFAULT_PAR.copy()
        
        # Check if the model is local first
        local_path = self.get_local_model_path(model)
        
        if local_path and os.path.exists(local_path):
            if self.verbose:
                print(f"Loading model from local storage: {local_path}")
            model_path = local_path
            # Check if using "large-v3-turbo" and set torch_dtype
            if "large-v3-turbo" in model:
                if self.verbose:
                    print("Model is large-v3-turbo, setting torch_dtype to float16")
                torch_dtype = torch.float16
            else:
                torch_dtype = torch.float32  # Default dtype for other models

        else:
            if self.verbose:
                print(f"Model not found locally. Downloading and caching: {model}")
            
            # Check if using "large-v3-turbo" and set torch_dtype
            if "large-v3-turbo" in model:
                if self.verbose:
                    print("Model is large-v3-turbo, setting torch_dtype to float16")
                torch_dtype = torch.float16
            else:
                torch_dtype = torch.float32  # Default dtype for other models

            # Use snapshot_download to cache the model to the default transformers cache
            # This also avoids re-downloading if already in the transformers cache
            try:
                model_path = snapshot_download(repo_id=model, cache_dir=LOCAL_MODEL_DIR)
                if self.verbose:
                    print(f"Model downloaded to: {model_path}")
            except Exception as e:
                if self.verbose:
                    print(f"Failed to download model {model}. Error: {str(e)}")
                return

        # Set device
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        if self.verbose:
            print(f"Device set to use {self.device}")

        # Load model and processor
        try:
            model_obj = AutoModelForSpeechSeq2Seq.from_pretrained(
                model_path, torch_dtype=torch_dtype, low_cpu_mem_usage=True, use_safetensors=True
            )
            model_obj.to(self.device)

            processor = AutoProcessor.from_pretrained(model_path)

            self.pipe = pipeline(
                "automatic-speech-recognition",
                model=model_obj,
                tokenizer=processor.tokenizer,
                feature_extractor=processor.feature_extractor,
                max_new_tokens=128,
                chunk_length_s=CHUNK_LENGHT,
                batch_size=BATCH,
                return_timestamps=self.par.get("return_timestamps", False),
                torch_dtype=torch_dtype,
                device=self.device,
            )
            
            if self.verbose:
                if "large-v3-turbo" in model:
                    print(f"Model {model} successfully loaded from local storage with float16")
                else:
                    print(f"Model {model} successfully loaded from local storage")
                    
        except Exception as e:
            if self.verbose:
                print(f"Failed to load model {model}. Error: {str(e)}")

    def __del__(self):
        if(self.verbose):
            print("Deleting model...")
        gc.collect()
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        if(self.verbose):
            print("Model deleted.")

    def transcribe(self, audio_np):
        if(self.verbose):
            print("Transcribing...")
        #preprocess
        audio_np = audio_np.astype(np.float32) / 32768.0
        
        # --- (Esta correção de segurança que fizemos antes mantém-se) ---
        try:
            results = self.pipe(audio_np, generate_kwargs=self.par)
        except Exception as e:
            # Se a pipeline falhar (ex: erro de config), não trave
            print(f"Transcription error: {str(e)}")
            results = None # Retorne None em vez de travar

        # VERIFICAÇÃO DE SEGURANÇA ADICIONADA
        if not results:
            if(self.verbose):
                print("Transcription returned None or failed.")
            # Retorna um dicionário seguro para que o nó principal não trave
            return {"text": ""} 

        if(self.verbose):
            print("Transcription complete.")
        return results
        
    def get_local_model_path(self, model_id):
        """Get the local path for a model"""
        return os.path.join(LOCAL_MODEL_DIR, model_id.replace("/", "_"))

    def download_model(self, model_id):
        """Download and store a model locally"""
        local_path = self.get_local_model_path(model_id)
        if not os.path.exists(local_path):
            try:
                print(f"Downloading model {model_id} to {local_path}...")
                snapshot_download(repo_id=model_id, local_dir=local_path, local_dir_use_symlinks=False)
                print(f"Model {model_id} downloaded successfully.")
                return True
            except Exception as e:
                print(f"Error downloading model {model_id}: {str(e)}")
                return False
        else:
            print(f"Model {model_id} already exists locally.")
            return True

    def remove_local_model(self, model_id):
        """Remove a locally stored model"""
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