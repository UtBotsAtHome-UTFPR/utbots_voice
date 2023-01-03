// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>

// AUDIO PROCESSING
#include "miniaudio.h"

// VAD
#include "vad_iterator.hpp"

// ROS PUBLISHER
ros::Publisher pub_audio;
ros::Publisher pub_emotion;

// ADJUSTABLE PARAMETERS
int ms_to_process_vad   = 300;  // Defines how often will run VAD for buffer_vad
int ms_word_interval    = 1500; // Defines maximum time between pauses can be a word interval
int ms_buffer_vad_aux   = 600;  // Defines how much time auxiliary buffer will have (bigger than ms_to_process_vad)

// AUDIO BUFFERS
std::vector<int16_t>    buffer_vad;         // Contains continuously captured audio, is cleared often
std::vector<int16_t>    buffer_vad_aux;     // Contains buffer_vad but with extra samples
std::vector<int16_t>    buffer_voice_only;  // Contains audio with voice

// TO CONTROL WHEN SPEECH HAPPENED/IS HAPPENING
#define STATUS_WAITING_FOR_SPEECH  0
#define STATUS_SPEECH_IS_HAPPENING 1
int     speech_status       = STATUS_WAITING_FOR_SPEECH;
double  t_last_speech       = 0;                            // Saves when last speech ended

// AUDIO SETTINGS (FIXED)
const int sample_rate   = 16000;
const int channels      = 1;
const int bit_depth     = 16;

// AUXILIARY SAMPLES
const int aux_samples                   = ms_buffer_vad_aux * sample_rate / 1000;

// VAD SETTINGS
std::string vad_model_path              = ros::package::getPath("whisper_cpp_ros") + "/models/silero_vad.onnx";
const int   vad_sample_rate             = sample_rate;
const int   vad_small_frame_ms          = ms_to_process_vad;
const int   vad_small_window_samples    = vad_small_frame_ms * vad_sample_rate / 1000;
const float vad_threshold               = 0.5f;

// VAD
VadIterator vad_small(vad_model_path, vad_sample_rate, vad_small_frame_ms, vad_threshold);

// MINIAUDIO
ma_result           audio_result;
ma_encoder_config   audio_encoder_config;
ma_encoder          audio_encoder;
ma_device_config    audio_device_config;
ma_device           audio_device;

// FUNCTION DECLARATIONS
void        CallbackAudio(ma_device* pDevice, void* pOutput, const void* pInput, ma_uint32 frame_count);
const void  ResizeAuxBuffer();
const void  UpdateVoiceBuffer();
const void  UpdateVoiceActivityStatus();
const bool  EvaluateSpeechPresence();
const bool  RunFailsafeVAD();
const void  InitAudioCapture();
const void  PublishBuffer(std::vector<int16_t>* buffer);
const void  JoinFrames(std::vector<int16_t>* frames_input, std::vector<int16_t>* frames_output);
const void  ClearFrames(std::vector<int16_t>* frames);
const void  WriteFramesToFile(std::vector<int16_t>* frames);

int main(int argc, char **argv)
{
    // ROS
    ros::init(argc, argv, "vad_node");
    ros::NodeHandle nh;
    pub_audio = nh.advertise<std_msgs::Int16MultiArray>("audio/voice", 100);
    pub_emotion = nh.advertise<std_msgs::String>("emotion", 100);

    // GET PARAMS
    nh.getParam("/voice/vad_node/ms_to_process_vad", ms_to_process_vad);
    nh.getParam("/voice/vad_node/ms_word_interval", ms_word_interval);
    nh.getParam("/voice/vad_node/ms_buffer_vad_aux", ms_buffer_vad_aux);
    ROS_INFO("[VAD] ms_to_process_vad = %d", ms_to_process_vad);
    ROS_INFO("[VAD] ms_word_interval = %d", ms_word_interval);
    ROS_INFO("[VAD] ms_buffer_vad_aux = %d\n", ms_buffer_vad_aux);

    // START AUDIO CAPTURE
    InitAudioCapture();

    // LOOPRATE
    ros::Rate loopRate(30);

    ROS_INFO("[VAD] Looping\n");
    while (ros::ok())
    {
        loopRate.sleep();
        ros::spinOnce();
    }

    ma_device_uninit(&audio_device);
    ma_encoder_uninit(&audio_encoder);
    return 0;
}

void CallbackAudio(ma_device* pDevice, void* pOutput, const void* pInput, ma_uint32 frame_count)
{
    // Store frames as vector and append to buffer
    std::vector<int16_t> current_frames(frame_count);
    std::memcpy(current_frames.data(), pInput, frame_count * ma_get_bytes_per_frame(pDevice->capture.format, channels));
    
    JoinFrames(&current_frames, &buffer_vad);

    if (speech_status != STATUS_SPEECH_IS_HAPPENING) {
        JoinFrames(&current_frames, &buffer_vad_aux);
        if (buffer_vad_aux.size() >= aux_samples)
            ResizeAuxBuffer();
    }

    if (buffer_vad.size() >= vad_small_window_samples) {
        UpdateVoiceBuffer();
        UpdateVoiceActivityStatus();
        ClearFrames(&buffer_vad);
    }
}

const void ResizeAuxBuffer()
{
    std::vector<int16_t> tmp(buffer_vad_aux.end() - aux_samples, buffer_vad_aux.end());
    buffer_vad_aux = tmp;
}

const void UpdateVoiceBuffer()
{
    std_msgs::String msg_emotion;

    if (speech_status == STATUS_SPEECH_IS_HAPPENING) {
        JoinFrames(&buffer_vad, &buffer_voice_only);
        msg_emotion.data = "vigilance";
    }
    else {
        msg_emotion.data = "acceptance";
    }

    pub_emotion.publish(msg_emotion);
}

const void UpdateVoiceActivityStatus()
{
    bool voice_activity = EvaluateSpeechPresence();
    
    if (voice_activity) {
        ROS_INFO("[VAD] Voice activity");
        speech_status = STATUS_SPEECH_IS_HAPPENING;
        t_last_speech = ros::Time::now().toSec();
    }
    else {
        if (speech_status == STATUS_SPEECH_IS_HAPPENING) {
            float dt_ms = 1000 * (ros::Time::now().toSec() - t_last_speech);
            ROS_INFO("[VAD] Interval (%f ms)...", dt_ms);
            if (dt_ms  > ms_word_interval) {
                ROS_INFO("[VAD] Sentence end");
                speech_status = STATUS_WAITING_FOR_SPEECH;
                if (RunFailsafeVAD()) {
                    JoinFrames(&buffer_voice_only, &buffer_vad_aux);
                    WriteFramesToFile(&buffer_vad_aux);
                    PublishBuffer(&buffer_vad_aux);
                    ClearFrames(&buffer_vad_aux);
                }
                ClearFrames(&buffer_voice_only);
            }
        }
    }
}

const bool EvaluateSpeechPresence()
{     
    const int num_samples = buffer_vad.size();

    // Converts audio to float32 PCM
    std::vector<float> pcm_f32(num_samples);
    for (int i = 0; i < num_samples; i++)
        pcm_f32[i] = static_cast<float>(buffer_vad[i]) / 32768.0;

    // Runs model
    vad_small.reset_states();
    vad_small.predict(pcm_f32);

    // Returns model output
    if (vad_small.get_output() >= vad_threshold)
        return true;
    else
        return false;
}

const bool RunFailsafeVAD()
{
    const int num_samples = buffer_voice_only.size();
    ROS_INFO("[VAD] Running failsafe VAD for %d samples", num_samples);

    // Converts audio to float32 PCM
    std::vector<float> pcm_f32(num_samples);
    for (int i = 0; i < num_samples; i++)
        pcm_f32[i] = static_cast<float>(buffer_voice_only[i]) / 32768.0;

    const int test_frame_ms = 64;
    const int test_samples  = test_frame_ms * sample_rate / 1000;
    VadIterator vad_failsafe(
        vad_model_path, vad_sample_rate, test_frame_ms, vad_threshold);

    std::vector<float> inferences;
    for (int j = 0; j < num_samples; j += test_samples)
    {
        std::vector<float> r{&pcm_f32[0] + j, &pcm_f32[0] + j + test_samples};
        vad_failsafe.predict(r);
        inferences.push_back(vad_failsafe.get_output());
    }

    // Takes average inference value
    float inference_avg = 0;
    for (float inf : inferences)
        inference_avg = inference_avg + inf;
    inference_avg = inference_avg / inferences.size();
    
    if (inference_avg >= 0.3)
        return true;
    else
        return false;
}

const void InitAudioCapture()
{
    ROS_INFO("[VAD] sample_rate = %d, channels = %d, bit_depth = %d",
        sample_rate, channels, bit_depth);

    // Encoder config init
    audio_encoder_config = ma_encoder_config_init(
        ma_encoding_format_wav, 
        ma_format_s16, 
        channels, 
        sample_rate);
    
    // Encoder file init
    if (ma_encoder_init_file(
        "/tmp/test.wav", &audio_encoder_config, &audio_encoder) != MA_SUCCESS)
        ROS_INFO("[VAD] Failed to init file");

    // Miniaudio device
    audio_device_config                    = ma_device_config_init(ma_device_type_capture);
    audio_device_config.capture.format     = audio_encoder.config.format;
    audio_device_config.capture.channels   = audio_encoder.config.channels;
    audio_device_config.sampleRate         = audio_encoder.config.sampleRate;
    audio_device_config.dataCallback       = CallbackAudio;
    audio_device_config.pUserData          = &audio_encoder;
    
    // Device init
    if (ma_device_init(NULL, &audio_device_config, &audio_device)  != MA_SUCCESS)
        ROS_INFO("[VAD] Failed to init capture device");
    if (ma_device_start(&audio_device) != MA_SUCCESS)
        ROS_INFO("[VAD] Failed to start capture device");
}

const void JoinFrames(std::vector<int16_t>* frames_input, std::vector<int16_t>* frames_output)
{
    frames_output->reserve(frames_input->size() + frames_output->size());                       // Preallocate memory
    frames_output->insert(frames_output->end(), frames_input->begin(),   frames_input->end());  // Inserts one into the other
}

const void ClearFrames(std::vector<int16_t>* frames)
{
    frames->clear();
}

const void WriteFramesToFile(std::vector<int16_t>* frames)
{
    ma_encoder_write_pcm_frames(&audio_encoder, frames->data(), frames->size(), NULL);
}

const void PublishBuffer(std::vector<int16_t>* buffer)
{
    ROS_INFO("[VAD] Publishing buffer with size %d\n", (int)buffer->size());
    std_msgs::Int16MultiArray msg;
    msg.data = *buffer;
    pub_audio.publish(msg);
}