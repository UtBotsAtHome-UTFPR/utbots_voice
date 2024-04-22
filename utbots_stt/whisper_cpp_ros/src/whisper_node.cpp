// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>

// Whisper
#include "whisper.h"

// C++
#include <thread>

typedef struct parameters {
    // Whisper parameters
    int32_t n_threads       = std::min(4, (int32_t) std::thread::hardware_concurrency());
    int32_t n_processors    = 1;
    int32_t offset_t_ms     = 0;
    int32_t duration_ms     = 0;
    int32_t max_context     = -1;
    int32_t max_len         = 60;
    float word_thold        = 0.01f;
    bool speed_up           = true;
    bool translate          = false;
    bool print_special      = false;
    bool print_progress     = false;
    bool no_timestamps      = false;
    std::string language    = "en";
    std::string model       = ros::package::getPath("whisper_cpp_ros") + "/models/ggml-tiny.en.bin";

    // Other parameters
    bool print_timings = false;
    bool show_result = true;
} parameters;

class WhisperNode
{
    // Whisper params
    parameters params;

    // Whisper context
    struct whisper_context* wsp_context;

    // ROS
    ros::NodeHandle nh;
    ros::Subscriber sub_audio_data = nh.subscribe("/utbots/voice/stt/voice_signal", 10, &WhisperNode::CallbackAudioData, this);
    ros::Publisher  pub_output_text = nh.advertise<std_msgs::String>("/utbots/voice/stt/whispered", 1);

    public:
        // Constructor
        WhisperNode()
        {
            InitParams();
            InitWhisper();
        }

        // Destructor
        ~WhisperNode()
        {
            whisper_free(wsp_context);
        }

        void InitParams()
        {
            ROS_INFO("[WHISPER] Initializing parameters");
            nh.getParam("/utbots/voice/stt/whisper_node/offset_t_ms",     params.offset_t_ms);
            nh.getParam("/utbots/voice/stt/whisper_node/duration_ms",     params.duration_ms);
            nh.getParam("/utbots/voice/stt/whisper_node/max_context",     params.max_context);
            nh.getParam("/utbots/voice/stt/whisper_node/max_len",         params.max_len);
            nh.getParam("/utbots/voice/stt/whisper_node/word_thold",      params.word_thold);
            nh.getParam("/utbots/voice/stt/whisper_node/speed_up",        params.speed_up);
            nh.getParam("/utbots/voice/stt/whisper_node/translate",       params.translate);
            nh.getParam("/utbots/voice/stt/whisper_node/print_special",   params.print_special);
            nh.getParam("/utbots/voice/stt/whisper_node/print_progress",  params.print_progress);
            nh.getParam("/utbots/voice/stt/whisper_node/no_timestamps",   params.no_timestamps);
            nh.getParam("/utbots/voice/stt/whisper_node/language",        params.language);
            nh.getParam("/utbots/voice/stt/whisper_node/model",           params.model);
            nh.getParam("/utbots/voice/stt/whisper_node/print_timings",   params.print_timings);
            nh.getParam("/utbots/voice/stt/whisper_node/show_result",     params.show_result);

            ROS_INFO("[WHISPER] PARAMETERS");
            ROS_INFO("[WHISPER]   - offset_t_ms: %d",       params.offset_t_ms);
            ROS_INFO("[WHISPER]   - duration_ms: %d",       params.duration_ms);
            ROS_INFO("[WHISPER]   - max_context: %d",       params.max_context);
            ROS_INFO("[WHISPER]   - max_len: %d",           params.max_len);
            ROS_INFO("[WHISPER]   - word_thold: %f",        params.word_thold);
            ROS_INFO("[WHISPER]   - speed_up: %d",          params.speed_up);
            ROS_INFO("[WHISPER]   - translate: %d",         params.translate);
            ROS_INFO("[WHISPER]   - print_special: %d",     params.print_special);
            ROS_INFO("[WHISPER]   - print_progress: %d",    params.print_progress);
            ROS_INFO("[WHISPER]   - no_timestamps: %d",     params.no_timestamps);
            ROS_INFO("[WHISPER]   - language: %s",          params.language.c_str());
            ROS_INFO("[WHISPER]   - model: %s",             params.model.c_str());
            ROS_INFO("[WHISPER]   - show_result: %d\n",     params.show_result);
        }

        void InitWhisper()
        {
            ROS_INFO("[WHISPER] system_info: n_threads = %d / %d | %s", params.n_threads * params.n_processors, std::thread::hardware_concurrency(), whisper_print_system_info());
            ROS_INFO("[WHISPER] Initializing whisper");
            wsp_context = whisper_init(params.model.c_str());
            if (wsp_context == nullptr)
                ROS_INFO("[WHISPER] Error: failed to initialize whisper context");
        }

        // Callback function for audio data
        void CallbackAudioData(const std_msgs::Int16MultiArray::ConstPtr &msg)
        {
            ROS_INFO("[WHISPER] Callback: AudioData (vector with size %d)", int(msg->data.size()));
            std::vector<int16_t> input_frames   = msg->data;
            double t_start                      = ros::Time::now().toSec();
            std::vector<float>                  pcm_float_32_mono;
            ConvertMsgToPcmFloat                (&input_frames, &pcm_float_32_mono);
            Inference                           (&pcm_float_32_mono);
            double t_final                      = ros::Time::now().toSec();
            double dt                           = t_final - t_start;
            ROS_INFO                            ("[WHISPER] Elapsed time: %f s\n", dt);
        }

        void ConvertMsgToPcmFloat(
            std::vector<int16_t>*               input_pcm16_mono,
            std::vector<float>*                 output_pcmf32_mono)
        {
            const int channels = 1, sample_rate = 16000, bit_depth = 16;
            ROS_INFO("[WHISPER] channels = %d, sample_rate = %d, bit_depth = %d",
                channels, sample_rate, bit_depth);

            const int num_frames    = input_pcm16_mono->size();
            const int num_samples   = num_frames / channels;

            output_pcmf32_mono->clear();
            output_pcmf32_mono->resize(num_samples);
            for (int i = 0; i < num_samples; i++)
                (*output_pcmf32_mono)[i] = static_cast<float>(input_pcm16_mono->at(i)) / 32768;

            ROS_INFO("[WHISPER] Processing %d samples, %.1f sec, %d threads, %d processors, lang = %s, task = %s",
                int(output_pcmf32_mono->size()), float(output_pcmf32_mono->size())/WHISPER_SAMPLE_RATE, 
                params.n_threads, params.n_processors, params.language.c_str(), 
                params.translate ? "translate" : "transcribe");
        }

        void SetWhisperFullParams(whisper_full_params* wparams)
        {
            wparams->print_realtime   = false;
            wparams->print_progress   = params.print_progress;
            wparams->print_timestamps = !params.no_timestamps;
            wparams->print_special    = params.print_special;
            wparams->translate        = params.translate;
            wparams->language         = params.language.c_str();
            wparams->n_threads        = params.n_threads;
            wparams->n_max_text_ctx   = params.max_context >= 0 ? params.max_context : wparams->n_max_text_ctx;
            wparams->offset_ms        = params.offset_t_ms;
            wparams->duration_ms      = params.duration_ms;

            wparams->token_timestamps = false;
            wparams->thold_pt         = params.word_thold;
            wparams->max_len          = params.max_len;

            wparams->speed_up         = params.speed_up;

            wparams->prompt_tokens    = nullptr;
            wparams->prompt_n_tokens  = 0;
        }

        void ShowResult()
        {
            const int n_segments = whisper_full_n_segments(wsp_context);
            std::string text("");
            for (int i = 0; i < n_segments; ++i)
                text = text + whisper_full_get_segment_text(wsp_context, i);

            // Publishes
            std_msgs::String msg;
            msg.data = text.c_str();
            pub_output_text.publish(msg);
            
            ROS_INFO("[WHISPER] Result: %s", text.c_str());
        }

        void Inference(std::vector<float>* pcmf32)
        {
            ROS_INFO("[WHISPER] Setting full parameters");
            whisper_full_params wparams = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);
            SetWhisperFullParams(&wparams);

            ROS_INFO("[WHISPER] Inferencing");

            if (whisper_full_parallel(wsp_context, wparams, pcmf32->data(), pcmf32->size(), params.n_processors) != 0)
                ROS_INFO("[WHISPER] Failed to process audio");
            else 
            {
                ROS_INFO("[WHISPER] Inference done");
                if (params.show_result)
                    ShowResult();
                if (params.print_timings) 
                    whisper_print_timings(wsp_context);
            }
        }
};

int main(int argc, char **argv)
{
    // ROS
    ros::init(argc, argv, "whisper_node");
    WhisperNode* whisper_node = new WhisperNode();
    ros::Rate loopRate(30);

    ROS_INFO("[WHISPER] Looping\n");
    while (ros::ok())
    {
        loopRate.sleep();
        ros::spinOnce();
    }

    delete(whisper_node);
    return 0;
}