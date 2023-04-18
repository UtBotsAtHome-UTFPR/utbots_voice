#ifndef GUARD_VAD_ITERATOR
#define GUARD_VAD_ITERATOR

#include <onnxruntime_cxx_api.h>

class VadIterator
{
    // OnnxRuntime resources
    Ort::Env env;
    Ort::SessionOptions session_options;
    std::shared_ptr<Ort::Session> session = nullptr;
    Ort::AllocatorWithDefaultOptions allocator;
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeCPU);

public:
    // Construction
    VadIterator(
            const std::string ModelPath, 
            int Sample_rate, 
            int frame_size, 
            float Threshold) 
    {
        init_onnx_model(ModelPath);
        sample_rate = Sample_rate;
        sr_per_ms = sample_rate / 1000;
        threshold = Threshold;
        window_size_samples = frame_size * sr_per_ms;
        
        input.resize(window_size_samples);
        input_node_dims[0] = 1;
        input_node_dims[1] = window_size_samples;
        // std::cout << "== Input size " << input.size() << std::endl;
        _h.resize(size_hc);
        _c.resize(size_hc);
        sr.resize(1);
        sr[0] = sample_rate;
    }

    void init_engine_threads(int inter_threads, int intra_threads)
    {   
        // The method should be called in each thread/proc in multi-thread/proc work
        session_options.SetIntraOpNumThreads(intra_threads);
        session_options.SetInterOpNumThreads(inter_threads);
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    }

    void init_onnx_model(const std::string &model_path)
    {   
        // Init threads = 1 for 
        init_engine_threads(1, 1);
        // Load model
        session = std::make_shared<Ort::Session>(env, model_path.c_str(), session_options);
    }

    void reset_states()
    {
        // Call reset before each audio start
        std::memset(_h.data(), 0.0f, _h.size() * sizeof(float));
        std::memset(_c.data(), 0.0f, _c.size() * sizeof(float));
        triggerd = false;
        temp_end = 0;
        current_sample = 0;
    }

    // Call it in predict func. if you prefer raw bytes input.
    void bytes_to_float_tensor(const char *pcm_bytes) 
    {
        std::memcpy(input.data(), pcm_bytes, window_size_samples * sizeof(int16_t));
        for (int i = 0; i < window_size_samples; i++)
        {
            input[i] = static_cast<float>(input[i]) / 32768; // int16_t normalized to float
        }
    }


    void predict(const std::vector<float> &data)
    {
        // Infer
        // Create ort tensors
        input.assign(data.begin(), data.end());
        Ort::Value input_ort = Ort::Value::CreateTensor<float>(
            memory_info, input.data(), input.size(), input_node_dims, 2);
        Ort::Value sr_ort = Ort::Value::CreateTensor<int64_t>(
            memory_info, sr.data(), sr.size(), sr_node_dims, 1);
        Ort::Value h_ort = Ort::Value::CreateTensor<float>(
            memory_info, _h.data(), _h.size(), hc_node_dims, 3);
        Ort::Value c_ort = Ort::Value::CreateTensor<float>(
            memory_info, _c.data(), _c.size(), hc_node_dims, 3);

        // Clear and add inputs
        ort_inputs.clear();
        ort_inputs.emplace_back(std::move(input_ort));
        ort_inputs.emplace_back(std::move(sr_ort));
        ort_inputs.emplace_back(std::move(h_ort));
        ort_inputs.emplace_back(std::move(c_ort));

        // Infer
        ort_outputs = session->Run(
            Ort::RunOptions{nullptr},
            input_node_names.data(), ort_inputs.data(), ort_inputs.size(),
            output_node_names.data(), output_node_names.size());

        // Output probability & update h,c recursively
        output = ort_outputs[0].GetTensorMutableData<float>()[0];
        float *hn = ort_outputs[1].GetTensorMutableData<float>();
        std::memcpy(_h.data(), hn, size_hc * sizeof(float));
        float *cn = ort_outputs[2].GetTensorMutableData<float>();
        std::memcpy(_c.data(), cn, size_hc * sizeof(float));

        // Push forward sample index
        current_sample += window_size_samples;
    }

    float get_output()
    {
        return output;
    }

private:
    // model config
    int64_t window_size_samples;  // Assign when init, support 256 512 768 for 8k; 512 1024 1536 for 16k.
    int sample_rate;
    int sr_per_ms;  // Assign when init, support 8 or 16
    float threshold;

    // model states
    bool triggerd = false;
    unsigned int speech_start = 0; 
    unsigned int speech_end = 0;
    unsigned int temp_end = 0;
    unsigned int current_sample = 0;    
    // MAX 4294967295 samples / 8sample per ms / 1000 / 60 = 8947 minutes  
    float output;

    // Onnx model
    // Inputs
    std::vector<Ort::Value> ort_inputs;
    
    std::vector<const char *> input_node_names = {"input", "sr", "h", "c"};
    std::vector<float> input;
    std::vector<int64_t> sr;
    unsigned int size_hc = 2 * 1 * 64; // It's FIXED.
    std::vector<float> _h;
    std::vector<float> _c;

    int64_t input_node_dims[2] = {}; 
    const int64_t sr_node_dims[1] = {1};
    const int64_t hc_node_dims[3] = {2, 1, 64};

    // Outputs
    std::vector<Ort::Value> ort_outputs;
    std::vector<const char *> output_node_names = {"output", "hn", "cn"};    
};

#endif