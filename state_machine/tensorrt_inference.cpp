#include "tensorrt_inference.hpp"
#include <NvInferRuntime.h>
#include <fstream>

InferenceEngine::InferenceEngine(const std::string& model_path, int input_dim_, int output_dim_) :
    output_dim_(output_dim_), input_dim_(input_dim_) {

    runtime_ = nvinfer1::createInferRuntime(logger_);
    if (!runtime_) {
        throw std::runtime_error("Failed to create TensorRT runtime.");
    }

    std::ifstream file(model_path.c_str(), std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open engine file: " + model_path);
    }
    file.seekg(0, std::ios::end);
    size_t fsize = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> modelStream(fsize);
    if (!file.read(modelStream.data(), fsize)) {
        throw std::runtime_error("Failed to read engine file.");
    }
    file.close();

    engine_ = runtime_->deserializeCudaEngine(modelStream.data(), modelStream.size());
    if (!engine_) {
        throw std::runtime_error("Failed to deserialize engine.");
    }
    context_ = engine_->createExecutionContext();

    if (!context_) {
        throw std::runtime_error("Failed to create execution context.");
    }

    cudaMalloc(&device_buffers_[0], input_dim_ * sizeof(float));
    cudaMalloc(&device_buffers_[1], output_dim_ * sizeof(float));

    input_buffer_.resize(input_dim_);
    output_buffer_.resize(output_dim_);
}

InferenceEngine::~InferenceEngine() {
    if (context_) delete context_;
    if (engine_) delete engine_;
    if (runtime_) delete runtime_;
    cudaFree(device_buffers_[0]);
    cudaFree(device_buffers_[1]);
}

void InferenceEngine::infer(const std::vector<float>& input, std::vector<float>& output) {
    if (input.size() != (size_t)input_dim_) {
        throw std::runtime_error("Input dimension mismatch. Got " + std::to_string(input.size()) +
                                 " expected " + std::to_string(input_dim_));
    }
    if (output.size() != (size_t)output_dim_) {
        throw std::runtime_error("Output dimension mismatch. Got " + std::to_string(output.size()) +
                                 " expected " + std::to_string(output_dim_));
    }

    cudaMemcpy(device_buffers_[0], input.data(), input.size() * sizeof(float), cudaMemcpyHostToDevice);
    context_->executeV2(device_buffers_);
    cudaMemcpy(output.data(), device_buffers_[1], output_dim_ * sizeof(float), cudaMemcpyDeviceToHost);
}
