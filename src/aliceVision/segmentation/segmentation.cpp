// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "segmentation.hpp"

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CUDA)
    #include <cuda_runtime.h>
#endif

#include <aliceVision/image/all.hpp>
#include <aliceVision/image/imageAlgo.hpp>
#include <aliceVision/numeric/numeric.hpp>

namespace aliceVision {
namespace segmentation {

void imageToPlanes(std::vector<float>& output, const image::Image<image::RGBfColor>::Base& source)
{
    size_t planeSize = source.rows() * source.cols();

    output.resize(planeSize * 3);

    float* planeR = output.data();
    float* planeG = planeR + planeSize;
    float* planeB = planeG + planeSize;

    size_t pos = 0;
    for (int i = 0; i < source.rows(); i++)
    {
        for (int j = 0; j < source.cols(); j++)
        {
            const image::RGBfColor& rgb = source(i, j);
            planeR[pos] = rgb.r();
            planeG[pos] = rgb.g();
            planeB[pos] = rgb.b();

            pos++;
        }
    }
}

bool Segmentation::initialize()
{
    const auto& api = Ort::GetApi();

    _ortEnvironment = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "aliceVision-imageSegmentation");

    Ort::SessionOptions ortSessionOptions;

    // this is false if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ONNX_GPU) is false
    if (_parameters.useGpu)
    {
// Disable for compilation purpose if needed
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ONNX_GPU)
        OrtCUDAProviderOptionsV2* cuda_options = nullptr;
        api.CreateCUDAProviderOptions(&cuda_options);
        api.SessionOptionsAppendExecutionProvider_CUDA_V2(static_cast<OrtSessionOptions*>(ortSessionOptions), cuda_options);
        api.ReleaseCUDAProviderOptions(cuda_options);

    #if defined(_WIN32) || defined(_WIN64)
        std::wstring modelWeights(_parameters.modelWeights.begin(), _parameters.modelWeights.end());
        _ortSession = std::make_unique<Ort::Session>(*_ortEnvironment, modelWeights.c_str(), ortSessionOptions);
    #else
        _ortSession = std::make_unique<Ort::Session>(*_ortEnvironment, _parameters.modelWeights.c_str(), ortSessionOptions);
    #endif

        Ort::MemoryInfo memInfoCuda("Cuda", OrtAllocatorType::OrtArenaAllocator, 0, OrtMemType::OrtMemTypeDefault);
        Ort::Allocator cudaAllocator(*_ortSession, memInfoCuda);

        _output.resize(_parameters.classes.size() * _parameters.modelHeight * _parameters.modelWidth);
        _cudaInput = cudaAllocator.Alloc(_output.size() * sizeof(float));
        _cudaOutput = cudaAllocator.Alloc(_output.size() * sizeof(float));
#endif
    }
    else
    {
#if defined(_WIN32) || defined(_WIN64)
        std::wstring modelWeights(_parameters.modelWeights.begin(), _parameters.modelWeights.end());
        _ortSession = std::make_unique<Ort::Session>(*_ortEnvironment, modelWeights.c_str(), ortSessionOptions);
#else
        _ortSession = std::make_unique<Ort::Session>(*_ortEnvironment, _parameters.modelWeights.c_str(), ortSessionOptions);
#endif
    }

    return true;
}

bool Segmentation::terminate()
{
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ONNX_GPU)
    Ort::MemoryInfo mem_info_cuda("Cuda", OrtAllocatorType::OrtArenaAllocator, 0, OrtMemType::OrtMemTypeDefault);
    Ort::Allocator cudaAllocator(*_ortSession, mem_info_cuda);
    cudaAllocator.Free(_cudaInput);
    cudaAllocator.Free(_cudaOutput);
#endif

    return true;
}

bool Segmentation::processImage(image::Image<IndexT>& labels, const image::Image<image::RGBfColor>& source)
{
    // Todo : handle orientation and small images smaller than model input

    // Compute the optimal resized size such that:
    // - at last one dimension fit the model
    // - both dimensions are larger or equal than the model dimensions
    int resizedHeight = 0;
    int resizedWidth = 0;
    if (source.height() < source.width())
    {
        resizedWidth =
          static_cast<int>(static_cast<double>(source.width()) * static_cast<double>(_parameters.modelHeight) / static_cast<double>(source.height()));
        if (resizedWidth < _parameters.modelWidth)
        {
            resizedWidth = _parameters.modelWidth;
            resizedHeight = static_cast<int>(static_cast<double>(resizedWidth) * static_cast<double>(_parameters.modelHeight) /
                                             static_cast<double>(_parameters.modelWidth));
        }
        else
        {
            resizedHeight = _parameters.modelHeight;
        }
    }
    else
    {
        resizedHeight =
          static_cast<int>(static_cast<double>(source.height()) * static_cast<double>(_parameters.modelWidth) / static_cast<double>(source.width()));
        if (resizedHeight < _parameters.modelHeight)
        {
            resizedHeight = _parameters.modelHeight;
            resizedWidth = static_cast<int>(static_cast<double>(resizedHeight) * static_cast<double>(_parameters.modelWidth) /
                                            static_cast<double>(_parameters.modelHeight));
        }
        else
        {
            resizedWidth = _parameters.modelWidth;
        }
    }

    // Resize image
    image::Image<image::RGBfColor> resized;
    imageAlgo::resizeImage(resizedWidth, resizedHeight, source, resized);

    // Normalize image to fit model statistics
    for (int i = 0; i < resizedHeight; i++)
    {
        for (int j = 0; j < resizedWidth; j++)
        {
            image::RGBfColor value = resized(i, j);
            resized(i, j) = (value - _parameters.center) * _parameters.scale;
        }
    }

    image::Image<IndexT> resizedLabels;
    if (!tiledProcess(resizedLabels, resized))
    {
        return false;
    }

    imageAlgo::resampleImage(source.width(), source.height(), resizedLabels, labels, false);

    return true;
}

bool Segmentation::tiledProcess(image::Image<IndexT>& labels, const image::Image<image::RGBfColor>& source)
{
    // Compute the theorical tiles count
    int cwidth = divideRoundUp(source.width(), _parameters.modelWidth);
    int cheight = divideRoundUp(source.height(), _parameters.modelHeight);

    image::Image<ScoredLabel> scoredLabels(source.width(), source.height(), true, {0, 0.0f});

    // Loop over tiles
    for (int i = 0; i < cheight; i++)
    {
        // Compute starting point with overlap on previous
        int y = std::max(0, int(i * _parameters.modelHeight - _parameters.overlapRatio * _parameters.modelHeight));
        int ly = y + _parameters.modelHeight;

        // If we are on the end border, shift on the other side
        int shifty = source.height() - ly;
        if (shifty < 0)
        {
            y = std::max(0, y + shifty);
        }

        for (int j = 0; j < cwidth; j++)
        {
            // Compute starting point with overlap on previous
            int x = std::max(0, int(j * _parameters.modelWidth - _parameters.overlapRatio * _parameters.modelWidth));
            int lx = x + _parameters.modelWidth;

            // If we are on the end border, shift on the other side
            int shiftx = source.width() - lx;
            if (shiftx < 0)
            {
                x = std::max(0, x + shiftx);
            }

            // x and y contains the position of the tile in the input image
            auto& block = source.block(y, x, _parameters.modelHeight, _parameters.modelWidth);

            // Compute tile
            image::Image<ScoredLabel> tileLabels(_parameters.modelWidth, _parameters.modelHeight, true, {0, 0.0f});

            if (_parameters.useGpu)
            {
                processTileGPU(tileLabels, block);
            }
            else
            {
                processTile(tileLabels, block);
            }

            // Update the global labeling
            mergeLabels(scoredLabels, tileLabels, x, y);
        }
    }

    labels = scoredLabels.cast<IndexT>();

    return true;
}

bool Segmentation::mergeLabels(image::Image<ScoredLabel>& labels, image::Image<ScoredLabel>& tileLabels, int tileX, int tileY)
{
    for (int i = 0; i < tileLabels.height(); i++)
    {
        int y = i + tileY;
        for (int j = 0; j < tileLabels.width(); j++)
        {
            int x = j + tileX;

            if (tileLabels(i, j).score > labels(y, x).score)
            {
                labels(y, x) = tileLabels(i, j);
            }
        }
    }

    return true;
}

bool Segmentation::labelsFromModelOutput(image::Image<ScoredLabel>& labels, const std::vector<float>& modelOutput)
{
    for (int outputY = 0; outputY < _parameters.modelHeight; outputY++)
    {
        for (int outputX = 0; outputX < _parameters.modelWidth; outputX++)
        {
            int maxClasse = 0;
            int maxVal = 0;

            for (int classe = 0; classe < _parameters.classes.size(); classe++)
            {
                int classPos = classe * _parameters.modelWidth * _parameters.modelHeight;
                int pos = classPos + outputY * _parameters.modelWidth + outputX;

                float val = modelOutput[pos];
                if (val > maxVal)
                {
                    maxVal = val;
                    maxClasse = classe;
                }
            }

            labels(outputY, outputX) = {static_cast<IndexT>(maxClasse), static_cast<float>(maxVal)};
        }
    }

    return true;
}

bool Segmentation::processTile(image::Image<ScoredLabel>& labels, const image::Image<image::RGBfColor>::Base& source)
{
    ALICEVISION_LOG_TRACE("Process tile using cpu");
    Ort::MemoryInfo memInfo = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

    std::vector<const char*> inputNames{"input"};
    std::vector<const char*> outputNames{"output"};
    std::vector<int64_t> inputDimensions = {1, 3, _parameters.modelHeight, _parameters.modelWidth};
    std::vector<int64_t> outputDimensions = {1, static_cast<int64_t>(_parameters.classes.size()), _parameters.modelHeight, _parameters.modelWidth};

    std::vector<float> output(_parameters.classes.size() * _parameters.modelHeight * _parameters.modelWidth);
    Ort::Value outputTensors =
      Ort::Value::CreateTensor<float>(memInfo, output.data(), output.size(), outputDimensions.data(), outputDimensions.size());

    std::vector<float> transformedInput;
    imageToPlanes(transformedInput, source);

    Ort::Value inputTensors =
      Ort::Value::CreateTensor<float>(memInfo, transformedInput.data(), transformedInput.size(), inputDimensions.data(), inputDimensions.size());

    try
    {
        _ortSession->Run(Ort::RunOptions{nullptr}, inputNames.data(), &inputTensors, 1, outputNames.data(), &outputTensors, 1);
    }
    catch (const Ort::Exception& exception)
    {
        ALICEVISION_LOG_ERROR("ERROR running model inference: " << exception.what());
        return false;
    }

    if (!labelsFromModelOutput(labels, output))
    {
        return false;
    }

    return true;
}

bool Segmentation::processTileGPU(image::Image<ScoredLabel>& labels, const image::Image<image::RGBfColor>::Base& source)
{
    ALICEVISION_LOG_TRACE("Process tile using gpu");
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CUDA)
    Ort::MemoryInfo mem_info_cuda("Cuda", OrtAllocatorType::OrtArenaAllocator, 0, OrtMemType::OrtMemTypeDefault);
    Ort::Allocator cudaAllocator(*_ortSession, mem_info_cuda);

    std::vector<const char*> inputNames{"input"};
    std::vector<const char*> outputNames{"output"};
    std::vector<int64_t> inputDimensions = {1, 3, _parameters.modelHeight, _parameters.modelWidth};
    std::vector<int64_t> outputDimensions = {1, static_cast<int64_t>(_parameters.classes.size()), _parameters.modelHeight, _parameters.modelWidth};

    Ort::Value outputTensors = Ort::Value::CreateTensor<float>(
      mem_info_cuda, reinterpret_cast<float*>(_cudaOutput), _output.size(), outputDimensions.data(), outputDimensions.size());

    std::vector<float> transformedInput;
    imageToPlanes(transformedInput, source);

    cudaMemcpy(_cudaInput, transformedInput.data(), sizeof(float) * transformedInput.size(), cudaMemcpyHostToDevice);

    Ort::Value inputTensors = Ort::Value::CreateTensor<float>(
      mem_info_cuda, reinterpret_cast<float*>(_cudaInput), transformedInput.size(), inputDimensions.data(), inputDimensions.size());

    try
    {
        _ortSession->Run(Ort::RunOptions{nullptr}, inputNames.data(), &inputTensors, 1, outputNames.data(), &outputTensors, 1);
    }
    catch (const Ort::Exception& exception)
    {
        ALICEVISION_LOG_ERROR("ERROR running model inference: " << exception.what());
        return false;
    }

    cudaMemcpy(_output.data(), _cudaOutput, sizeof(float) * _output.size(), cudaMemcpyDeviceToHost);

    if (!labelsFromModelOutput(labels, _output))
    {
        return false;
    }

#endif

    return true;
}

}  // namespace segmentation
}  // namespace aliceVision