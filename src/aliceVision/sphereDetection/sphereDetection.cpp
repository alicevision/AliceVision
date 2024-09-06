// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sphereDetection/sphereDetection.hpp>

// Standard libs
#include <iostream>
#include <numeric>

// AliceVision image library
#include <aliceVision/image/all.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/image/Image.hpp>

// AliceVision logger
#include <aliceVision/system/Logger.hpp>

// ONNX Runtime
#include <onnxruntime_cxx_api.h>

// Helper to convert Eigen Matrix to OpenCV image
#include <aliceVision/imageMasking/eigen2cvHelpers.hpp>

// OpenCv
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Boost JSON
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

// SFMData
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

// namespaces
namespace bpt = boost::property_tree;

namespace aliceVision {
namespace sphereDetection {

void modelExplore(Ort::Session& session)
{
    // Define allocator
    Ort::AllocatorWithDefaultOptions allocator;

    // Print infos of inputs
    size_t inputCount = session.GetInputCount();
    for (size_t i = 0; i < inputCount; ++i)
    {
#if ORT_API_VERSION >= 14
        const Ort::AllocatedStringPtr inputName = session.GetInputNameAllocated(i, allocator);
        ALICEVISION_LOG_DEBUG("Input[" << i << "]: " << inputName.get());
#else
        const char* inputName = session.GetInputName(i, allocator);
        ALICEVISION_LOG_DEBUG("Input[" << i << "]: " << inputName);
#endif

        Ort::TypeInfo inputInfo = session.GetInputTypeInfo(i);
        auto inputInfo2 = inputInfo.GetTensorTypeAndShapeInfo();

        ONNXTensorElementDataType inputType = inputInfo2.GetElementType();
        ALICEVISION_LOG_DEBUG("  Type : " << inputType);

        std::vector<int64_t> inputShape = inputInfo2.GetShape();
        size_t inputSize = std::accumulate(begin(inputShape), end(inputShape), 1, std::multiplies<float>());
        ALICEVISION_LOG_DEBUG("  Shape: " << inputShape);
        ALICEVISION_LOG_DEBUG("  Size : " << inputSize);
    }

    // Print infos of outputs
    const size_t outputCount = session.GetOutputCount();
    for (size_t i = 0; i < outputCount; ++i)
    {
#if ORT_API_VERSION >= 14
        const Ort::AllocatedStringPtr outputName = session.GetOutputNameAllocated(i, allocator);
        ALICEVISION_LOG_DEBUG("Output[" << i << "]: " << outputName.get());
#else
        const char* outputName = session.GetOutputName(i, allocator);
        ALICEVISION_LOG_DEBUG("Output[" << i << "]: " << outputName);
#endif

        Ort::TypeInfo outputInfo = session.GetOutputTypeInfo(i);
        auto outputInfo2 = outputInfo.GetTensorTypeAndShapeInfo();

        ONNXTensorElementDataType outputType = outputInfo2.GetElementType();
        ALICEVISION_LOG_DEBUG("  Type: " << outputType);

        std::vector<int64_t> outputShape = outputInfo2.GetShape();
        const size_t outputSize = std::accumulate(begin(outputShape), end(outputShape), 1, std::multiplies<float>());
        ALICEVISION_LOG_DEBUG("  Shape: " << outputShape);
        ALICEVISION_LOG_DEBUG("  Size: " << outputSize);
    }
}

Prediction predict(Ort::Session& session, const fs::path imagePath, const float minScore)
{
    // Read image
    image::Image<image::RGBColor> imageAlice;
    image::readImage(imagePath.string(), imageAlice, image::EImageColorSpace::SRGB);

    // Eigen -> OpenCV
    cv::Mat imageOpencv;
    cv::eigen2cv(imageAlice.getMat(), imageOpencv);
    cv::Size imageOpencvShape = imageOpencv.size();

    // uint8 -> float32
    imageOpencv.convertTo(imageOpencv, CV_32FC3, 1 / 255.0);

    // HWC to CHW
    cv::dnn::blobFromImage(imageOpencv, imageOpencv);

    // Inference on CPU
    // TODO: use GPU
    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

    // Initialize input tensor
    std::vector<int64_t> inputShape = {1, 3, imageAlice.height(), imageAlice.width()};
    const size_t inputSize = std::accumulate(begin(inputShape), end(inputShape), 1, std::multiplies<size_t>());
    std::vector<float> inputTensor(inputSize);
    inputTensor.assign(imageOpencv.begin<float>(), imageOpencv.end<float>());

    // Create input data
    std::vector<Ort::Value> inputData;
    inputData.push_back(Ort::Value::CreateTensor<float>(memoryInfo, inputTensor.data(), inputSize, inputShape.data(), inputShape.size()));

    // Select inputs and outputs
    std::vector<const char*> inputNames{"input"};
    std::vector<const char*> outputNames{"boxes", "scores", "masks"};

    // Run the inference
    auto output =
      session.Run(Ort::RunOptions{nullptr}, inputNames.data(), inputData.data(), inputNames.size(), outputNames.data(), outputNames.size());

    // Get pointers to outputs
    float* bboxesPtr = output.at(0).GetTensorMutableData<float>();
    float* scoresPtr = output.at(1).GetTensorMutableData<float>();

    // Get output shape
    const auto infos = output.at(2).GetTensorTypeAndShapeInfo();
    const auto shape = infos.GetShape();

    // Get scores of detections
    std::vector<float> allScores = {scoresPtr, scoresPtr + shape[0]};

    // Initialize arrays
    std::vector<std::vector<float>> bboxes;
    std::vector<float> scores;

    // Filter detections and fill arrays
    for (size_t i = 0; i < shape[0]; ++i)
    {
        float score = allScores.at(i);
        if (score > minScore)
        {
            // Extract bboxe
            std::vector<float> bboxe(bboxesPtr + 4 * i, bboxesPtr + 4 * (i + 1));
            bboxes.push_back(bboxe);

            // Extract score
            scores.push_back(score);
        }
    }

    return Prediction{bboxes, scores, imageOpencvShape};
}

void sphereDetection(const sfmData::SfMData& sfmData, Ort::Session& session, fs::path outputPath, const float minScore)
{
    // Main tree
    bpt::ptree fileTree;

    for (auto& viewID : sfmData.getViews())
    {
        ALICEVISION_LOG_DEBUG("View Id: " << viewID);

        const std::string sphereName = std::to_string(viewID.second->getViewId());
        const fs::path imagePath = fs::path(sfmData.getView(viewID.second->getViewId()).getImage().getImagePath());

        if (boost::algorithm::icontains(imagePath.stem().string(), "ambient"))
            continue;

        const auto pred = predict(session, imagePath, minScore);

        // If there is no bounding box, then no sphere has been detected
        if (pred.bboxes.size() > 0)
        {
            bpt::ptree spheresNode;

            // We only take the best sphere in the picture
            const int i = 0;
            // Compute sphere coords from bbox coords
            const auto bbox = pred.bboxes.at(i);
            const float r = std::min(bbox.at(3) - bbox.at(1), bbox.at(2) - bbox.at(0)) / 2;
            const float x = bbox.at(0) + r - pred.size.width / 2;
            const float y = bbox.at(1) + r - pred.size.height / 2;

            // Create an unnamed node containing the sphere
            bpt::ptree sphereNode;
            sphereNode.put("x", x);
            sphereNode.put("y", y);
            sphereNode.put("r", r);
            sphereNode.put("score", pred.scores.at(i));
            sphereNode.put("type", "matte");

            // Add sphere to array
            spheresNode.push_back(std::make_pair("", sphereNode));

            fileTree.add_child(sphereName, spheresNode);
        }
        else
        {
            ALICEVISION_LOG_WARNING("No sphere detected for '" << imagePath << "'.");
        }
    }
    bpt::write_json(outputPath.string(), fileTree);
}

void writeManualSphereJSON(const sfmData::SfMData& sfmData, const std::array<float, 3>& sphereParam, fs::path outputPath)
{
    // Main tree
    bpt::ptree fileTree;

    for (auto& viewID : sfmData.getViews())
    {
        ALICEVISION_LOG_DEBUG("View Id: " << viewID);

        const std::string sphereName = std::to_string(viewID.second->getViewId());

        bpt::ptree spheresNode;
        // Create an unnamed node containing the sphere
        bpt::ptree sphereNode;
        sphereNode.put("x", sphereParam[0]);
        sphereNode.put("y", sphereParam[1]);
        sphereNode.put("r", sphereParam[2]);
        sphereNode.put("type", "matte");

        // Add sphere to array
        spheresNode.push_back(std::make_pair("", sphereNode));

        fileTree.add_child(sphereName, spheresNode);
    }
    bpt::write_json(outputPath.string(), fileTree);
}

}  // namespace sphereDetection
}  // namespace aliceVision
