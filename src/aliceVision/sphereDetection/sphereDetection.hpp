// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

// OpenCV
#include <opencv2/opencv.hpp>

// ONNXRuntime
#include <onnxruntime_cxx_api.h>

// SFMData
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <filesystem>

namespace aliceVision {
namespace sphereDetection {

// namespaces
namespace fs = std::filesystem;

struct Prediction
{
    std::vector<std::vector<float>> bboxes;
    std::vector<float> scores;
    cv::Size size;
};

/**
 * @brief Print inputs and outputs of neural network, and checks the requirements
 * @param session The ONNXRuntime session
 */
void modelExplore(Ort::Session& session);

/**
 * @brief Detect a sphere using ONNXRuntime to make a prediction
 *
 * @param sfmData The input SfMData file
 * @param session The ONNXRuntime session
 * @param outputPath The path to write the JSON with the detected spheres to
 * @return minScore The minimum score for the predictions
 */
void sphereDetection(const sfmData::SfMData& sfmData, Ort::Session& session, fs::path outputPath, const float minScore);

/**
 * @brief Write JSON for a hand-detected sphere
 *
 * @param sfmData Input .sfm file
 * @param sphereParam Parameters of the hand-detected sphere
 * @return outputPath Path to the JSON file
 */
void writeManualSphereJSON(const sfmData::SfMData& sfmData, const std::array<float, 3>& sphereParam, fs::path outputPath);

}  // namespace sphereDetection
}  // namespace aliceVision
