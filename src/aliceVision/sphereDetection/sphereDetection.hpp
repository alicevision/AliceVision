// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
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

// Boost
#include <boost/filesystem.hpp>

// namespaces
namespace fs = boost::filesystem;

namespace aliceVision {
namespace sphereDetection {

struct prediction
{
    std::vector<std::vector<float>> bboxes;
    std::vector<float> scores;
    std::vector<std::string> masks;
    cv::Size size;
};

/**
 * @brief Prints inputs and outputs of neural network, and checks the requirements.
 * @param session the ONNXRuntime session
 */
void model_explore(Ort::Session& session);

/**
 * @brief Use ONNXRuntime to make a prediction
 *
 * @param session
 * @param image_path the path to the input image
 * @return cv::Mat, the prediction
 */
void sphereDetection(const sfmData::SfMData& sfmData, Ort::Session& session, fs::path output_path, const float min_score);

/**
 * @brief Write json for a hand-detected sphere
 *
 * @param sfmData - Input .sfm file
 * @param sphereParam - Parameters of the hand-detected sphere
 * @return output_path - Path to the json file
 */
void writeManualSphereJSON(const sfmData::SfMData& sfmData, const std::array<float, 3>& sphereParam, fs::path output_path);

}
}
