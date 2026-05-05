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

// Boost Property Tree
#include <boost/property_tree/ptree.hpp>

// SFMData
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <filesystem>

namespace aliceVision {
namespace sphereDetection {

// namespaces
namespace fs = std::filesystem;
namespace bpt = boost::property_tree;

struct Prediction
{
    std::vector<std::vector<float>> bboxes;
    std::vector<float> scores;
    cv::Size size;
};

void fillShapeTree(bpt::ptree& fileTree, const bpt::ptree& spheresTree);

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
 * @brief Write a JSON file containing the shapes for the hand-detected spheres.
 *
 * @param sfmData Input SfMData.
 * @param x Flat vector of strings containing "view ID":"x-coordinate" pairs for the hand-detected spheres.
 * @param y Flat vector of strings containing "view ID":"y-coordinate" pairs for the hand-detected spheres.
 * @param radius Flat vector of strings containing "view ID":"radius" pairs for the hand-detected spheres.
 * @param outputPath Path to the output JSON file.
 * @param fillMissingSpheres If enabled, view IDs for which no sphere has been hand-detected will use the location of the last detected sphere.
 * @return True if the JSON file was correctly written, False otherwise.
 */
bool writeManualSphereJSON(const sfmData::SfMData& sfmData,
                           const std::vector<std::string>& x,
                           const std::vector<std::string>& y,
                           const std::vector<std::string>& radius,
                           fs::path outputPath,
                           bool fillMissingSpheres);

/**
 * @brief Write a JSON file containing the shapes for the spheres, based on a provided JSON file that contains sphere locations.
 *
 * @param sfmData Input SfMData.
 * @param sphereFile A JSON file containing the locations of the detected spheres.
 * @param outputPath Path to the output JSON file.
 * @param fillMissingSpheres If enabled, view IDs for which no sphere has been hand-detected will use the location of the last detected sphere.
 * @return True if the JSON file was correctly written, False otherwise.
 */
bool writeManualSphereJSON(const sfmData::SfMData& sfmData, const std::string& sphereFile, const std::string& outputPath, bool fillMissingSpheres);

}  // namespace sphereDetection
}  // namespace aliceVision
