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

struct prediction
{
    std::vector<std::vector<float>> bboxes;
    std::vector<float> scores;
    std::vector<std::string> masks;
    cv::Size size;
};

void model_explore(Ort::Session& session);

void sphereDetection(const aliceVision::sfmData::SfMData& sfmData, Ort::Session& session, fs::path output_path, const float min_score);
