// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

struct prediction
{
    cv::Mat bboxes;
    std::vector<float> scores;
    std::vector<cv::Mat> masks;
};

void model_explore(Ort::Session& session);

std::vector<std::string> get_images_paths(std::string path_images);

prediction predict(Ort::Session& session, const std::string image_path);

// void export_json(std::string output_path, std::vector<circle_info> circles);
