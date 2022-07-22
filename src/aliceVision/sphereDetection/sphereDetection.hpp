// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

void model_explore(const Ort::Session session);

cv::Size resolution_verify(std::string path_images);

cv::Mat compute_mask(Ort::Session& session, const std::string image_path, const cv::Size image_size);
cv::Mat compute_mask_mean(Ort::Session& session, const std::string images_path, const cv::Size image_size);

std::vector<std::pair<cv::Point2f, float>> compute_circles(const cv::Mat mask);
