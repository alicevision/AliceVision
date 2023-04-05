// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include <opencv2/opencv.hpp>

#include <onnxruntime_cxx_api.h>

void modelExplore(const Ort::Session session);

cv::Size verifySameResolution(std::string imagesPath);

cv::Mat computeAverageMask(Ort::Session& session, const std::string imagesPath, const cv::Size imageSize);

std::vector<std::pair<cv::Point2f, float>> circlesFromMask(const cv::Mat mask);
