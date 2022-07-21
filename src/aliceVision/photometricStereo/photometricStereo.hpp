// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include <string>
#include <vector>

// SFMData
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

void photometricStereo(const std::string& inputPath, const std::string& lightData, const std::string& outputPath, const size_t HS_order, const int& downscale, aliceVision::image::Image<aliceVision::image::RGBfColor>& normals, aliceVision::image::Image<aliceVision::image::RGBfColor>& albedo);

void photometricStereo(const aliceVision::sfmData::SfMData& sfmData, const std::string& lightData, const std::string& maskPath, const std::string& outputPath, const size_t HS_order, const int& downscale, aliceVision::image::Image<aliceVision::image::RGBfColor>& normals, aliceVision::image::Image<aliceVision::image::RGBfColor>& albedo);

void photometricStereo(const std::vector<std::string>& imageList, const std::vector<std::array<float, 3>>& intList, const Eigen::MatrixXf& lightMat, aliceVision::image::Image<float>& mask, const int& downscale, aliceVision::image::Image<aliceVision::image::RGBfColor>& normals, aliceVision::image::Image<aliceVision::image::RGBfColor>& albedo);

void loadPSData(const std::string& folderPath, const size_t& HS_order, std::vector<std::array<float, 3>>& intList, Eigen::MatrixXf& lightMat);

void getPicturesNames(const std::string& folderPath, std::vector<std::string>& imageList);

bool compareFunction(std::string a, std::string b);
