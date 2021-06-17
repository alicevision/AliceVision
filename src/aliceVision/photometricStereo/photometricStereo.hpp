// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include <string>
#include <vector>

void photometricStero(const std::string& folderPath, const bool isPerspective, aliceVision::image::Image<aliceVision::image::RGBfColor>& normals);

void loadPSData(const std::string& folderPath, std::vector<std::array<float, 3>>& intList, Eigen::MatrixXf& lightMat, Eigen::MatrixXf& convertionMatrix, aliceVision::image::Image<float>& mask);

void getPicturesNames(const std::string& folderPath, std::vector<std::string>& imageList);

bool compareFunction (std::string a, std::string b);
