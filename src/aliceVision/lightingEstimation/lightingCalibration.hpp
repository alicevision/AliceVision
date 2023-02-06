// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include <string>
#include <vector>

// SFMData
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

void lightCalibration(const std::string& inputPath, const std::string& outputPath);

void lightCalibration(const aliceVision::sfmData::SfMData& sfmData, const std::string& inputJSON, const std::string& outputPath);

void lightCalibration(const aliceVision::sfmData::SfMData& sfmData, const std::array<float, 3>& sphereParam, const std::string& outputPath);

void lightCalibration(const std::vector<std::string>& imageList, const std::vector<std::array<float, 3>>& allSpheresParams, const std::string& jsonName, const std::vector<float>& focals);

void lightCalibration(const std::vector<std::string>& imageList, const std::array<float, 3>& sphereParam, const std::string& jsonName, const float& focal);

void lightCalibrationOneImage(const std::string& picturePath, const std::array<float, 3>& sphereParam, const float& focal, const std::string& method, Eigen::Vector3f& lightingDirection);

void detectBrightestPoint(const std::array<float, 3>& sphereParam, const aliceVision::image::Image<float>& imageFloat, Eigen::Vector2f& brigthestPoint);

void createTriangleKernel(const size_t kernelSize, Eigen::VectorXf& kernel);

void getNormalOnSphere(const float& x_picture, const float& y_picture, const std::array<float, 3>& sphereParam, Eigen::Vector3f& currentNormal);

void cutImage(const aliceVision::image::Image<float>& imageFloat, const std::array<float, 3>& sphereParam, aliceVision::image::Image<float>& patch, std::array<float, 2>& patchOrigin);

void writeJSON(const std::string& fileName, const std::vector<std::string>& imageList, const Eigen::MatrixXf& lightMat, const std::vector<std::array<float, 3>>& intList);
