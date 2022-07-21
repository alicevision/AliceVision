// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

// SFMData
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/image/Image.hpp>

#include <string>
#include <vector>

void normalIntegration(const std::string& inputPath, const bool& perspective, const int& downscale, const std::string& outputFodler);

void normalIntegration(const aliceVision::sfmData::SfMData& sfmData, const std::string& inputPath, const bool& perspective, const int& downscale, const std::string& outputFodler);

void DCT_integration(const aliceVision::image::Image<aliceVision::image::RGBfColor>& normals, aliceVision::image::Image<float>& depth, bool perspective, const Eigen::Matrix3f& K, const aliceVision::image::Image<float>& normalsMask);

void normal2PQ(const aliceVision::image::Image<aliceVision::image::RGBfColor>& normals, Eigen::MatrixXf& p, Eigen::MatrixXf& q, bool perspective, const Eigen::Matrix3f& K);
void normal2PQ(const aliceVision::image::Image<aliceVision::image::RGBfColor>& normals, Eigen::MatrixXf& p, Eigen::MatrixXf& q, bool perspective, const Eigen::Matrix3f& K, const aliceVision::image::Image<float>& normalsMask);

void getDivergenceField(const Eigen::MatrixXf& p, const Eigen::MatrixXf& q, Eigen::MatrixXf& f);

void setBoundaryConditions(const Eigen::MatrixXf& p, const Eigen::MatrixXf& q, Eigen::MatrixXf& f);

void adjustScale(const aliceVision::sfmData::SfMData& sfmData, aliceVision::image::Image<float>& initDepth, size_t viewID);

void convertZtoDistance(const aliceVision::image::Image<float>& zMap, aliceVision::image::Image<float>& distanceMap, const Eigen::Matrix3f& K);

void convertDistanceToZ(const aliceVision::image::Image<float>& distanceMap, aliceVision::image::Image<float>& zMap, const Eigen::Matrix3f& K);

void loadNormalMap(aliceVision::image::Image<aliceVision::image::RGBColor> inputNormals, const aliceVision::image::Image<float>& normalsMask, aliceVision::image::Image<aliceVision::image::RGBfColor>& outputNormals);
