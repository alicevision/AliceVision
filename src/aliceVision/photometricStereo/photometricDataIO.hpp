// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include <string>
#include <vector>

void loadLightIntensities(const std::string& intFileName, std::vector<std::array<float, 3>>& intList);

void loadLightDirections(const std::string& dirFileName, const Eigen::MatrixXf& convertionMatrix, Eigen::MatrixXf& lightMat);

void loadLightHS(const std::string& dirFileName, Eigen::MatrixXf& lightMat);

void buildLigtMatFromJSON(const std::string& fileName, const std::vector<std::string>& imageList, Eigen::MatrixXf& lightMat, std::vector<std::array<float, 3>>& intList);

void loadMask(std::string const& maskName, aliceVision::image::Image<float>& mask);

void getIndMask(aliceVision::image::Image<float> const& mask, std::vector<int>& indexes);

void intensityScaling(std::array<float, 3> const& intensities, aliceVision::image::Image<aliceVision::image::RGBfColor>& imageToScale);

void image2PsMatrix(const aliceVision::image::Image<aliceVision::image::RGBfColor>& imageIn, const aliceVision::image::Image<float>& mask, Eigen::MatrixXf& imageOut);

void applyMask(const Eigen::MatrixXf& inputMatrix, const std::vector<int>& maskIndexes, Eigen::MatrixXf& maskedMatrix);

void reshapeInImage(const Eigen::MatrixXf& matrixIn, aliceVision::image::Image<aliceVision::image::RGBfColor>& imageOut);

void convertNormalMap2png(const aliceVision::image::Image<aliceVision::image::RGBfColor>& normalsIm, aliceVision::image::Image<aliceVision::image::RGBColor>& normalsImPNG);

void readMatrix(const std::string& fileName, Eigen::MatrixXf& matrix);

void writePSResults(const std::string& outputPath, const aliceVision::image::Image<aliceVision::image::RGBfColor>& normals, const aliceVision::image::Image<aliceVision::image::RGBfColor>& albedo);

void writePSResults(const std::string& outputPath, const aliceVision::image::Image<aliceVision::image::RGBfColor>& normals, const aliceVision::image::Image<aliceVision::image::RGBfColor>& albedo, const aliceVision::IndexT& poseId);
