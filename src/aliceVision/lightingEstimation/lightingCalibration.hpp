// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <string>
#include <vector>

// SFMData
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <aliceVision/lightingEstimation/sphereData.hpp>
#include <aliceVision/lightingEstimation/lightingData.hpp>

namespace cv
{
    class Mat;
}

namespace aliceVision {
namespace lightingEstimation {

class CalibrationData
{
  public:
    CalibrationData();

    bool prepareView(const aliceVision::IndexT viewId, const sfmData::SfMData& sfmData, const CalibrationSpheres& calibrationSpheres, bool usePose=false, unsigned int resolution=4);

    unsigned int nbPixels() const;

    const Eigen::MatrixX3f& getPoints();

    const Eigen::MatrixX3f& getNormals();

    const Eigen::MatrixX2<unsigned int>& getPixels();

    const Eigen::VectorXf& getPixelsIntensity();

  private:
    Eigen::MatrixX3f points;
    Eigen::MatrixX3f normals;
    Eigen::MatrixX2<unsigned int> pixels;
    Eigen::VectorXf pixelsIntensity;
};

using CalibrationDatas = sfmData::SharedPtrMap<CalibrationData>;

/**
 * @brief Calibrate lighting direction for a set of images from a .sfm file
 * @param[in] sfmData Input .sfm file to calibrate from
 * @param[in] calibrationSpheres Input spheres parameters
 * @param[in] lightings Output lightings
 */
bool lightCalibration(const sfmData::SfMData& sfmData, const CalibrationSpheres& calibrationSpheres, const LightType lightType, Lightings& lightings);

}  // namespace lightingEstimation
}  // namespace aliceVision
