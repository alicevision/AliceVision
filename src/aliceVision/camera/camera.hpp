// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/camera/Distortion.hpp>
#include <aliceVision/camera/Undistortion.hpp>
#include <aliceVision/camera/IntrinsicBase.hpp>

#include <memory>
#include <initializer_list>

namespace aliceVision {
namespace camera {

class Pinhole;
class Equidistant;

/**
 * @brief Factory function to create a distortion object.
 * @param[in] distortionType Type of distortion to use.
 * @param[in] params Distortion parameters.
 * @return Shared pointer of initialized distortion object.
 */
std::shared_ptr<Distortion> createDistortion(EDISTORTION distortionType, std::initializer_list<double> params = {});

/**
 * @brief Factory function to create an undistortion object.
 * @param[in] distortionType Type of distortion to use.
 * @param[in] params Distortion parameters.
 * @return Shared pointer of initialized undistortion object.
 */
std::shared_ptr<Undistortion> createUndistortion(EUNDISTORTION undistortionType,
                                                unsigned int w = 0,
                                                unsigned int h = 0,
                                                std::initializer_list<double> params = {});


/**
 * @brief Factory function to create an intrinsic object.
 * @param[in] intrinsicType Type of intrinsic to use.
 * @param[in] distortionType Type of distortion to use.
 * @param[in] undistortionType Type of undistortion to use.
 * @param[in] w Intrinsic width.
 * @param[in] h Intrinsic height.
 * @param[in] focalLengthPixX Focal length in pixels (x-axis).
 * @param[in] focalLengthPixY Focal length in pixels (y-axis).
 * @param[in] offsetX Optical offset in pixels (x-axis).
 * @param[in] offsetY Optical offset in pixels (y-axis).
 * @return Shared pointer of initialized intrinsic object.
 */
std::shared_ptr<IntrinsicBase> createIntrinsic(EINTRINSIC intrinsicType,
                                            EDISTORTION distortionType,
                                            EUNDISTORTION undistortionType,
                                            unsigned int w = 0,
                                            unsigned int h = 0,
                                            double focalLengthPixX = 0.0,
                                            double focalLengthPixY = 0.0,
                                            double offsetX = 0.0,
                                            double offsetY = 0.0);

/**
 * @brief Factory function to create a pinhole camera object.
 * @param[in] distortionType Type of distortion to use.
 * @param[in] undistortionType Type of undistortion to use.
 * @param[in] w Intrinsic width.
 * @param[in] h Intrinsic height.
 * @param[in] focalLengthPixX Focal length in pixels (x-axis).
 * @param[in] focalLengthPixY Focal length in pixels (y-axis).
 * @param[in] offsetX Optical offset in pixels (x-axis).
 * @param[in] offsetY Optical offset in pixels (y-axis).
 * @param[in] distortionParams Distortion parameters.
 * @return Shared pointer of initialized pinhole camera object.
 */
std::shared_ptr<Pinhole> createPinhole(EDISTORTION distortionType,
                                    EUNDISTORTION undistortionType,
                                    unsigned int w = 0,
                                    unsigned int h = 0,
                                    double focalLengthPixX = 0.0,
                                    double focalLengthPixY = 0.0,
                                    double offsetX = 0.0,
                                    double offsetY = 0.0,
                                    std::initializer_list<double> distortionParams = {});

/**
 * @brief Factory function to create an equidistant camera object.
 * @param[in] distortionType Type of distortion to use.
 * @param[in] w Intrinsic width.
 * @param[in] h Intrinsic height.
 * @param[in] focalLengthPixX Focal length in pixels (x-axis).
 * @param[in] focalLengthPixY Focal length in pixels (y-axis).
 * @param[in] offsetX Optical offset in pixels (x-axis).
 * @param[in] offsetY Optical offset in pixels (y-axis).
 * @param[in] distortionParams Distortion parameters.
 * @return Shared pointer of initialized equidistant camera object.
 */
std::shared_ptr<Equidistant> createEquidistant(EDISTORTION distortionType,
                                            unsigned int w = 0,
                                            unsigned int h = 0,
                                            double focalLengthPix = 0.0,
                                            double offsetX = 0.0,
                                            double offsetY = 0.0,
                                            std::initializer_list<double> distortionParams = {});


/**
 * @brief get Distortion Type for a given intrinsic object
 * @param intrinsic the abstract class IntrinsicBase instance
 * @return a distortion type enum
*/
EDISTORTION getDistortionType(const IntrinsicBase& intrinsic);

}  // namespace camera
}  // namespace aliceVision
