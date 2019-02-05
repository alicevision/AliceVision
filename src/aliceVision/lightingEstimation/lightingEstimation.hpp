// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "augmentedNormals.hpp"

#include <aliceVision/image/Image.hpp>
#include <aliceVision/image/pixelTypes.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <iostream>

namespace aliceVision {
namespace lightingEstimation {

using Eigen::MatrixXf;

enum class ELightingColor {
  Luminance = 1,
  RGB = 3
};

inline std::string ELightingColor_enumToString(ELightingColor v)
{
  switch(v)
  {
    case ELightingColor::RGB:
      return "rgb";
    case ELightingColor::Luminance:
      return "luminance";
  }
  throw std::out_of_range("Invalid LightingColor type Enum: " + std::to_string(int(v)));
}

inline ELightingColor ELightingColor_stringToEnum(const std::string& v)
{
  std::string vv = v;
  std::transform(vv.begin(), vv.end(), vv.begin(), ::tolower); //tolower

  if(vv == "rgb")
    return ELightingColor::RGB;
  if(vv == "luminance")
    return ELightingColor::Luminance;
  throw std::out_of_range("Invalid LightingColor type string " + v);
}

inline std::ostream& operator<<(std::ostream& os, ELightingColor v)
{
    return os << ELightingColor_enumToString(v);
}

inline std::istream& operator>>(std::istream& in, ELightingColor& v)
{
    std::string token;
    in >> token;
    v = ELightingColor_stringToEnum(token);
    return in;
}

/**
 * @brief Augmented lighting vetor for augmented Lambert's law (using Spherical Harmonics model)
 * Composed of 9 coefficients
 */ 
using LightingVector = Eigen::Matrix<float, 9, 3>;

void estimateLigthing(Eigen::Matrix<float, 9, 1>& lighting, const MatrixXf& rhoTimesN, const MatrixXf& pictureChannel);

/**
 * @brief Lighting estimation from picture, albedo and geometry
 */ 
void estimateLigthingLuminance(LightingVector& lighting, const image::Image<image::RGBfColor>& albedo, const image::Image<image::RGBfColor>& picture, const image::Image<image::RGBfColor>& normals);

/**
 * @brief Lighting estimation from picture, albedo and geometry
 */
void estimateLigthingRGB(LightingVector& lighting, const image::Image<image::RGBfColor>& albedo, const image::Image<image::RGBfColor>& picture, const image::Image<image::RGBfColor>& normals);

inline void estimateLigthing(LightingVector& lighting, const image::Image<image::RGBfColor>& albedo, const image::Image<image::RGBfColor>& picture, const image::Image<image::RGBfColor>& normals, ELightingColor lightingColor)
{
  switch(lightingColor)
  {
  case ELightingColor::RGB:
    estimateLigthingRGB(lighting, albedo, picture, normals);
    break;
  case ELightingColor::Luminance:
    estimateLigthingLuminance(lighting, albedo, picture, normals);
    break;
  }
}

void accumulateImageData(MatrixXf& all_rhoTimesN, MatrixXf& all_colors, const MatrixXf& albedoChannel, const MatrixXf& pictureChannel, const image::Image<AugmentedNormal>& augNormals);


}
}
