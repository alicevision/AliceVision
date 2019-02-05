// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "lightingEstimation.hpp"
#include "augmentedNormals.hpp"

#include <aliceVision/system/Logger.hpp>

#include <Eigen/Dense>

#include <iostream>


namespace aliceVision {
namespace lightingEstimation {

/**
 * @brief Evaluate albedo and normal product for one channel
 */
void albedoNormalsProduct(MatrixXf& rhoTimesN, const MatrixXf& albedoChannel, const image::Image<AugmentedNormal>& augmentedNormals)
{
    std::size_t validIndex = 0;
    for (int i = 0; i < augmentedNormals.size(); ++i)
    {
      // If the normal is undefined
      if(augmentedNormals(i).nx() == -1.0f && augmentedNormals(i).ny() == -1.0f && augmentedNormals(i).nz() == -1.0f)
      {
        continue;
      }
      rhoTimesN(validIndex, 0) = albedoChannel(i) * augmentedNormals(i).nx();
      rhoTimesN(validIndex, 1) = albedoChannel(i) * augmentedNormals(i).ny();
      rhoTimesN(validIndex, 2) = albedoChannel(i) * augmentedNormals(i).nz();
      rhoTimesN(validIndex, 3) = albedoChannel(i) * augmentedNormals(i).nambiant();
      rhoTimesN(validIndex, 4) = albedoChannel(i) * augmentedNormals(i).nx_ny();
      rhoTimesN(validIndex, 5) = albedoChannel(i) * augmentedNormals(i).nx_nz();
      rhoTimesN(validIndex, 6) = albedoChannel(i) * augmentedNormals(i).ny_nz();
      rhoTimesN(validIndex, 7) = albedoChannel(i) * augmentedNormals(i).nx2_ny2();
      rhoTimesN(validIndex, 8) = albedoChannel(i) * augmentedNormals(i).nz2();
      ++validIndex;
    }
}

/**
 * @brief Resolve lighting estimation problem for one channel
 */
void estimateLigthing(Eigen::Matrix<float, 9, 1>& lighting, const MatrixXf& rhoTimesN, const MatrixXf& pictureChannel)
{
    ALICEVISION_LOG_INFO("estimateLigthing: " << rhoTimesN.rows() << "x" << rhoTimesN.cols());
    lighting = rhoTimesN.colPivHouseholderQr().solve(pictureChannel);
}

void prepareImageData(MatrixXf& rhoTimesN, MatrixXf& outColor, const MatrixXf& albedoChannel, const MatrixXf& pictureChannel, const image::Image<AugmentedNormal>& augNormals)
{
  std::size_t nbValidPoints = 0;
  for (int i = 0; i < augNormals.size(); ++i)
  {
    // If the normal is undefined
    if(augNormals(i).nx() == -1.0f && augNormals(i).ny() == -1.0f && augNormals(i).nz() == -1.0f)
      continue;
    ++nbValidPoints;
  }

  rhoTimesN.resize(nbValidPoints, 9);
  albedoNormalsProduct(rhoTimesN, albedoChannel, augNormals);

  outColor.resize(nbValidPoints, 1);
  {
    std::size_t validIndex = 0;
    for (int i = 0; i < augNormals.size(); ++i)
    {
      // If the normal is undefined
      if(augNormals(i).nx() == -1.0f && augNormals(i).ny() == -1.0f && augNormals(i).nz() == -1.0f)
        continue;

      outColor(validIndex++) = pictureChannel(i);
    }
  }
}

void accumulateImageData(MatrixXf& all_rhoTimesN, MatrixXf& all_colors, const MatrixXf& albedoChannel, const MatrixXf& pictureChannel, const image::Image<AugmentedNormal>& augNormals)
{
  MatrixXf rhoTimesN;
  MatrixXf colors;
  prepareImageData(rhoTimesN, colors, albedoChannel, pictureChannel, augNormals);

  all_rhoTimesN.resize(all_rhoTimesN.rows() + rhoTimesN.rows(), rhoTimesN.cols());
  all_colors.resize(all_colors.rows() + colors.rows(), colors.cols());

  all_rhoTimesN.bottomLeftCorner(rhoTimesN.rows(), rhoTimesN.cols()) << rhoTimesN;
  all_colors.bottomLeftCorner(colors.rows(), colors.cols()) << colors;
}

void estimateLigthingOneChannel(Eigen::Matrix<float, 9, 1>& lighting, const MatrixXf& albedoChannel, const MatrixXf& pictureChannel, const image::Image<AugmentedNormal>& augNormals)
{
  MatrixXf rhoTimesN;
  MatrixXf colors;
  prepareImageData(rhoTimesN, colors, albedoChannel, pictureChannel, augNormals);
  estimateLigthing(lighting, rhoTimesN, colors);
}

void estimateLigthingLuminance(LightingVector& lighting, const image::Image<image::RGBfColor>& albedo, const image::Image<image::RGBfColor>& picture, const image::Image<image::RGBfColor>& normals)
{
    using namespace Eigen;

    // Map albedo, image
    const std::size_t nbPixels = albedo.Width() * albedo.Height();

    // Augmented normales
    image::Image<AugmentedNormal> augNormals(normals.cast<AugmentedNormal>());

    MatrixXf rhoTimesN;
    MatrixXf colors;

    // estimate Lighting per Channel
    for(std::size_t c = 0; c < 3; ++c)
    {
        // Map albedo, image
        Map<MatrixXf, 0, InnerStride<3>> albedoC((float*)&(albedo(0,0)(c)), nbPixels, 1);
        Map<MatrixXf, 0, InnerStride<3>> pictureC((float*)&(picture(0,0)(c)), nbPixels, 1);

        accumulateImageData(rhoTimesN, colors, albedoC, pictureC, augNormals);

        ALICEVISION_LOG_INFO("estimateLigthingLuminance (channel=" << c << "): " << rhoTimesN.rows() << "x" << rhoTimesN.cols());
    }

    Eigen::Matrix<float, 9, 1> lightingL;
    estimateLigthing(lightingL, rhoTimesN, colors);

    // lighting vectors fusion
    lighting.col(0) = lightingL;
    lighting.col(1) = lightingL;
    lighting.col(2) = lightingL;
}


void estimateLigthingRGB(LightingVector& lighting, const image::Image<image::RGBfColor>& albedo, const image::Image<image::RGBfColor>& picture, const image::Image<image::RGBfColor>& normals)
{
    using namespace Eigen;

    // Augmented normales
    image::Image<AugmentedNormal> augNormals(normals.cast<AugmentedNormal>());

    const std::size_t nbPixels = albedo.Width() * albedo.Height();

    // estimate Lighting per Channel
    for(std::size_t c = 0; c < 3; ++c)
    {
        // Map albedo, image
        Map<MatrixXf, 0, InnerStride<3>> albedoC((float*)&(albedo(0,0)(c)), nbPixels, 1);
        Map<MatrixXf, 0, InnerStride<3>> pictureC((float*)&(picture(0,0)(c)), nbPixels, 1);

        Eigen::Matrix<float, 9, 1> lightingC;

        estimateLigthingOneChannel(lightingC, albedoC, pictureC, augNormals);

        // lighting vectors fusion
        lighting.col(c) = lightingC;
    }
}

}
}
