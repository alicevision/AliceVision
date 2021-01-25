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

void prepareImageData(const MatrixXf& albedo,
                      const MatrixXf& picture,
                      const image::Image<AugmentedNormal>& augmentedNormals,
                      MatrixXf& rhoTimesN,
                      MatrixXf& colors)
{
  std::size_t nbValidPoints = 0;

  for(int i = 0; i < augmentedNormals.size(); ++i)
  {
    // if the normal is undefined
    if(augmentedNormals(i).nx() == -1.0f && augmentedNormals(i).ny() == -1.0f && augmentedNormals(i).nz() == -1.0f)
      continue;
    ++nbValidPoints;
  }

  rhoTimesN.resize(nbValidPoints, 9);
  albedoNormalsProduct(rhoTimesN, albedo, augmentedNormals);
  colors.resize(nbValidPoints, 1);

  {
    std::size_t validIndex = 0;
    for(int i = 0; i < augmentedNormals.size(); ++i)
    {
      // if the normal is undefined
      if(augmentedNormals(i).nx() == -1.0f && augmentedNormals(i).ny() == -1.0f && augmentedNormals(i).nz() == -1.0f)
        continue;

      colors(validIndex++) = picture(i);
    }
  }
}

void LighthingEstimator::addImage(const image::Image<float>& albedo, const image::Image<float>& picture, const image::Image<image::RGBfColor>& normals)
{
  using namespace Eigen;

  // augmented normales
  image::Image<AugmentedNormal> augmentedNormals(normals.cast<AugmentedNormal>());

  const std::size_t nbPixels = augmentedNormals.Width() * augmentedNormals.Height();

  MatrixXf rhoTimesN;
  MatrixXf colors;

  // map albedo, image
  Map<const MatrixXf> channelAlbedo(albedo.data(), nbPixels, 1);
  Map<const MatrixXf> channelPicture(picture.data(), nbPixels, 1);

  prepareImageData(channelAlbedo, channelPicture, augmentedNormals, rhoTimesN, colors);

  // store image data
  _all_rhoTimesN.at(0).push_back(rhoTimesN);
  _all_pictureChannel.at(0).push_back(colors);
}

void LighthingEstimator::addImage(const image::Image<image::RGBfColor>& albedo, const image::Image<image::RGBfColor>& picture, const image::Image<image::RGBfColor>& normals)
{
  using namespace Eigen;

  // augmented normales
  image::Image<AugmentedNormal> augmentedNormals(normals.cast<AugmentedNormal>());

  const std::size_t nbPixels = augmentedNormals.Width() * augmentedNormals.Height();

  // estimate lighting per channel
  for(std::size_t channel = 0; channel < 3; ++channel)
  {
    MatrixXf rhoTimesN;
    MatrixXf colors;

    // map albedo, image
    Map<const MatrixXf, 0, InnerStride<3>> channelAlbedo(static_cast<const float*>(&(albedo(0,0)(channel))), nbPixels, 1);
    Map<const MatrixXf, 0, InnerStride<3>> channelPicture(static_cast<const float*>(&(picture(0,0)(channel))), nbPixels, 1);

    prepareImageData(channelAlbedo, channelPicture, augmentedNormals, rhoTimesN, colors);

    // store image data
    _all_rhoTimesN.at(channel).push_back(rhoTimesN);
    _all_pictureChannel.at(channel).push_back(colors);
  }
}

void LighthingEstimator::estimateLigthing(LightingVector& lighting) const
{
  int nbChannels = 3;

  // check number of channels
  for(int channel = 0; channel < 3; ++channel)
  {
    if(_all_rhoTimesN.at(channel).empty())
    {
      nbChannels = channel;
      break;
    }
  }

  // for each channel
  for(int channel = 0; channel < nbChannels; ++channel)
  {
    std::size_t nbRows_all_rhoTimesN = 0;
    std::size_t nbRows_all_pictureChannel = 0;

    // count and check matrices rows
    {
      for(const MatrixXf& matrix : _all_rhoTimesN.at(channel))
        nbRows_all_rhoTimesN += matrix.rows();

      for(const MatrixXf& matrix : _all_pictureChannel.at(channel))
        nbRows_all_pictureChannel += matrix.rows();

      assert(nbRows_all_rhoTimesN == nbRows_all_pictureChannel);
    }

    // agglomerate rhoTimesN
    MatrixXf rhoTimesN(nbRows_all_rhoTimesN, 9);
    {
      std::size_t nbRow = 0;
      for(const MatrixXf& matrix : _all_rhoTimesN.at(channel))
      {
        for(int matrixRow = 0; matrixRow < matrix.rows(); ++matrixRow)
          rhoTimesN.row(nbRow + matrixRow) = matrix.row(matrixRow);
        nbRow += matrix.rows();
      }
    }

    // agglomerate pictureChannel
    MatrixXf pictureChannel(nbRows_all_pictureChannel, 1);
    {
      std::size_t nbRow = 0;
      for(const MatrixXf& matrix : _all_pictureChannel.at(channel))
      {
        for(int matrixRow = 0; matrixRow < matrix.rows(); ++matrixRow)
          pictureChannel.row(nbRow + matrixRow) = matrix.row(matrixRow);
        nbRow += matrix.rows();
      }
    }

    ALICEVISION_LOG_INFO("estimate ligthing channel: rhoTimesN(" << rhoTimesN.rows() << "x" << rhoTimesN.cols()<< ")");
    Eigen::Matrix<float, 9, 1> lightingC = rhoTimesN.colPivHouseholderQr().solve(pictureChannel);

    // lighting vectors fusion
    lighting.col(channel) = lightingC;

    // luminance estimation
    if(nbChannels == 1)
    {
      lighting.col(1) = lightingC;
      lighting.col(2) = lightingC;
    }
  }
}

void LighthingEstimator::clear()
{
  _all_rhoTimesN = std::array<std::vector<MatrixXf>, 3>();
  _all_pictureChannel = std::array<std::vector<MatrixXf>, 3>();
}

} // namespace lightingEstimation
} // namespace aliceVision
