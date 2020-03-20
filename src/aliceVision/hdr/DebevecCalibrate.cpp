// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "DebevecCalibrate.hpp"
#include <iostream>
#include <fstream>
#include <cassert>
#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/image/all.hpp>
#include <aliceVision/image/io.hpp>
#include <OpenImageIO/imagebufalgo.h>
#include "sampling.hpp"


namespace aliceVision {
namespace hdr {

using T = Eigen::Triplet<double>;

bool DebevecCalibrate::process(const std::vector< std::vector<std::string>> & imagePathsGroups,
                               const std::size_t channelQuantization,
                               const std::vector<std::vector<float> > &times,
                               const int nbPoints,
                               const int calibrationDownscale,
                               const bool fisheye,
                               const rgbCurve &weight,
                               const float lambda,
                               rgbCurve &response)
{
  const int nbGroups = imagePathsGroups.size();
  const int nbImages = imagePathsGroups.front().size();
  const int samplesPerImage = nbPoints / (nbGroups*nbImages);

  // Always 3 channels for the input images
  static const std::size_t channelsCount = 3;

  /*Extract samples*/
  ALICEVISION_LOG_DEBUG("Extract color samples");
  std::vector<std::vector<ImageSamples>> samples;
  extractSamples(samples, imagePathsGroups, times, nbPoints, calibrationDownscale, fisheye);

  // Initialize response
  response = rgbCurve(channelQuantization);

  // Store intermediate data for all three channels
  Vec b_array[channelsCount];
  std::vector<T> tripletList_array[channelsCount];

  // Initialize intermediate buffers
  for(unsigned int channel=0; channel < channelsCount; ++channel)
  {
    Vec & b = b_array[channel];
    b = Vec::Zero(nbPoints + channelQuantization + 1);
    std::vector<T> & tripletList = tripletList_array[channel];
    tripletList.reserve(2 * nbPoints + 1 + 3 * channelQuantization);
  }

  size_t count = 0;
  for (size_t groupId = 0; groupId < samples.size(); groupId++) {
    
    std::vector<ImageSamples> & group = samples[groupId];
          
    for (size_t bracketId = 0; bracketId < group.size() - 1; bracketId++) {
            
      ImageSamples & bracket_cur = group[bracketId];
      
      for (size_t sampleId = 0; sampleId < bracket_cur.colors.size(); sampleId++) {
        
        for (int channel = 0; channel < channelsCount; channel++) {
          float sample = bracket_cur.colors[sampleId](channel);
          float w_ij = weight(sample, channel);
          const float time = std::log(bracket_cur.exposure);
          std::size_t index = std::round(sample * (channelQuantization - 1));

          tripletList_array[channel].push_back(T(count, index, w_ij));
          tripletList_array[channel].push_back(T(count, channelQuantization + groupId * samplesPerImage + sampleId, -w_ij));
          b_array[channel](count) = w_ij * time;
        }

        count++;
      }
    }
  }       

  // fix the curve by setting its middle value to zero
  for (int channel = 0; channel < channelsCount; channel++)
  {
    tripletList_array[channel].push_back(T(count, std::floor(channelQuantization/2), 1.f));
  }
  count += 1;

  // include the smoothness equations
  for(std::size_t k = 0; k < channelQuantization - 2; k++)
  {
    for (int channel = 0; channel < channelsCount; channel++)
    {
      float w = weight.getValue(k + 1, channel);
      tripletList_array[channel].push_back(T(count, k, lambda * w));
      tripletList_array[channel].push_back(T(count, k + 1, - 2.f * lambda * w));
      tripletList_array[channel].push_back(T(count, k + 2, lambda * w));
    }

    count++;
  }


  for (int channel = 0; channel < channelsCount; channel ++)
  {
    sMat A(count, channelQuantization + samplesPerImage * nbGroups);
    A.setFromTriplets(tripletList_array[channel].begin(), tripletList_array[channel].end());
    b_array[channel].conservativeResize(count);

    // solve the system using SVD decomposition
    A.makeCompressed();
    Eigen::SparseQR<sMat, Eigen::COLAMDOrdering<int>> solver;
    solver.compute(A);

    // Check solver failure
    if (solver.info() != Eigen::Success)
    {
      return false;
    }

    Vec x = solver.solve(b_array[channel]);

    // Check solver failure
    if(solver.info() != Eigen::Success)
    {
      return false;
    }

    double relative_error = (A*x - b_array[channel]).norm() / b_array[channel].norm();

    // Copy the result to the response curve
    for(std::size_t k = 0; k < channelQuantization; ++k)
    {
      response.setValue(k, channel, x(k));
    }
  }

  return true;
}

} // namespace hdr
} // namespace aliceVision
