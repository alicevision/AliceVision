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

namespace aliceVision {
namespace hdr {

using T = Eigen::Triplet<double>;

void DebevecCalibrate::process(const std::vector< std::vector< image::Image<image::RGBfColor> > > &ldrImageGroups,
                               const std::size_t channelQuantization,
                               const std::vector< std::vector<float> > &times,
                               const int nbPoints,
                               const rgbCurve &weight,
                               const float lambda,
                               rgbCurve &response)
{
  //checks
  for (int g = 0; g < ldrImageGroups.size(); ++g)
  {
    assert(ldrImageGroups[g].size() == times[g].size());
  }

  //set channels count always RGB
  static const std::size_t channels = 3;

  //initialize response
  response = rgbCurve(channelQuantization);
  response.setLinear();
  response.normalize();

  for(unsigned int g=0; g<ldrImageGroups.size(); ++g)
  {
    const std::vector< image::Image<image::RGBfColor> > &ldrImagesGroup = ldrImageGroups.at(g);
    const std::vector<float> &ldrTimes = times.at(g);
    const int nbImages = ldrImagesGroup.size();
    const int step = std::floor(ldrImagesGroup.at(0).Width() * ldrImagesGroup.at(0).Height() / nbPoints);

    for(unsigned int channel=0; channel<channels; ++channel)
    {
      sMat A(nbPoints*nbImages + channelQuantization + 1, channelQuantization + nbPoints);
      Vec b = Vec::Zero(nbPoints*nbImages + channelQuantization + 1);
      int count=0;

      std::vector<T> tripletList;
      tripletList.reserve(2 * nbPoints*nbImages + 1 + 3 * channelQuantization);

//      ALICEVISION_LOG_TRACE("filling A and b matrices");

      // include the data-fitting equations
      for(unsigned int j=0; j<nbImages; ++j)
      {
        const image::Image<image::RGBfColor> &image = ldrImagesGroup.at(j);
        for(unsigned int i=0; i<nbPoints; ++i)
        {
          float sample = std::max(0.f, std::min(1.f, image(step*i)(channel)));
          float w_ij = weight(sample, channel);
          std::size_t index = std::round(sample * (channelQuantization - 1));

          tripletList.push_back(T(count, index, w_ij));
          tripletList.push_back(T(count, channelQuantization+i, -w_ij));

          b(count) = w_ij * std::log(ldrTimes.at(j));
          count += 1;
        }
      }

      // fix the curve by setting its middle value to zero
      tripletList.push_back(T(count, std::floor(channelQuantization/2), 1.f));
      count += 1;

      // include the smoothness equations
      for(std::size_t k=0; k<channelQuantization-2; ++k)
      {
        float w = weight.getValue(k+1, channel);

        tripletList.push_back(T(count, k, lambda * w));
        tripletList.push_back(T(count, k+1, -2.f * lambda * w));
        tripletList.push_back(T(count, k+2, lambda * w));

        count += 1;
      }

      A.setFromTriplets(tripletList.begin(), tripletList.end());



      // solve the system using SVD decomposition
      A.makeCompressed();
      Eigen::SparseQR<sMat, Eigen::COLAMDOrdering<int>> solver;
      solver.compute(A);
      if(solver.info() != Eigen::Success)  return; // decomposition failed
      Vec x = solver.solve(b);
      if(solver.info() != Eigen::Success)  return; // solving failed

//      ALICEVISION_LOG_TRACE("system solved");

//      double relative_error = (A*x - b).norm() / b.norm();
//      ALICEVISION_LOG_TRACE("relative error is : " << relative_error);

      for(std::size_t k=0; k<channelQuantization; ++k)
        response.setValue(k, channel, x(k));

    }
  }
}

} // namespace hdr
} // namespace aliceVision
