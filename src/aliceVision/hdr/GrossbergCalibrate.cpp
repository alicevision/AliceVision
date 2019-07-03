// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "GrossbergCalibrate.hpp"
#include <iostream>
#include <cassert>
#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/system/Logger.hpp>
#include <Eigen/Dense>


namespace aliceVision {
namespace hdr {


GrossbergCalibrate::GrossbergCalibrate(const unsigned int dimension)
{
    _dimension = dimension;
}

void GrossbergCalibrate::process(const std::vector< std::vector< image::Image<image::RGBfColor> > > &ldrImageGroups,
                                 const std::size_t channelQuantization,
                                 const std::vector< std::vector<float> > &times,
                                 const int nbPoints,
                                 const rgbCurve &weight,
                                 rgbCurve &response)
{

    //checks
    for (int g = 0; g < ldrImageGroups.size(); ++g)
    {
      assert(ldrImageGroups[g].size() == times[g].size());
    }

    //set channels count always RGB
    static const std::size_t channels = 3;

    //initialize response with g0 from invEmor
    response = rgbCurve(channelQuantization);
    response.setEmor();
//    response.write("/s/prods/mvg/_source_global/samples/HDR_selection/samsung/8bits/emor.ods");
//    const double* ptrf0 = getEmorInvCurve(0);
//    std::vector<double> f0;
//    f0.assign(ptrf0, ptrf0 + channelQuantization);

    const std::size_t emorSize = std::pow(2, 10);

    Mat H(channelQuantization, _dimension);
    std::vector<std::vector<double>> hCurves(_dimension);

    for(unsigned int i=0; i<_dimension; ++i)
    {
        const double *h = getEmorInvCurve(i+1);
        if(emorSize == channelQuantization)
        {
          hCurves[i].assign(h, h + emorSize);
        }
        else if(emorSize > channelQuantization)
        {
          std::vector<double> emorH;
          emorH.assign(h, h + emorSize);
          std::vector<double> h0 = std::vector<double>(emorH.begin(), emorH.end());

          std::size_t step = emorSize/channelQuantization;
          for(std::size_t k = 0; k<channelQuantization; ++k)
            hCurves[i].emplace_back(h0.at(k*step));
        }
        else
        {
          std::vector<double> emorH;
          emorH.assign(h, h + emorSize);
          std::vector<double> h0 = std::vector<double>(emorH.begin(), emorH.end());

          std::size_t step = channelQuantization/emorSize;
          hCurves[i].resize(channelQuantization, 0.0);
          for(std::size_t k = 0; k<emorSize-1; ++k)
          {
            hCurves[i].at(k*step) = h0.at(k);
          }
          hCurves[i].at(emorSize*step-1) = h0.at(emorSize-1);
          std::size_t previousValidIndex = 0;
          for(std::size_t index = 1; index<channelQuantization; ++index)
          {
              if(hCurves[i].at(index) != 0.0f)
              {
                  if(previousValidIndex+1 < index)
                  {
                      const float inter = (hCurves[i].at(index) - hCurves[i].at(previousValidIndex)) / (index - previousValidIndex);
                      for(std::size_t j = previousValidIndex+1; j < index; ++j)
                      {
                          hCurves[i].at(j) = hCurves[i].at(previousValidIndex) + inter * (j-previousValidIndex);
                      }
                  }
                  previousValidIndex = index;
              }
            }
        }
        H.col(i) = Eigen::Map<Vec>(hCurves[i].data(), channelQuantization);
    }

    for(unsigned int g=0; g<ldrImageGroups.size(); ++g)
    {
        const std::vector< image::Image<image::RGBfColor> > &ldrImagesGroup = ldrImageGroups.at(g);
      const std::vector<float> &ldrTimes= times.at(g);

      const int nbImages = ldrImagesGroup.size();
        const int nbPixels = ldrImagesGroup.at(0).Width() * ldrImagesGroup.at(0).Height();
        const int step = std::floor(nbPixels/nbPoints);


      ALICEVISION_LOG_TRACE("filling A and b matrices");
        for(unsigned int channel=0; channel<channels; ++channel)
        {

            Mat A = Mat::Zero(nbPoints*(nbImages-1), _dimension);
            Vec b = Vec::Zero(nbPoints*(nbImages-1));

//            ALICEVISION_LOG_TRACE("filling A and b matrices");

          for(unsigned int j=0; j<nbImages-1; ++j)
          {
            const image::Image<image::RGBfColor> &image1 = ldrImagesGroup.at(j);
                const image::Image<image::RGBfColor> &image2 = ldrImagesGroup.at(j+1);
                const double k = ldrTimes.at(j+1)/ldrTimes.at(j);

                // fill A and b matrices with the equations
                for(unsigned int l=0; l<nbPoints; ++l)
                {
                    double sample1 = std::max(0.f, std::min(1.f, image1(step*l)(channel)));
                    double sample2 = std::max(0.f, std::min(1.f, image2(step*l)(channel)));


                    std::size_t index1 = std::round((channelQuantization-1) * sample1);
                    std::size_t index2 = std::round((channelQuantization-1) * sample2);

//                    double w = std::min(weight(sample1, channel), weight(sample2, channel));

                    double w = (weight(sample1, channel) + weight(sample2, channel)) / 2.0;

//                    double w_R = (weight(sample1, 0) + weight(sample2, 0)) / 2.0;
//                    double w_G = (weight(sample1, 1) + weight(sample2, 1)) / 2.0;
//                    double w_B = (weight(sample1, 2) + weight(sample2, 2)) / 2.0;

//                    double w = (w_R + w_G + w_B) / 3.0;
//                    double w = (w_R * w_G * w_B);
//                    double w = std::min(std::min(w_R, w_G), w_B);

                    b(j*nbPoints + l) = w * (f0.at(index2) - k * f0.at(index1));
                    for(unsigned int i=0; i<_dimension; ++i)
                      A(j*nbPoints + l, i) = w * (k * H(index1, i) - H(index2, i));

            }
        }

            ALICEVISION_LOG_TRACE("solving Ax=b system");

            // solve the system using QR decomposition
            Eigen::HouseholderQR<Mat> solver(A);
            Vec c = solver.solve(b);

            ALICEVISION_LOG_TRACE("system solved");

            double relative_error = (A*c - b).norm() / b.norm();
            ALICEVISION_LOG_DEBUG("relative error is : " << relative_error);

            ALICEVISION_LOG_DEBUG("emor coefficients are : ");

            for(unsigned int i=0; i<_dimension; ++i)
            {
              std::vector<double> temp_hCurve = hCurves[i];
              for(auto &value : temp_hCurve)
                value *= c(i);

              ALICEVISION_LOG_DEBUG("emor coefficients : ");
              ALICEVISION_LOG_DEBUG(c(i));

                std::transform(response.getCurve(channel).begin(), response.getCurve(channel).end(), temp_hCurve.begin(), response.getCurve(channel).begin(), std::plus<float>());
            }
        }
    }
}


} // namespace hdr
} // namespace aliceVision
