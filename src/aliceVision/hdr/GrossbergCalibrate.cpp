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
#include "sampling.hpp"
#include "QuadProg++.hh"

namespace aliceVision
{
namespace hdr
{

GrossbergCalibrate::GrossbergCalibrate(const unsigned int dimension)
{
    _dimension = dimension;
}

void GrossbergCalibrate::process(const std::vector<std::vector<std::string>>& imagePathsGroups,
                                 const std::size_t channelQuantization, const std::vector<std::vector<float>>& times,
                                 const int nbPoints, const bool fisheye, rgbCurve& response)
{
    const int nbGroups = imagePathsGroups.size();
    const int nbImages = imagePathsGroups.front().size();
    const int samplesPerImage = nbPoints / (nbGroups * nbImages);
    const double step = 1.0 / double(channelQuantization - 1);

    ALICEVISION_LOG_DEBUG("Extract color samples");
    std::vector<std::vector<ImageSamples>> samples;
    extractSamples(samples, imagePathsGroups, times, nbPoints, 1.0, fisheye);

    // set channels count always RGB
    static const std::size_t channels = 3;

    // initialize response with g0 from invEmor
    response = rgbCurve(channelQuantization);
    response.setEmor(3);

    const std::size_t emorSize = std::pow(2, 10);
    if (channelQuantization !=emorSize) {
      ALICEVISION_LOG_ERROR("Incompatible channel quantization");
      return;
    }

    //finv(f(e1*E)) = finv(f(e2*E))
    //g(Ba) - k*g(Bb) = 0
    //f0(Ba) + sum(c_i * f_i(Ba)) - k*f0(Bb) - k*sum(c_i * f_i(Bb)) = 0
    //sum(c_i * f_i(Ba)) - k*sum(c_i * f_i(Bb)) = k*f0(Bb) - f0(Ba)

    size_t count_measures = 0;
    for (size_t group = 0; group < samples.size(); group++) {
      size_t groupsize = samples[group].size();
      count_measures += (groupsize - 1) * samples[group][0].colors.size();
    }
           

    for (int channel = 0; channel < 3; channel++) {
      Eigen::MatrixXd E(count_measures, _dimension);
      Eigen::MatrixXd v(count_measures, 1);

      rgbCurve f0(channelQuantization);
      f0.setEmor(0);

      for (size_t dim = 0; dim < _dimension; dim++) {

        rgbCurve fdim(channelQuantization);
        fdim.setEmor(dim + 1);

        size_t rowId = 0;
        for (size_t groupId = 0; groupId < samples.size(); groupId++) {

          std::vector<ImageSamples> & group = samples[groupId];
          

          for (size_t bracketId = 0; bracketId < group.size() - 1; bracketId++) {
            
            ImageSamples & bracket_cur = group[bracketId];
            ImageSamples & bracket_next = group[bracketId + 1];

            double k = bracket_cur.exposure / bracket_next.exposure;
            
            for (size_t sampleId = 0; sampleId < bracket_cur.colors.size(); sampleId++) {

              image::Rgb<double> Ba = bracket_cur.colors[sampleId];
              image::Rgb<double> Bb = bracket_next.colors[sampleId];

              float valA = Ba(channel); 
              float valB = Bb(channel);

              
              valA /= 0.95;
              valB /= 0.95;

              if (valB > 1.0) {
                E(rowId, dim) = 0;
                v(rowId, 0) = 0;
                rowId++;

                continue;
              }

              E(rowId, dim) = fdim(valA, 0) - k * fdim(valB, 0);
              v(rowId, 0) = f0(valA, 0) - k * f0(valB, 0);
              rowId++;
            }
          }
        }
      }

     

      /* Get first linear solution */
      Eigen::VectorXd c = (E.transpose() * E).inverse() * E.transpose() * -v; 
      Eigen::MatrixXd H = E.transpose() * E;
      Eigen::VectorXd d = (E.transpose() * v).col(0);

      /**
       * d (f0(val) + sum_i(c_i * f_i(val))) d_val > 0
       * d (f0(val)) + sum_i(d(c_i * f_i(val))) > 0
       * d (f0(val)) + sum_i(c_i * d_f_i(val)) > 0 
       * 
       * f(x) ~ f(x+1) - f(x)
       * d (f0(val)) + sum_i(c_i * f(val + 1) - c_i * f(val)) > 0 
       */
      Eigen::MatrixXd dF0(channelQuantization - 1, 1);
      dF0.setZero();
      for (int i = 0; i < channelQuantization - 1; i++) {
        double eval_cur = double(i) * step;
        double eval_next = double(i + 1) * step;
        dF0(i, 0)  = f0(eval_next, channel) - f0(eval_cur, channel);
      }
      
      Eigen::MatrixXd D(channelQuantization - 1, _dimension);
      D.setZero();

      for (int dim = 0; dim < _dimension; dim++) {
        rgbCurve fdim(channelQuantization);
        fdim.setEmor(dim + 1);

        for (int i = 0; i < channelQuantization - 1; i++) {
          double eval_cur = double(i) * step;
          double eval_next = double(i + 1) * step;
          D(i, dim)  = fdim(eval_next, channel) - fdim(eval_cur, channel);
        }
      }
   

      Eigen::MatrixXd CE(_dimension, 1);
      for (int i = 0; i < 1; i++) {
        for (int j = 0; j < _dimension; j++) {
          rgbCurve fdim(channelQuantization);
          fdim.setEmor(j + 1);
          CE(j, i) = fdim(1.0, channel);
        }
      }

      Eigen::VectorXd ce0(1);
      for (int i = 0; i < 1; i++) {
        ce0[i] = 0;
      }
      
      quadprogpp::solve_quadprog(H, d, CE, ce0, D.transpose(), dF0, c);
      

      /*
      Create final curve
      */
      std::vector<float>& curve = response.getCurve(channel);
      for(unsigned int i = 0; i < curve.size(); ++i)
      {
        rgbCurve f0(channelQuantization);
        f0.setEmor(0);

        double val = double(i) * step;
        val = val / 0.95;
        if (val > 1.0) {
          val = 1.0;
        }
            
        double curve_val = f0(val, channel);
        for (int d = 0; d < _dimension; d++) {

          rgbCurve fdim(channelQuantization);
          fdim.setEmor(d + 1);

          double val = double(i) * step;
          val /= 0.95;

          if (val > 1.0) {
            val = 1.0;
          }
          curve_val += c(d) * fdim(val, channel);
        }

        curve[i] = curve_val;
      }
    }
}

} // namespace hdr
} // namespace aliceVision
