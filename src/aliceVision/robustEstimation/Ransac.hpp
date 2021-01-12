// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/system/Logger.hpp>
#include "aliceVision/robustEstimation/randSampling.hpp"
#include "aliceVision/robustEstimation/ransacTools.hpp"
#include <limits>
#include <numeric>
#include <vector>
#include <iostream>
#include <iterator>

namespace aliceVision {
namespace robustEstimation{

/// \brief The famous Random Sample Consensus algorithm (Fischler&Bolles 1981).
/// \details The number of tests is reevaluated down as soon as more inliers are
/// found. Return the found model.
// 1. The model.
// 2. The minimum number of samples needed to fit.
// 3. A way to convert samples to a model.
// 4. A way to convert samples and a model to an error.
//
// 1. Kernel::ModelT
// 2. Kernel::getMinimumNbRequiredSamples()
// 3. Kernel::fit(vector<int>, vector<Kernel::Model> *)
// 4. Kernel::error(Model, int) -> error
template<typename Kernel, typename Scorer>
typename Kernel::ModelT RANSAC(
  const Kernel& kernel,
  const Scorer& scorer,
  std::mt19937 & randomNumberGenerator,
  std::vector<std::size_t>* best_inliers = nullptr,
  double* best_score = nullptr,
  bool bVerbose = true,
  double outliers_probability = 1e-2)
{
  assert(outliers_probability < 1.0);
  assert(outliers_probability > 0.0);
  size_t iteration = 0;
  const size_t min_samples = kernel.getMinimumNbRequiredSamples();
  const size_t total_samples = kernel.nbSamples();

  size_t max_iterations = 100;
  const size_t really_max_iterations = 4096;

  size_t best_num_inliers = 0;
  double best_inlier_ratio = 0.0;
  typename Kernel::ModelT best_model;

  // Test if we have sufficient points for the kernel.
  if (total_samples < min_samples) 
  {
    if (best_inliers) {
      best_inliers->resize(0);
    }
    return best_model;
  }

  // In this robust estimator, the scorer always works on all the data points
  // at once. So precompute the list ahead of time [0,..,total_samples].
  std::vector<size_t> all_samples(total_samples);
  std::iota(all_samples.begin(), all_samples.end(), 0);

  for (iteration = 0;
    iteration < max_iterations &&
    iteration < really_max_iterations; ++iteration) 
  {
      std::vector<size_t> sample;
      uniformSample(randomNumberGenerator, min_samples, total_samples, sample);

      std::vector<typename Kernel::ModelT> models;
      kernel.fit(sample, models);

      // Compute the inlier list for each fit.
      for (size_t i = 0; i < models.size(); ++i) 
      {
        std::vector<size_t> inliers;
        scorer.score(kernel, models[i], all_samples, inliers);

        if (best_num_inliers < inliers.size()) 
        {
          best_num_inliers = inliers.size();
          best_inlier_ratio = inliers.size() / double(total_samples);
          best_model = models[i];
          if (best_inliers) 
          {
            best_inliers->swap(inliers);
          }
          if(bVerbose)
          {
            ALICEVISION_LOG_DEBUG("inliers=" << best_num_inliers << "/" << total_samples
                      << " (iter=" << iteration
                      << ", sample=" << sample
                      << ")");
        }
          if (best_inlier_ratio) 
          {
          max_iterations = iterationsRequired(min_samples,
            outliers_probability,
            best_inlier_ratio);
            if(bVerbose)
              ALICEVISION_LOG_DEBUG("New max_iteration: " << max_iterations);
          }
        }
      }
  }
  if (best_score)
    *best_score = best_num_inliers;
  
  if(best_num_inliers)
    kernel.unnormalize(best_model);
  
  return best_model;
}


} // namespace robustEstimation
} // namespace aliceVision
