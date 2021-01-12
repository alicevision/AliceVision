// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/robustEstimation/randSampling.hpp"
#include <limits>
#include <vector>

namespace aliceVision {
namespace robustEstimation{

/// Naive implementation of RANSAC without noise and iteration reduction options
/// Pick max_iteration times N_samples and Fit a solution.
/// Keep the solution that have the most of inlier
///  under the confidence threshold of the Scorer.
///
/// Requirements :
/// 1. The model type.
/// 2. The minimum number of samples needed to fit.
/// 3. A way to convert samples to a model.
/// 4. A way to convert samples and a model to an error.
///
/// 1. Kernel::Model
/// 2. Kernel::getMinimumNbRequiredSamples()
/// 3. Kernel::fit(vector<int>, vector<Kernel::Model> *)
/// 4. Kernel::error(Model, int) -> error
template<typename Kernel, typename Scorer>
typename Kernel::ModelT maxConsensus(const Kernel& kernel,
                                     const Scorer& scorer,
                                     std::mt19937 & randomNumberGenerator,
                                     std::vector<std::size_t>* best_inliers = nullptr,
                                     std::size_t max_iteration = 1024) {

    const std::size_t min_samples = kernel.getMinimumNbRequiredSamples();
    const std::size_t total_samples = kernel.nbSamples();

    std::size_t best_num_inliers = 0;
    typename Kernel::ModelT best_model;

    // Test if we have sufficient points to for the kernel.
    if (total_samples < min_samples) 
    {
      if (best_inliers) 
      {
        best_inliers->resize(0);
      }
      return best_model;
    }

    // In this robust estimator, the scorer always works on all the data points
    // at once. So precompute the list ahead of time.
    std::vector<std::size_t> all_samples;
    for(std::size_t i = 0; i < total_samples; ++i)
    {
      all_samples.push_back(i);
    }

    for(std::size_t iteration = 0;  iteration < max_iteration; ++iteration) 
    {
      std::vector<std::size_t> sample;
      uniformSample(randomNumberGenerator, min_samples, total_samples, sample);

      std::vector<typename Kernel::ModelT> models;
      kernel.fit(sample, models);

      // Compute costs for each fit.
      for(std::size_t i = 0; i < models.size(); ++i) 
      {
        std::vector<std::size_t> inliers;
        scorer.score(kernel, models[i], all_samples, inliers);

        if (best_num_inliers < inliers.size())
        {
          //ALICEVISION_LOG_DEBUG("Fit cost: " << cost/inliers.size()
          //  << ", number of inliers: " << inliers.size());
          best_num_inliers = inliers.size();
          best_model = models[i];
          if (best_inliers) 
          {
            best_inliers->swap(inliers);
          }
        }
      }
    }
    return best_model;
}

} // namespace robustEstimation
} // namespace aliceVision
