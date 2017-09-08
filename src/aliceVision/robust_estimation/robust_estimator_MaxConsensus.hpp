// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef ALICEVISION_ROBUST_ESTIMATION_MAX_CONSENSUS_H_
#define ALICEVISION_ROBUST_ESTIMATION_MAX_CONSENSUS_H_

#include "aliceVision/robust_estimation/rand_sampling.hpp"
#include <limits>
#include <vector>

namespace aliceVision {
namespace robust{

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
/// 2. Kernel::MINIMUM_SAMPLES
/// 3. Kernel::Fit(vector<int>, vector<Kernel::Model> *)
/// 4. Kernel::Error(Model, int) -> error
template<typename Kernel, typename Scorer>
typename Kernel::Model MaxConsensus(const Kernel &kernel,
  const Scorer &scorer,
  std::vector<std::size_t> *best_inliers = nullptr, std::size_t max_iteration = 1024) {

    const std::size_t min_samples = Kernel::MINIMUM_SAMPLES;
    const std::size_t total_samples = kernel.NumSamples();

    std::size_t best_num_inliers = 0;
    typename Kernel::Model best_model;

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
      UniformSample(min_samples, total_samples, sample);

      std::vector<typename Kernel::Model> models;
      kernel.Fit(sample, &models);

      // Compute costs for each fit.
      for(std::size_t i = 0; i < models.size(); ++i) 
      {
        std::vector<std::size_t> inliers;
        scorer.Score(kernel, models[i], all_samples, &inliers);

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

} // namespace robust
} // namespace aliceVision
#endif // ALICEVISION_ROBUST_ESTIMATION_MAX_CONSENSUS_H_
