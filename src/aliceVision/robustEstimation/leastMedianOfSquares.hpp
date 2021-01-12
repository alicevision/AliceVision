// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/robustEstimation/randSampling.hpp>
#include <aliceVision/robustEstimation/ransacTools.hpp>

#include <algorithm>
#include <limits>
#include <vector>

namespace aliceVision {
namespace robustEstimation {

/**
 * @brief Variant of RANSAC using Least Median of Squares.
 *
 * @details Instead of using a fixed threshold to distinguish inlier/outlier,
 *          find the threshold at 1-\a outlierRatio quantile of residuals and keep the
 *          parameters minimizing this threshold. The final threshold
 *          returned in \a outlierThreshold is a multiple of this and can be used to
 *          filter out outliers.
 *
 * @ref LMedS : Z. Zhang. Determining The Epipolar Geometry And Its Uncertainty.
 *      A Review IJCV 1998
 */
template <typename Kernel>
double leastMedianOfSquares(const Kernel& kernel,
                            typename Kernel::ModelT* model = nullptr,
                            double* outlierThreshold = nullptr,
                            double outlierRatio = 0.5,
                            double minProba = 0.99)
{
  const std::size_t min_samples = kernel.getMinimumNbRequiredSamples();
  const std::size_t total_samples = kernel.nbSamples();

  std::vector<double> residuals(total_samples); // Array for storing residuals

  double dBestMedian = std::numeric_limits<double>::max();

  // Required number of iterations is evaluated from outliers ratio
  const std::size_t N = (min_samples < total_samples) ?
          getNumSamples(minProba, outlierRatio, min_samples) : 0;

  for(std::size_t i = 0; i < N; i++)
  {

    std::vector<std::size_t> vec_sample(min_samples);
    // Get Samples indexes
    uniformSample(min_samples, total_samples, vec_sample);

    // Estimate parameters: the solutions are stored in a vector
    std::vector<typename Kernel::ModelT> models;
    kernel.Fit(vec_sample, &models);

    // Now test the solutions on the whole data
    for(std::size_t k = 0; k < models.size(); ++k)
    {
      //Compute Residuals :
      for(std::size_t sampleIdx = 0; sampleIdx < total_samples; ++sampleIdx)
      {
        double error = kernel.Error(sampleIdx, models[k]);
        residuals[sampleIdx] = error;
      }

      // Compute median
      std::vector<double>::iterator itMedian = residuals.begin() +
              std::size_t(total_samples * (1. - outlierRatio));
      std::nth_element(residuals.begin(), itMedian, residuals.end());
      double median = *itMedian;

      // Store best solution
      if(median < dBestMedian)
      {
        dBestMedian = median;
        if(model)
        {
          (*model) = models[k];
        }
      }
    }
  }

  // This array of precomputed values corresponds to the inverse
  //  cumulative function for a normal distribution. For more information
  //  consult the litterature (Robust Regression for Outlier Detection,
  //  rouseeuw-leroy). The values are computed for each 5%
  static const double ICDF[21] = {
    1.4e16, 15.94723940, 7.957896558, 5.287692054,
    3.947153876, 3.138344200, 2.595242369, 2.203797543,
    1.906939402, 1.672911853, 1.482602218, 1.323775627,
    1.188182950, 1.069988721, 0.9648473415, 0.8693011162,
    0.7803041458, 0.6946704675, 0.6079568319, 0.5102134568,
    0.3236002672
  };

  // Evaluate the outlier threshold
  if(outlierThreshold)
  {
    double sigma = ICDF[int((1. - outlierRatio)*20.)] *
            (1. + 5. / double(total_samples - min_samples));
    *outlierThreshold = (double) (sigma * sigma * dBestMedian * 4.);
    if(N == 0)
    {
      *outlierThreshold = std::numeric_limits<double>::max();
    }
  }

  return dBestMedian;
}


} // namespace robustEstimation
} // namespace aliceVision
