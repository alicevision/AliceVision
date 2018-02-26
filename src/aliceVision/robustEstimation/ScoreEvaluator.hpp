// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {
namespace robustEstimation{

using namespace std;

/// Templated Functor class to evaluate a given model over a set of samples.
template<typename Kernel>
class ScoreEvaluator {
public:
  ScoreEvaluator(double threshold) : threshold_(threshold) {}

  template <typename T>
  double Score(const Kernel &kernel,
               const typename Kernel::Model &model,
               const std::vector<T> &samples,
               std::vector<T> *inliers,
               double threshold) const
  {
    double cost = 0.0;
    for (size_t j = 0; j < samples.size(); ++j) 
    {
      double error = kernel.Error(samples[j], model);
      if (error < threshold) 
      {
        cost += error;
        inliers->push_back(samples[j]);
      } 
      else 
      {
//        cost += threshold;
        cost += error;
      }
    }
    return cost;
  }

  template <typename T>
  double Score(const Kernel &kernel,
               const typename Kernel::Model &model,
               const std::vector<T> &samples,
               std::vector<T> *inliers) const
  {
    return Score(kernel, model, samples, inliers, threshold_);
  }
  
  double getThreshold() const {return threshold_;} 
  
private:
  double threshold_;
};

} // namespace robustEstimation
} // namespace aliceVision
