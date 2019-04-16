// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2007 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <cmath>

namespace aliceVision {
namespace robustEstimation{

/**
 * @brief Number of samplings to have at least \a minProba probability of absence of
 *        outlier in a sample of \a sampleSize elements.
 * @param minProba
 * @param outlierRatio
 * @param SampleSize
 * @return
 */
inline std::size_t getNumSamples(double minProba, double outlierRatio, std::size_t sampleSize)
{
  return static_cast<std::size_t>(std::log(1.-minProba) / std::log(1.-std::pow(1.-outlierRatio, static_cast<int>(sampleSize))));
}

inline std::size_t iterationsRequired(std::size_t min_samples, double outliersProbability, double inlierRatio)
{
  return static_cast<std::size_t>(std::log(outliersProbability) / std::log(1.0 - std::pow(inlierRatio, static_cast<int>(min_samples))));
}

} // namespace robustEstimation
} // namespace aliceVision
