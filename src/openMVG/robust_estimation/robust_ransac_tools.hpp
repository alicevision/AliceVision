// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef OPENMVG_ROBUST_RANSAC_TOOLS_H_
#define OPENMVG_ROBUST_RANSAC_TOOLS_H_

#include <cmath>

namespace openMVG {
namespace robust{

/// Number of samplings to have at least \a minProba probability of absence of
/// outlier in a sample of \a SampleSize elements.
inline size_t getNumSamples(
  double minProba,
  double outlierRatio,
  std::size_t SampleSize)
{
  return static_cast<std::size_t>( std::log(1.-minProba) /
    std::log(1.-std::pow(1.-outlierRatio, static_cast<int>(SampleSize))));
}

inline size_t IterationsRequired(
  std::size_t min_samples,
  double outliers_probability,
  double inlier_ratio)
{
  return static_cast<std::size_t>(
    std::log(outliers_probability) /
    std::log(1.0 - std::pow(inlier_ratio, static_cast<int>(min_samples))));
}

} // namespace robust
} // namespace openMVG
#endif // OPENMVG_ROBUST_RANSAC_TOOLS_H_
