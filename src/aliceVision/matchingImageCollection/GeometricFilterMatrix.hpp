// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

namespace aliceVision {


namespace feature {
class RegionsPerView;
}

namespace matching {
class MatchesPerDescType;
}

namespace sfm {
class SfMData;
}


namespace matchingImageCollection {


struct GeometricFilterMatrix
{
  GeometricFilterMatrix(double precision,
                        double precisionRobust,
                        std::size_t stIteration)
    : m_dPrecision(precision)
    , m_dPrecision_robust(precisionRobust)
    , m_stIteration(stIteration)
  {}

  /**
   * @brief Geometry_guided_matching
   * @param sfm_data
   * @param regionsPerView
   * @param pairIndex
   * @param dDistanceRatio
   * @param matches
   * @return
   */
  virtual bool Geometry_guided_matching
  (
    const sfm::SfMData * sfmData,
    const feature::RegionsPerView& regionsPerView,
    const Pair imageIdsPair,
    const double dDistanceRatio,
    matching::MatchesPerDescType & matches
  ) = 0;


  double m_dPrecision;  //upper_bound precision used for robust estimation
  double m_dPrecision_robust;
  std::size_t m_stIteration; //maximal number of iteration for robust estimation
};


} // namespace matchingImageCollection
} // namespace aliceVision
