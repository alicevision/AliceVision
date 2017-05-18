#pragma once

namespace openMVG {
namespace matching_image_collection {


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
    const sfm::SfM_Data * sfmData,
    const features::RegionsPerView& regionsPerView,
    const Pair imageIdsPair,
    const double dDistanceRatio,
    matching::MatchesPerDescType & matches
  ) = 0;


  double m_dPrecision;  //upper_bound precision used for robust estimation
  double m_dPrecision_robust;
  std::size_t m_stIteration; //maximal number of iteration for robust estimation
};

struct EstimationState
{
  EstimationState(bool valid, bool strongSupport)
    : isValid(valid)
    , hasStrongSupport(strongSupport)
  {}

  bool isValid = false;
  bool hasStrongSupport = false;
};

} // namespace matching_image_collection
} // namespace openMVG
