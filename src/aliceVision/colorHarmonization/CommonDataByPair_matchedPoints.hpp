// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include "aliceVision/colorHarmonization/CommonDataByPair.hpp"
#include "aliceVision/matching/indMatch.hpp"
#include "aliceVision/feature/feature.hpp"

#include <vector>

namespace aliceVision {
namespace colorHarmonization {

class CommonDataByPair_matchedPoints  : public CommonDataByPair
{
public:
  CommonDataByPair_matchedPoints(const std::string& sLeftImage,
                                 const std::string& sRightImage,
                                 const matching::MatchesPerDescType& matchesPerDesc,
                                 const feature::MapRegionsPerDesc& regionsL,
                                 const feature::MapRegionsPerDesc& regionsR,
                                 const size_t radius = 1 )
     : CommonDataByPair( sLeftImage, sRightImage )
     , _matchesPerDesc( matchesPerDesc )
     , _regionsL( regionsL )
     , _regionsR( regionsR )
     , _radius( radius )
  {}

  virtual ~CommonDataByPair_matchedPoints()
  {}

  /**
   * Fill mask from corresponding points (each point pictured by a disk of radius _radius)
   *
   * \param[out] maskLeft Mask of the left image (initialized to corresponding image size).
   * \param[out] maskRight  Mask of the right image  (initialized to corresponding image size).
   *
   * \return True if some pixel have been set to true.
   */
  virtual bool computeMask( image::Image< unsigned char > & maskLeft, image::Image< unsigned char > & maskRight )
  {
    maskLeft.fill(0);
    maskRight.fill(0);
    for(const auto& matchesPerDescIt : _matchesPerDesc) //< loop over descType
    {
      const feature::EImageDescriberType descType = matchesPerDescIt.first;

      for(const matching::IndMatch& match : matchesPerDescIt.second) //< loop over matches
      {
        const feature::SIOPointFeature& L = feature::getSIOPointFeatures(*_regionsL.at(descType)).at(match._i);
        const feature::SIOPointFeature& R = feature::getSIOPointFeatures(*_regionsR.at(descType)).at(match._j);

        image::FilledCircle( L.x(), L.y(), ( int )_radius, ( unsigned char ) 255, &maskLeft );
        image::FilledCircle( R.x(), R.y(), ( int )_radius, ( unsigned char ) 255, &maskRight );
      }

    }
    return _matchesPerDesc.getNbAllMatches() > 0;
  }

private:
  size_t _radius;
  matching::MatchesPerDescType _matchesPerDesc;
  const feature::MapRegionsPerDesc& _regionsL;
  const feature::MapRegionsPerDesc& _regionsR;
};

}  // namespace colorHarmonization
}  // namespace aliceVision
