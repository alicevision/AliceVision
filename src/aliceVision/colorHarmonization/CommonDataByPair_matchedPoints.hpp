// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/colorHarmonization/CommonDataByPair.hpp"
#include "aliceVision/matching/IndMatch.hpp"
#include "aliceVision/feature/feature.hpp"
#include "aliceVision/feature/RegionsPerView.hpp"

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
        const feature::PointFeature& L = _regionsL.at(descType)->Features().at(match._i);
        const feature::PointFeature& R = _regionsR.at(descType)->Features().at(match._j);

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
