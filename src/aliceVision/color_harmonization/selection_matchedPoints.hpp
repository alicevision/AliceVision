// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef ALICEVISION_COLORHARMONIZATION_MATCHESPOINTS_H
#define ALICEVISION_COLORHARMONIZATION_MATCHESPOINTS_H

#include "aliceVision/color_harmonization/selection_interface.hpp"
#include "aliceVision/matching/indMatch.hpp"
#include "aliceVision/features/features.hpp"

#include <vector>

namespace aliceVision {
namespace color_harmonization {

class commonDataByPair_MatchedPoints  : public commonDataByPair
{
public:
  commonDataByPair_MatchedPoints(const std::string& sLeftImage,
                                 const std::string& sRightImage,
                                 const matching::MatchesPerDescType& matchesPerDesc,
                                 const features::MapRegionsPerDesc& regionsL,
                                 const features::MapRegionsPerDesc& regionsR,
                                 const size_t radius = 1 )
     : commonDataByPair( sLeftImage, sRightImage )
     , _matchesPerDesc( matchesPerDesc )
     , _regionsL( regionsL )
     , _regionsR( regionsR )
     , _radius( radius )
  {}

  virtual ~commonDataByPair_MatchedPoints()
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
      const features::EImageDescriberType descType = matchesPerDescIt.first;

      for(const matching::IndMatch& match : matchesPerDescIt.second) //< loop over matches
      {
        const features::SIOPointFeature& L = features::getSIOPointFeatures(*_regionsL.at(descType)).at(match._i);
        const features::SIOPointFeature& R = features::getSIOPointFeatures(*_regionsR.at(descType)).at(match._j);

        image::FilledCircle( L.x(), L.y(), ( int )_radius, ( unsigned char ) 255, &maskLeft );
        image::FilledCircle( R.x(), R.y(), ( int )_radius, ( unsigned char ) 255, &maskRight );
      }

    }
    return _matchesPerDesc.getNbAllMatches() > 0;
  }

private:
  size_t _radius;
  matching::MatchesPerDescType _matchesPerDesc;
  const features::MapRegionsPerDesc& _regionsL;
  const features::MapRegionsPerDesc& _regionsR;
};

}  // namespace color_harmonization
}  // namespace aliceVision

#endif  // ALICEVISION_COLORHARMONIZATION_MATCHESPOINTS_H
