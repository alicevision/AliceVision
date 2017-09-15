// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef ALICEVISION_COLORHARMONIZATION_VLDSEGMENT_H
#define ALICEVISION_COLORHARMONIZATION_VLDSEGMENT_H

#include "aliceVision/color_harmonization/selection_interface.hpp"
#include "aliceVision/matching/kvld/kvld.h"
#include "aliceVision/matching/kvld/kvld_draw.h"

namespace aliceVision {
namespace color_harmonization {

class commonDataByPair_VLDSegment  : public commonDataByPair
{
  public:
  commonDataByPair_VLDSegment( const std::string& sLeftImage,
                               const std::string& sRightImage,
                               const matching::IndMatches& matchesPerDesc,
                               const std::vector<feature::SIOPointFeature>& featsL,
                               const std::vector<feature::SIOPointFeature>& featsR)
           : commonDataByPair( sLeftImage, sRightImage )
           , _matches( matchesPerDesc )
           , _featsL( featsL )
           , _featsR( featsR )
  {}

  virtual ~commonDataByPair_VLDSegment()
  {}

  /**
   * Put masks to white, images are conserved
   *
   * \param[out] maskLeft Mask of the left image (initialized to corresponding image size).
   * \param[out] maskRight  Mask of the right image (initialized to corresponding image size).
   *
   * \return True.
   */
  virtual bool computeMask(
    image::Image< unsigned char > & maskLeft,
    image::Image< unsigned char > & maskRight )
  {
    image::Image< unsigned char > imageL, imageR;
    image::ReadImage( _sLeftImage.c_str(), &imageL );
    image::ReadImage( _sRightImage.c_str(), &imageR );

    image::Image< float > imgA ( imageL.GetMat().cast< float >() );
    image::Image< float > imgB(imageR.GetMat().cast< float >());

    std::vector< Pair > matchesFiltered, matchesPair;

    for(const matching::IndMatch& match : _matches)
    {
      matchesPair.emplace_back(match._i, match._j);
    }

    std::vector< double > vec_score;

    //In order to illustrate the gvld(or vld)-consistant neighbors, the following two parameters has been externalized as inputs of the function KVLD.
    // gvld-consistancy matrix, intitialized to -1,  >0 consistancy value, -1=unknow, -2=false
    aliceVision::Mat E = aliceVision::Mat::Ones( _matches.size(), _matches.size() ) * ( -1 );

    // indices of match in the initial matches, if true at the end of KVLD, a match is kept.
    std::vector< bool > valid( _matches.size(), true );

    size_t it_num = 0;
    KvldParameters kvldparameters;//initial parameters of KVLD
    //kvldparameters.K = 5;
    while (
      it_num < 5 &&
      kvldparameters.inlierRate >
      KVLD(
        imgA, imgB,
        _featsL, _featsR,
        matchesPair, matchesFiltered,
        vec_score, E, valid, kvldparameters ) )
    {
      kvldparameters.inlierRate /= 2;
      ALICEVISION_LOG_DEBUG("low inlier rate, re-select matches with new rate=" << kvldparameters.inlierRate);
      kvldparameters.K = 2;
      it_num++;
    }

    if(matchesPair.empty())
    {
      maskLeft.fill(0);
      maskRight.fill(0);
      return false;
    }

    // Get mask
    getKVLDMask(
      &maskLeft, &maskRight,
      _featsL, _featsR,
      matchesPair,
      valid,
      E);

    return true;
  }

private:
  // Left and Right features
  const vector<feature::SIOPointFeature>& _featsL;
  const vector<feature::SIOPointFeature>& _featsR;
  // Left and Right corresponding index (putatives matches)
  matching::IndMatches _matches;
};

}  // namespace color_harmonization
}  // namespace aliceVision

#endif  // ALICEVISION_COLORHARMONIZATION_VLDSEGMENT_H
