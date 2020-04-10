// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/colorHarmonization/CommonDataByPair.hpp"
#include "aliceVision/matching/kvld/kvld.h"
#include "aliceVision/matching/kvld/kvld_draw.h"

namespace aliceVision {
namespace colorHarmonization {

class CommonDataByPair_vldSegment  : public CommonDataByPair
{
  public:
  CommonDataByPair_vldSegment( const std::string& sLeftImage,
                               const std::string& sRightImage,
                               const matching::IndMatches& matchesPerDesc,
                               const std::vector<feature::PointFeature>& featsL,
                               const std::vector<feature::PointFeature>& featsR)
           : CommonDataByPair( sLeftImage, sRightImage )
           , _matches( matchesPerDesc )
           , _featsL( featsL )
           , _featsR( featsR )
  {}

  virtual ~CommonDataByPair_vldSegment()
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
    image::readImage( _sLeftImage, imageL, image::EImageColorSpace::LINEAR);
    image::readImage( _sRightImage, imageR, image::EImageColorSpace::LINEAR);

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
  const std::vector<feature::PointFeature>& _featsL;
  const std::vector<feature::PointFeature>& _featsR;
  // Left and Right corresponding index (putatives matches)
  matching::IndMatches _matches;
};

}  // namespace colorHarmonization
}  // namespace aliceVision
