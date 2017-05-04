
// Copyright (c) 2014 openMVG authors.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_COLORHARMONIZATION_VLDSEGMENT_H
#define OPENMVG_COLORHARMONIZATION_VLDSEGMENT_H

#include "openMVG/color_harmonization/selection_interface.hpp"
#include "openMVG/matching/kvld/kvld.h"
#include "openMVG/matching/kvld/kvld_draw.h"

namespace openMVG {
namespace color_harmonization {

class commonDataByPair_VLDSegment  : public commonDataByPair
{
  public:
  commonDataByPair_VLDSegment( const std::string& sLeftImage,
                               const std::string& sRightImage,
                               const matching::IndMatches& matchesPerDesc,
                               const std::vector<features::SIOPointFeature>& featsL,
                               const std::vector<features::SIOPointFeature>& featsR):
           commonDataByPair( sLeftImage, sRightImage ),
           _matches( matchesPerDesc ),
           _featsL( featsL ), _featsR( featsR )
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
    //std::vector< matching::IndMatch > vec_KVLDMatches;

    image::Image< unsigned char > imageL, imageR;
    image::ReadImage( _sLeftImage.c_str(), &imageL );
    image::ReadImage( _sRightImage.c_str(), &imageR );

    image::Image< float > imgA ( imageL.GetMat().cast< float >() );
    image::Image< float > imgB(imageR.GetMat().cast< float >());

    std::vector< Pair > matchesFiltered, matchesPair;

    for(const matching::IndMatch& match : _matches)
    {
      matchesPair.push_back( std::make_pair( match._i, match._j ) );
    }

    std::vector< double > vec_score;

    //In order to illustrate the gvld(or vld)-consistant neighbors, the following two parameters has been externalized as inputs of the function KVLD.
    openMVG::Mat E = openMVG::Mat::Ones( _matches.size(), _matches.size() ) * ( -1 );
    // gvld-consistancy matrix, intitialized to -1,  >0 consistancy value, -1=unknow, -2=false
    std::vector< bool > valide( _matches.size(), true );// indices of match in the initial matches, if true at the end of KVLD, a match is kept.

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
        vec_score, E, valide, kvldparameters ) )
    {
      kvldparameters.inlierRate /= 2;
      OPENMVG_LOG_DEBUG("low inlier rate, re-select matches with new rate=" << kvldparameters.inlierRate);
      kvldparameters.K = 2;
      it_num++;
    }

    if(matchesPair.empty())
      return false;

    // Get mask
    getKVLDMask(
      &maskLeft, &maskRight,
      _featsL, _featsR,
      matchesPair,
      valide,
      E);

    return true;
  }

private:
  // Left and Right features
  const vector<features::SIOPointFeature>& _featsL;
  const vector<features::SIOPointFeature>& _featsR;
  // Left and Right corresponding index (putatives matches)
  matching::IndMatches _matches;
};

}  // namespace color_harmonization
}  // namespace openMVG

#endif  // OPENMVG_COLORHARMONIZATION_VLDSEGMENT_H
