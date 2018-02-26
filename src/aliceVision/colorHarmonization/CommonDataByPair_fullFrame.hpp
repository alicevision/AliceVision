// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/colorHarmonization/CommonDataByPair.hpp"

namespace aliceVision {
namespace colorHarmonization {

class CommonDataByPair_fullFrame  : public CommonDataByPair
{
public:
  CommonDataByPair_fullFrame( const std::string & sLeftImage,
                              const std::string & sRightImage ):
        CommonDataByPair( sLeftImage, sRightImage )
  {}

  virtual ~CommonDataByPair_fullFrame() {}

  /**
   * Put masks to white, all image is considered as valid pixel selection
   *
   * \param[out] maskLeft Mask of the left image (initialized to corresponding image size).
   * \param[out] maskRight  Mask of the right image (initialized to corresponding image size).
   *
   * \return True.
   */
  virtual bool computeMask( image::Image< unsigned char > & maskLeft, image::Image< unsigned char > & maskRight )
  {
    maskLeft.fill( image::WHITE );
    maskRight.fill( image::WHITE );
    return true;
  }

private:

};

}  // namespace colorHarmonization
}  // namespace aliceVision
