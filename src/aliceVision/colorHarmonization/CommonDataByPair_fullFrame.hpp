// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

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
