// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef ALICEVISION_COLORHARMONIZATION_FULLFRAME_H
#define ALICEVISION_COLORHARMONIZATION_FULLFRAME_H

#include "aliceVision/color_harmonization/selection_interface.hpp"

namespace aliceVision {
namespace color_harmonization {

class commonDataByPair_FullFrame  : public commonDataByPair
{
public:
  commonDataByPair_FullFrame( const std::string & sLeftImage,
                              const std::string & sRightImage ):
        commonDataByPair( sLeftImage, sRightImage )
  {}

  virtual ~commonDataByPair_FullFrame() {}

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

}  // namespace color_harmonization
}  // namespace aliceVision

#endif  // ALICEVISION_COLORHARMONIZATION_FULLFRAME_H
