// This file is part of the AliceVision project.
// Copyright (c) 2015 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/camera/PinholeRadial.hpp>
#include <aliceVision/image/Image.hpp>
#include <aliceVision/image/pixelTypes.hpp>

namespace aliceVision{
namespace dataio{

class IFeed
{
public:
  IFeed() { };

  /**
   * @brief Return true if the feed is correctly initialized.
   * @return True if the feed is correctly initialized.
   */
  virtual bool isInit() const = 0;
  
    /**
   * @brief Provide a new RGB image from the feed
   * @param[out] imageRGB The new RGB image from the feed.
   * @param[out] camIntrinsics The associated camera intrinsics.
   * @param[out] mediaPath The original media path.
   * @param[out] hasIntrinsics True if \p camIntrinsics is valid, otherwise there
   * is no intrinsics associated to \p imageRGB.
   * @return True if there is a new image, false otherwise.
   */
  virtual bool readImage(image::Image<image::RGBColor> &imageRGB,
                    camera::PinholeRadialK3 &camIntrinsics,
                    std::string &mediaPath,
                    bool &hasIntrinsics) = 0;

  /**
   * @brief Provide a new float grayscale image from the feed
   * @param[out] imageGray The new image from the feed.
   * @param[out] camIntrinsics The associated camera intrinsics.
   * @param[out] mediaPath The original media path.
   * @param[out] hasIntrinsics True if \p camIntrinsics is valid, otherwise there
   * is no intrinsics associated to \p imageGray.
   * @return True if there is a new image, false otherwise.
   */
  virtual bool readImage(image::Image<float> &imageGray,
                    camera::PinholeRadialK3 &camIntrinsics,
                    std::string &mediaPath,
                    bool &hasIntrinsics) = 0;
  
  /**
   * @brief Provide a new grayscale image from the feed
   * @param[out] imageGray The new image from the feed.
   * @param[out] camIntrinsics The associated camera intrinsics.
   * @param[out] mediaPath The original media path.
   * @param[out] hasIntrinsics True if \p camIntrinsics is valid, otherwise there
   * is no intrinsics associated to \p imageGray.
   * @return True if there is a new image, false otherwise.
   */
  virtual bool readImage(image::Image<unsigned char> &imageGray,
                    camera::PinholeRadialK3 &camIntrinsics, 
                    std::string &mediaPath,
                    bool &hasIntrinsics) = 0;  

  virtual std::size_t nbFrames() const = 0;
  
  virtual bool goToFrame(const unsigned int frame) = 0;
  
  virtual bool goToNextFrame() = 0;
  
  virtual ~IFeed( ) {}

};

//@todo to be move somewhere else more appropriated
/**
 * @brief Read the calibration from a simple text file.
 * @param[in] filename The file containing the calibration parameters.
 * @param[out] camIntrinsics The loaded parameters.
 */
void readCalibrationFromFile(const std::string &filename, camera::PinholeRadialK3 &camIntrinsics);

}//namespace dataio 
}//namespace aliceVision

