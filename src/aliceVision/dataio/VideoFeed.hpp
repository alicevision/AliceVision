// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include "IFeed.hpp"

#include <string>
#include <memory>

namespace aliceVision{
namespace dataio{

class VideoFeed : public IFeed
{
public:
  VideoFeed();

  /**
   * @brief Set up an image feed from a video
   * 
   * @param[in] videoPath The video source.
   * @param[in] calibPath The source for the camera intrinsics. 
   * The format for the file is
   * int #image width
   * int #image height
   * double #focal
   * double #ppx principal point x-coord
   * double #ppy principal point y-coord
   * double #k0
   * double #k1
   * double #k2
   * 
   * @see readCalibrationFromFile()
   */  
  VideoFeed(const std::string &videoPath, const std::string &calibPath);

  /**
   * @brief Set up an image feed from a video
   * 
   * @param[in] videoDevice The device id from which capture the live feed.
   * @param[in] calibPath The source for the camera intrinsics. 
   * The format for the file is
   * int #image width
   * int #image height
   * double #focal
   * double #ppx principal point x-coord
   * double #ppy principal point y-coord
   * double #k0
   * double #k1
   * double #k2
   * 
   * @see readCalibrationFromFile()
   */    
  VideoFeed(int videoDevice, const std::string &calibPath);
  
  /**
   * @brief Provide a new RGB image from the feed
   * 
   * @param[out] imageRGB The new RGB image from the feed.
   * @param[out] camIntrinsics The associated camera intrinsics.
   * @param[out] mediaPath The original video path.
   * @param[out] hasIntrinsics True if \p camIntrinsics is valid, otherwise there
   * is no intrinsics associated to \p imageRGB.
   * @return True if there is a new image, false otherwise.
   */
  bool readImage(image::Image<image::RGBColor> &imageRGB,
            camera::PinholeRadialK3 &camIntrinsics,
            std::string &mediaPath,
            bool &hasIntrinsics);

  /**
   * @brief Provide a new grayscale image from the feed
   * 
   * @param[out] imageGray The new image from the feed.
   * @param[out] camIntrinsics The associated camera intrinsics.
   * @param[out] mediaPath The original video path.
   * @param[out] hasIntrinsics True if \p camIntrinsics is valid, otherwise there
   * is no intrinsics associated to \p imageGray.
   * @return True if there is a new image, false otherwise.
   */
  bool readImage(image::Image<unsigned char> &imageGray,
            camera::PinholeRadialK3 &camIntrinsics,
            std::string &mediaPath,
            bool &hasIntrinsics);
  
  std::size_t nbFrames() const;
  
  bool goToFrame(const unsigned int frame);
  
  bool goToNextFrame();
  
  /**
   * @brief Return true if the feed is correctly initialized.
   * 
   * @return True if the feed is correctly initialized.
   */  
  bool isInit() const;

  virtual ~VideoFeed( );
  
private:
  class FeederImpl;
  std::unique_ptr<FeederImpl> _feeder;
};

}//namespace dataio 
}//namespace aliceVision
