/* 
 * File:   IFeed.hpp
 * Author: sgaspari
 *
 * Created on September 28, 2015, 10:24 AM
 */

#pragma once

#include <openMVG/cameras/Camera_Pinhole_Radial.hpp>
#include <openMVG/image/image_container.hpp>

namespace openMVG{
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
   * @brief Provide a new image from the feed
   * @param[out] imageGray The new image from the feed.
   * @param[out] camIntrinsics The associated camera intrinsics.
   * @param[out] mediaPath The original media path.
   * @param[out] hasIntrinsics True if \p camIntrinsics is valid, otherwise there
   * is no intrinsics associated to \p imageGray.
   * @return True if there is a new image, false otherwise.
   */
  virtual bool readImage(image::Image<unsigned char> &imageGray,
                    cameras::Pinhole_Intrinsic_Radial_K3 &camIntrinsics, 
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
void readCalibrationFromFile(const std::string &filename, cameras::Pinhole_Intrinsic_Radial_K3 &camIntrinsics);

}//namespace dataio 
}//namespace openMVG

