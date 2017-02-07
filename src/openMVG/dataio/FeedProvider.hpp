/* 
 * File:   FeedProvider.hpp
 * Author: sgaspari
 *
 * Created on September 28, 2015, 6:48 PM
 */

#pragma once

#include "IFeed.hpp"

#include <string>
#include <memory>

namespace openMVG{
namespace dataio{

class FeedProvider
{
public:
  
  FeedProvider(const std::string &feedPath, const std::string &calibPath = "");
  
  /**
   * @brief Provide a new image from the feed.
   * 
   * @param[out] imageGray The new image from the feed.
   * @param[out] camIntrinsics The associated camera intrinsics.
   * @param[out] mediaPath The original media path, for a video is the path to the 
   * file, for an image sequence is the path to the single image.
   * @param[out] hasIntrinsics True if \p camIntrinsics is valid, otherwise there
   * is no intrinsics associated to \p imageGray.
   * @return True if there is a new image, false otherwise.
   */
  bool readImage(image::Image<unsigned char> &imageGray,
        cameras::Pinhole_Intrinsic_Radial_K3 &camIntrinsics,
        std::string &mediaPath,
        bool &hasIntrinsics);

  /**
   * @brief It returns the number of frames contained of the video. It return infinity
   * if the feed is a live stream.
   * @return the number of frames of the video or infinity if it is a live stream.
   */
  std::size_t nbFrames() const;

  /**
   * @brief It retrieve the given frame number. In case
   * of live feeds, it just give the next available frame.
   * @return true if successful.
   */
  bool goToFrame(const unsigned int frame);

  /**
   * @brief It acquires the next available frame.
   * @return true if successful.
   */  
  bool goToNextFrame();

  /**
   * @brief Return true if the feed is correctly initialized.
   * 
   * @return True if the feed is correctly initialized.
   */  
  bool isInit() const;

  /**
   * @brief Return true if the feed is a video.
   * 
   * @return True if the feed is a video.
   */    
  bool isVideo() const {return _isVideo; }
  
  /**
   * @brief Return true if the feed is a live stream (e.g. a  webcam).
   * 
   * @return True if the feed is correctly initialized.
   */    
  bool isLiveFeed() const {return _isLiveFeed; }

  virtual ~FeedProvider();
    
private:
  std::unique_ptr<IFeed> _feeder;
  bool _isVideo;
  bool _isLiveFeed;

};

}//namespace dataio 
}//namespace openMVG

