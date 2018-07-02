// This file is part of the AliceVision project.
// Copyright (c) 2015 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "FeedProvider.hpp"
#include <aliceVision/config.hpp>
#include "ImageFeed.hpp"
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
#include "VideoFeed.hpp"
#endif

#include <boost/filesystem.hpp>

#include <exception>
#include <iostream>
#include <string>
#include <limits>
#include <ctype.h>

namespace aliceVision{
namespace dataio{

FeedProvider::FeedProvider(const std::string &feedPath, const std::string &calibPath) 
: _isVideo(false), _isLiveFeed(false)
{
  namespace bf = boost::filesystem;
  if(feedPath.empty())
  {
    throw std::invalid_argument("Empty filepath.");
  }
  if(bf::is_regular_file(bf::path(feedPath))) 
  {
    // Image or video file
    const std::string extension = bf::path(feedPath).extension().string();
    if(ImageFeed::isSupported(extension))
    {
      _feeder.reset(new ImageFeed(feedPath, calibPath));
    }
    else 
    {
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
      // let's try it with a video
      _feeder.reset(new VideoFeed(feedPath, calibPath));
      _isVideo = true;
#else
      throw std::invalid_argument("Unsupported mode! If you intended to use a video"
                                  " please add OpenCV support");
#endif
    }
  }
  // parent_path() returns "/foo/bar/" when input path equals to "/foo/bar/"
  // if the user just gives the relative path as "bar", throws invalid argument exception.
  else if(bf::is_directory(bf::path(feedPath)) || bf::is_directory(bf::path(feedPath).parent_path()))
  {
    // Folder or sequence of images
    _feeder.reset(new ImageFeed(feedPath, calibPath));
  }
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
  else if(isdigit(feedPath[0]))
  {
    // let's try it with a video
    const int deviceNumber =  std::atoi(feedPath.c_str());
    _feeder.reset(new VideoFeed(deviceNumber, calibPath));
    _isVideo = true;
    _isLiveFeed = true;
  }
#endif
  else
  {
    throw std::invalid_argument(std::string("Input filepath not supported: ") + feedPath);
  }
}

bool FeedProvider::readImage(image::Image<image::RGBColor> &imageRGB,
      camera::PinholeRadialK3 &camIntrinsics,
      std::string &mediaPath,
      bool &hasIntrinsics)
{
  return(_feeder->readImage(imageRGB, camIntrinsics, mediaPath, hasIntrinsics));
}

bool FeedProvider::readImage(image::Image<float> &imageGray,
      camera::PinholeRadialK3 &camIntrinsics,
      std::string &mediaPath,
      bool &hasIntrinsics)
{
  return(_feeder->readImage(imageGray, camIntrinsics, mediaPath, hasIntrinsics));
}

bool FeedProvider::readImage(image::Image<unsigned char> &imageGray,
      camera::PinholeRadialK3 &camIntrinsics,
      std::string &mediaPath,
      bool &hasIntrinsics)
{
  return(_feeder->readImage(imageGray, camIntrinsics, mediaPath, hasIntrinsics));
}
  
std::size_t FeedProvider::nbFrames() const
{
  if(_isLiveFeed)
    return std::numeric_limits<std::size_t>::infinity();
  
  return _feeder->nbFrames();
}

bool FeedProvider::goToFrame(const unsigned int frame)
{
  return _feeder->goToFrame(frame);
}

bool FeedProvider::goToNextFrame()
{
  return _feeder->goToNextFrame();
}

bool FeedProvider::isInit() const
{
  return(_feeder->isInit());
}

FeedProvider::~FeedProvider( ) { }

}//namespace dataio 
}//namespace aliceVision
