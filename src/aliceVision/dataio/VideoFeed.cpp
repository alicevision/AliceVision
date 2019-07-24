// This file is part of the AliceVision project.
// Copyright (c) 2015 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "VideoFeed.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/image/convertion.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/videoio.hpp>

#include <iostream>
#include <exception>

namespace aliceVision{
namespace dataio{

class VideoFeed::FeederImpl
{
public:
  FeederImpl() : _isInit(false) { }
  
  FeederImpl(const std::string &videoPath, const std::string &calibPath);
  
  FeederImpl(int videoDevice, const std::string &calibPath);
  
  bool isInit() const {return _isInit;}
  
  bool readImage(image::Image<image::RGBColor> &imageRGB,
                   camera::PinholeRadialK3 &camIntrinsics,
                   std::string &mediaPath,
                   bool &hasIntrinsics);

  bool readImage(image::Image<float> &imageGray,
                     camera::PinholeRadialK3 &camIntrinsics,
                     std::string &mediaPath,
                     bool &hasIntrinsics);
  
  bool readImage(image::Image<unsigned char> &imageGray,
                     camera::PinholeRadialK3 &camIntrinsics,
                     std::string &mediaPath,
                     bool &hasIntrinsics);
  
  bool goToFrame(const unsigned int frame);
  
  bool goToNextFrame();
  
  std::size_t nbFrames() const;
  
private:
  bool _isInit;
  bool _isLive;
  bool _withIntrinsics;
  std::string _videoPath;
  cv::VideoCapture _videoCapture;
  camera::PinholeRadialK3 _camIntrinsics;
};


VideoFeed::FeederImpl::FeederImpl(const std::string &videoPath, const std::string &calibPath)
: _isInit(false), _isLive(false), _withIntrinsics(false), _videoPath(videoPath)
{
    // load the video
  _videoCapture.open(videoPath);
  if (!_videoCapture.isOpened())
  {
    ALICEVISION_LOG_WARNING("Unable to open the video : " << videoPath);
    throw std::invalid_argument("Unable to open the video : "+videoPath);
  }
  // Grab frame 0, so we can call readImage.
  goToFrame(0);

  // load the calibration path
  _withIntrinsics = !calibPath.empty();
  if(_withIntrinsics)
    readCalibrationFromFile(calibPath, _camIntrinsics);
  
  _isInit = true;
}

VideoFeed::FeederImpl::FeederImpl(int videoDevice, const std::string &calibPath)
: _isInit(false), _isLive(true), _withIntrinsics(false), _videoPath(std::to_string(videoDevice))
{
    // load the video
  _videoCapture.open(videoDevice);
  if (!_videoCapture.isOpened())
  {
    ALICEVISION_LOG_WARNING("Unable to open the video : " << _videoPath);
    throw std::invalid_argument("Unable to open the video : "+_videoPath);
  }

  goToNextFrame();
  
  // load the calibration path
  _withIntrinsics = !calibPath.empty();
  if(_withIntrinsics)
    readCalibrationFromFile(calibPath, _camIntrinsics);
  
  _isInit = true;
}

bool VideoFeed::FeederImpl::readImage(image::Image<image::RGBColor> &imageRGB,
          camera::PinholeRadialK3 &camIntrinsics,
          std::string &mediaPath,
          bool &hasIntrinsics)
{
  cv::Mat frame;
  const bool grabStatus = _videoCapture.retrieve(frame);

  if(!grabStatus || !frame.data)
  {
    return false;
  }
  
  if(frame.channels() == 3)
  {
    cv::Mat color;
    resize(frame, color, cv::Size(frame.cols, frame.rows));
    
    cv::cvtColor(frame, color, cv::COLOR_BGR2RGB);
    imageRGB.resize(color.cols, color.rows);
    
    unsigned char* pixelPtr = (unsigned char*)color.data;
    for(int i = 0; i < color.rows; i++)
    {
      for(int j = 0; j < color.cols; j++)
      {
        const size_t index = i*color.cols*3 + j*3;
        imageRGB(i,j) = image::RGBColor(pixelPtr[index], pixelPtr[index + 1], pixelPtr[index + 2]);
      }
    }
  }
  else
  {
    ALICEVISION_LOG_WARNING("Error can't read RGB frame " << _videoPath);
    throw std::invalid_argument("Error can't read RGB frame " + _videoPath);
  }
  
  hasIntrinsics = _withIntrinsics;
  if(_withIntrinsics)
    camIntrinsics = _camIntrinsics;

  mediaPath = _videoPath;
  return true;
}

bool VideoFeed::FeederImpl::readImage(image::Image<float> &imageGray,
          camera::PinholeRadialK3 &camIntrinsics,
          std::string &mediaPath,
          bool &hasIntrinsics)
{
  image::Image<unsigned char> imageGrayUChar;
  if(FeederImpl::readImage(imageGrayUChar, camIntrinsics, mediaPath, hasIntrinsics))
  {
    imageGray = (imageGrayUChar.GetMat().cast<float>() / 255.f);
    return true;
  }
  return false;
}


bool VideoFeed::FeederImpl::readImage(image::Image<unsigned char> &imageGray,
                   camera::PinholeRadialK3 &camIntrinsics,
                   std::string &mediaPath,
                   bool &hasIntrinsics)
{
  cv::Mat frame;
  const bool grabStatus = _videoCapture.retrieve(frame);

  if(!grabStatus || !frame.data)
  {
    return false;
  }
  
  if(frame.channels() == 3)
  {
    // convert to gray
    cv::Mat grey;
    cv::cvtColor(frame, grey, cv::COLOR_BGR2GRAY);
    imageGray.resize(grey.cols, grey.rows);
    cv::cv2eigen(grey, imageGray);
//      ALICEVISION_LOG_DEBUG(grey.channels() << " " << grey.rows << " " << grey.cols);
//      ALICEVISION_LOG_DEBUG(imageGray.Depth() << " " << imageGray.Height() << " " << imageGray.Width());
  }
  else
  {
    cv::cv2eigen(frame, imageGray);
  }

  hasIntrinsics = _withIntrinsics;
  if(_withIntrinsics)
    camIntrinsics = _camIntrinsics;

  mediaPath = _videoPath;
  return true;
}

std::size_t VideoFeed::FeederImpl::nbFrames() const
{
  if (!_videoCapture.isOpened())
    return 0;
  return _videoCapture.get(cv::CAP_PROP_FRAME_COUNT);
}

bool VideoFeed::FeederImpl::goToFrame(const unsigned int frame)
{
  if (!_videoCapture.isOpened())
  {
    ALICEVISION_LOG_WARNING("We cannot open the video file.");
    return false;
  }
  
  if(_isLive)
    return goToNextFrame();
  
  if(frame > 0)
  {
    _videoCapture.set(cv::CAP_PROP_POS_FRAMES, frame);
    _videoCapture.grab();
    return true;
  }
  else
  {
    _videoCapture.set(cv::CAP_PROP_POS_FRAMES, 0);
    _videoCapture.grab();
    return false;
  }
}

bool VideoFeed::FeederImpl::goToNextFrame()
{
  return _videoCapture.grab();
}

/*******************************************************************************/
/*                                 VideoFeed                                   */
/*******************************************************************************/

VideoFeed::VideoFeed() : _feeder(new FeederImpl()) { }

VideoFeed::VideoFeed(const std::string &videoPath, const std::string &calibPath) 
  : _feeder(new FeederImpl(videoPath, calibPath))
{ }

VideoFeed::VideoFeed(int videoDevice, const std::string &calibPath) 
  : _feeder(new FeederImpl(videoDevice, calibPath))
{ }

bool VideoFeed::readImage(image::Image<image::RGBColor> &imageRGB,
                     camera::PinholeRadialK3 &camIntrinsics,
                     std::string &mediaPath,
                     bool &hasIntrinsics)
{
  return(_feeder->readImage(imageRGB, camIntrinsics, mediaPath, hasIntrinsics));
}

bool VideoFeed::readImage(image::Image<float> &imageGray,
                     camera::PinholeRadialK3 &camIntrinsics,
                     std::string &mediaPath,
                     bool &hasIntrinsics)
{
  return(_feeder->readImage(imageGray, camIntrinsics, mediaPath, hasIntrinsics));
}

bool VideoFeed::readImage(image::Image<unsigned char> &imageGray,
                     camera::PinholeRadialK3 &camIntrinsics,
                     std::string &mediaPath,
                     bool &hasIntrinsics)
{
  return(_feeder->readImage(imageGray, camIntrinsics, mediaPath, hasIntrinsics));
}

std::size_t VideoFeed::nbFrames() const
{
  return _feeder->nbFrames();
}

bool VideoFeed::goToFrame(const unsigned int frame)
{
  return _feeder->goToFrame(frame);
}

bool VideoFeed::goToNextFrame()
{
  return _feeder->goToNextFrame();
}

bool VideoFeed::isInit() const {return(_feeder->isInit()); }

VideoFeed::~VideoFeed() { }

}//namespace dataio 
}//namespace aliceVision
