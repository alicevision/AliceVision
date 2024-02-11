// This file is part of the AliceVision project.
// Copyright (c) 2015 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "FeedProvider.hpp"
#include <aliceVision/config.hpp>
#include "ImageFeed.hpp"
#include "SfMDataFeed.hpp"
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
    #include "VideoFeed.hpp"
#endif

#include <filesystem>
#include <exception>
#include <iostream>
#include <string>
#include <limits>
#include <ctype.h>

namespace aliceVision {
namespace dataio {

FeedProvider::FeedProvider(const std::string& feedPath, const std::string& calibPath)
  : _isVideo(false),
    _isLiveFeed(false),
    _isSfmData(false)
{
    namespace fs = std::filesystem;
    if (feedPath.empty())
    {
        throw std::invalid_argument("Empty filepath.");
    }
    if (fs::is_regular_file(fs::path(feedPath)))
    {
        // Image or video file
        const std::string extension = fs::path(feedPath).extension().string();
        if (SfMDataFeed::isSupported(extension))
        {
            _feeder.reset(new SfMDataFeed(feedPath, calibPath));
            _isSfmData = true;
        }
        else if (ImageFeed::isSupported(extension))
        {
            _feeder.reset(new ImageFeed(feedPath, calibPath));
        }
        else
        {
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
            if (VideoFeed::isSupported(extension))
            {
                // let's try it with a video
                _feeder.reset(new VideoFeed(feedPath, calibPath));
                _isVideo = true;
            }
            else
            {
                throw std::invalid_argument("Unsupported file format: " + feedPath);
            }
#else
            throw std::invalid_argument("Unsupported mode! If you intended to use a video"
                                        " please add OpenCV support");
#endif
        }
    }
    // parent_path() returns "/foo/bar/" when input path equals to "/foo/bar/"
    // if the user just gives the relative path as "bar", throws invalid argument exception.
    else if (fs::is_directory(fs::path(feedPath)) || fs::is_directory(fs::path(feedPath).parent_path()))
    {
        // Folder or sequence of images
        _feeder.reset(new ImageFeed(feedPath, calibPath));
    }
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
    else if (isdigit(feedPath[0]))
    {
        // let's try it with a video
        const int deviceNumber = std::stoi(feedPath);
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

bool FeedProvider::readImage(image::Image<image::RGBColor>& imageRGB, camera::Pinhole& camIntrinsics, std::string& mediaPath, bool& hasIntrinsics)
{
    return (_feeder->readImage(imageRGB, camIntrinsics, mediaPath, hasIntrinsics));
}

bool FeedProvider::readImage(image::Image<float>& imageGray, camera::Pinhole& camIntrinsics, std::string& mediaPath, bool& hasIntrinsics)
{
    return (_feeder->readImage(imageGray, camIntrinsics, mediaPath, hasIntrinsics));
}

bool FeedProvider::readImage(image::Image<unsigned char>& imageGray, camera::Pinhole& camIntrinsics, std::string& mediaPath, bool& hasIntrinsics)
{
    return (_feeder->readImage(imageGray, camIntrinsics, mediaPath, hasIntrinsics));
}

std::size_t FeedProvider::nbFrames() const
{
    if (_isLiveFeed)
        return std::numeric_limits<std::size_t>::infinity();

    return _feeder->nbFrames();
}

bool FeedProvider::goToFrame(const unsigned int frame) { return _feeder->goToFrame(frame); }

bool FeedProvider::goToNextFrame() { return _feeder->goToNextFrame(); }

bool FeedProvider::isInit() const { return (_feeder->isInit()); }

FeedProvider::~FeedProvider() {}

}  // namespace dataio
}  // namespace aliceVision
