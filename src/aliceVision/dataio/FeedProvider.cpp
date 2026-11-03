// This file is part of the AliceVision project.
// Copyright (c) 2015 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "FeedProvider.hpp"
#include "FeedFactory.hpp"

#include <stdexcept>
#include <string>

namespace aliceVision {
namespace dataio {

FeedProvider::FeedProvider(const std::string& feedPath, const std::string& calibPath)
{
    if (feedPath.empty())
    {
        throw std::invalid_argument("Empty filepath.");
    }
    _feeder = FeedFactory::instance().create(feedPath, calibPath);
    if (!_feeder)
    {
        throw std::invalid_argument(std::string("Input filepath not supported (no registered feed can open it): ") + feedPath);
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

std::size_t FeedProvider::nbFrames() const { return _feeder->nbFrames(); }

bool FeedProvider::goToFrame(const unsigned int frame) { return _feeder->goToFrame(frame); }

bool FeedProvider::goToNextFrame() { return _feeder->goToNextFrame(); }

bool FeedProvider::isInit() const { return (_feeder->isInit()); }

FeedProvider::~FeedProvider() {}

}  // namespace dataio
}  // namespace aliceVision
