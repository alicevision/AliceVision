// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SfMDataFeed.hpp"
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/image/io.hpp>

#include <boost/algorithm/string/case_conv.hpp>

#include <exception>
#include <iterator>
#include <string>

namespace aliceVision {
namespace dataio {

class SfMDataFeed::FeederImpl
{
  public:
    FeederImpl()
      : _isInit(false)
    {}

    FeederImpl(const std::string& imagePath, const std::string& calibPath);

    template<typename T>
    bool readImage(image::Image<T>& image, camera::Pinhole& camIntrinsics, std::string& imageName, bool& hasIntrinsics)
    {
        if (!_isInit)
        {
            ALICEVISION_LOG_WARNING("SfMData feed is not initialized ");
            return false;
        }

        if (_currentImageIndex >= _views.size())
        {
            ALICEVISION_LOG_WARNING("No more images to process");
            return false;
        }

        // Get the image path
        const sfmData::View* view = _views.at(_currentImageIndex);
        imageName = view->getImage().getImagePath();
        image::readImage(imageName, image, image::EImageColorSpace::NO_CONVERSION);

        return true;
    }

    std::size_t nbFrames() const;

    bool goToFrame(const unsigned int frame);

    bool goToNextFrame();

    bool isInit() const { return _isInit; }

  private:
    bool _isInit;

    sfmData::SfMData _sfmData;
    std::vector<const sfmData::View*> _views;
    unsigned int _currentImageIndex = 0;
};

SfMDataFeed::FeederImpl::FeederImpl(const std::string& imagePath, const std::string& calibPath)
  : _isInit(false)
{
    _isInit = sfmDataIO::load(_sfmData, imagePath, sfmDataIO::ESfMData(sfmDataIO::ESfMData::VIEWS | sfmDataIO::ESfMData::INTRINSICS));

    // Order the views according to the frame ID and the intrinsics serial number
    std::map<std::string, std::vector<const sfmData::View*>> viewSequences;

    // Separate the views depending on their intrinsics' serial number
    auto& intrinsics = _sfmData.getIntrinsics();
    for (auto it = _sfmData.getViews().begin(); it != _sfmData.getViews().end(); ++it)
    {
        auto view = it->second.get();
        auto serialNumber = intrinsics.at(view->getIntrinsicId())->serialNumber();
        viewSequences[serialNumber].push_back(view);
    }

    // Sort the views with the same intrinsics together based on their frame ID and add them to the final global vector
    for (auto& view : viewSequences)
    {
        std::sort(view.second.begin(), view.second.end(), [](const sfmData::View* v1, const sfmData::View* v2) {
            return v1->getFrameId() < v2->getFrameId();
        });
        _views.insert(_views.end(), view.second.begin(), view.second.end());
    }
}

std::size_t SfMDataFeed::FeederImpl::nbFrames() const
{
    if (!_isInit)
        return 0;

    return _views.size();
}

bool SfMDataFeed::FeederImpl::goToFrame(const unsigned int frame)
{
    _currentImageIndex = frame;
    if (!_isInit)
    {
        ALICEVISION_LOG_WARNING("SfmData feed is not initialized");
        return false;
    }

    if (frame >= _views.size())
    {
        ALICEVISION_LOG_WARNING("The current frame is out of the range.");
        return false;
    }

    return true;
}

bool SfMDataFeed::FeederImpl::goToNextFrame()
{
    ++_currentImageIndex;
    ALICEVISION_LOG_DEBUG("next frame " << _currentImageIndex);
    if (_currentImageIndex >= _views.size())
        return false;

    return true;

    return true;
}

/*******************************************************************************/
/*                     SfMDataFeed                                               */
/*******************************************************************************/

SfMDataFeed::SfMDataFeed()
  : _sfmDataFeed(new FeederImpl())
{}

SfMDataFeed::SfMDataFeed(const std::string& imagePath, const std::string& calibPath)
  : _sfmDataFeed(new FeederImpl(imagePath, calibPath))
{}

bool SfMDataFeed::readImage(image::Image<image::RGBColor>& imageRGB, camera::Pinhole& camIntrinsics, std::string& mediaPath, bool& hasIntrinsics)
{
    return (_sfmDataFeed->readImage(imageRGB, camIntrinsics, mediaPath, hasIntrinsics));
}

bool SfMDataFeed::readImage(image::Image<float>& imageGray, camera::Pinhole& camIntrinsics, std::string& mediaPath, bool& hasIntrinsics)
{
    return (_sfmDataFeed->readImage(imageGray, camIntrinsics, mediaPath, hasIntrinsics));
}

bool SfMDataFeed::readImage(image::Image<unsigned char>& imageGray, camera::Pinhole& camIntrinsics, std::string& mediaPath, bool& hasIntrinsics)
{
    return (_sfmDataFeed->readImage(imageGray, camIntrinsics, mediaPath, hasIntrinsics));
}

std::size_t SfMDataFeed::nbFrames() const { return _sfmDataFeed->nbFrames(); }

bool SfMDataFeed::goToFrame(const unsigned int frame) { return _sfmDataFeed->goToFrame(frame); }

bool SfMDataFeed::goToNextFrame() { return _sfmDataFeed->goToNextFrame(); }

bool SfMDataFeed::isInit() const { return (_sfmDataFeed->isInit()); }

bool SfMDataFeed::isSupported(const std::string& extension)
{
    std::string ext = boost::to_lower_copy(extension);
    return (ext == ".sfm" || ext == ".abc" || ext == ".json");
}

SfMDataFeed::~SfMDataFeed() {}

}  // namespace dataio
}  // namespace aliceVision
