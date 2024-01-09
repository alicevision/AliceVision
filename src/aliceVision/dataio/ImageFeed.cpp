// This file is part of the AliceVision project.
// Copyright (c) 2015 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImageFeed.hpp"
#include <aliceVision/image/io.hpp>
#include <aliceVision/utils/regexFilter.hpp>

#include <boost/algorithm/string/case_conv.hpp>
#include <boost/algorithm/string/replace.hpp>

#include <filesystem>
#include <queue>
#include <iostream>
#include <fstream>
#include <exception>
#include <regex>
#include <iterator>
#include <string>

namespace aliceVision {
namespace dataio {

class ImageFeed::FeederImpl
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
            ALICEVISION_LOG_WARNING("Image feed is not initialized ");
            return false;
        }

        if (_images.empty())
            return false;
        if (_currentImageIndex >= _images.size())
            return false;

        if (_withCalibration)
        {
            // get the calibration
            camIntrinsics = _camIntrinsics;
            hasIntrinsics = true;
        }
        else
        {
            hasIntrinsics = false;
        }

        imageName = _images[_currentImageIndex];
        ALICEVISION_LOG_DEBUG(imageName);
        image::readImage(imageName, image, image::EImageColorSpace::NO_CONVERSION);

        return true;
    }

    std::size_t nbFrames() const;

    bool goToFrame(const unsigned int frame);

    bool goToNextFrame();

    bool isInit() const { return _isInit; }

  private:
    bool _isInit;
    bool _withCalibration;
    // It contains the images to be fed
    std::vector<std::string> _images;
    camera::Pinhole _camIntrinsics;

    unsigned int _currentImageIndex = 0;
};

ImageFeed::FeederImpl::FeederImpl(const std::string& imagePath, const std::string& calibPath)
  : _isInit(false),
    _withCalibration(false)
{
    namespace fs = std::filesystem;
    //    ALICEVISION_LOG_DEBUG(imagePath);
    // if it is a json, calibPath is neglected
    if (fs::is_regular_file(imagePath))
    {
        const std::string ext = fs::path(imagePath).extension().string();
        // if it is an image file
        if (image::isSupported(ext) && !image::isVideoExtension(ext))
        {
            _images.push_back(imagePath);
            _withCalibration = !calibPath.empty();
            _isInit = true;
        }
        // if it is an image file
        else if (ext == ".txt")
        {
            // we expect a simple txt file with a list of path to images relative to the
            // location of the txt file itself
            std::fstream fs(imagePath, std::ios::in);
            std::string line;
            // parse each line of the text file
            while (getline(fs, line))
            {
                // compose the file name as the base path of the inputPath and
                // the filename just read
                const std::string filename = (fs::path(imagePath).parent_path() / line).string();
                _images.push_back(filename);
            }
            // Close file
            fs.close();
            _withCalibration = !calibPath.empty();
            _isInit = true;
        }
        else
        {
            // no other file format are supported
            throw std::invalid_argument("File or mode not yet implemented");
        }
    }
    else if (fs::is_directory(imagePath) || fs::is_directory(fs::path(imagePath).parent_path()))
    {
        std::string folder = imagePath;
        // Recover the pattern : img.@.png (for example)
        std::string filePattern;
        std::regex re;
        if (!fs::is_directory(imagePath))
        {
            filePattern = fs::path(imagePath).filename().string();
            folder = fs::path(imagePath).parent_path().string();
            ALICEVISION_LOG_DEBUG("filePattern: " << filePattern);
            std::string regexStr = filePattern;
            re = utils::filterToRegex(regexStr);
        }
        else
        {
            ALICEVISION_LOG_DEBUG("folder without expression: " << imagePath);
        }
        ALICEVISION_LOG_DEBUG("directory feedImage");
        // if it is a directory, list all the images and add them to the list
        fs::directory_iterator iterator(folder);
        // since some OS will provide the files in a random order, first store them
        // in a priority queue and then fill the _image queue with the alphabetical
        // order from the priority queue
        std::priority_queue<std::string, std::vector<std::string>, std::greater<std::string>> tmpSorter;
        for (; iterator != fs::directory_iterator(); ++iterator)
        {
            // get the extension of the current file to check whether it is an image
            const std::string ext = iterator->path().extension().string();
            if (image::isSupported(ext) && !image::isVideoExtension(ext))
            {
                const std::string filepath = iterator->path().string();
                const std::string filename = iterator->path().filename().string();
                // If we have a filePattern (a sequence of images), we have to match the regex.
                if (filePattern.empty() || std::regex_match(filename, re))
                    tmpSorter.push(filepath);
            }
            else
            {
                ALICEVISION_LOG_WARNING("Unsupported file extension " << ext << " for " << iterator->path().string() << ".");
            }
        }
        // put all the retrieve files inside the queue
        while (!tmpSorter.empty())
        {
            _images.push_back(tmpSorter.top());
            tmpSorter.pop();
        }

        _withCalibration = !calibPath.empty();
        _isInit = true;
    }
    else
    {
        throw std::invalid_argument("File or mode not yet implemented");
    }

    // last thing: if _withCalibration is true it means that a path to a calibration file has been passed
    // then load the calibration
    if (_withCalibration)
    {
        // load the calibration from calibPath
        readCalibrationFromFile(calibPath, _camIntrinsics);
    }
}

std::size_t ImageFeed::FeederImpl::nbFrames() const
{
    if (!_isInit)
        return 0;

    return _images.size();
}

bool ImageFeed::FeederImpl::goToFrame(const unsigned int frame)
{
    if (!_isInit)
    {
        _currentImageIndex = frame;
        ALICEVISION_LOG_WARNING("Image feed is not initialized");
        return false;
    }

    _currentImageIndex = frame;
    // Image list mode
    if (frame >= _images.size())
    {
        ALICEVISION_LOG_WARNING("The current frame is out of the range.");
        return false;
    }
    ALICEVISION_LOG_DEBUG("frame " << frame);
    return true;
}

bool ImageFeed::FeederImpl::goToNextFrame()
{
    ++_currentImageIndex;
    ALICEVISION_LOG_DEBUG("next frame " << _currentImageIndex);
    if (_currentImageIndex >= _images.size())
        return false;

    return true;
}

/*******************************************************************************/
/*                     ImageFeed                                               */
/*******************************************************************************/

ImageFeed::ImageFeed()
  : _imageFeed(new FeederImpl())
{}

ImageFeed::ImageFeed(const std::string& imagePath, const std::string& calibPath)
  : _imageFeed(new FeederImpl(imagePath, calibPath))
{}

bool ImageFeed::readImage(image::Image<image::RGBColor>& imageRGB, camera::Pinhole& camIntrinsics, std::string& mediaPath, bool& hasIntrinsics)
{
    return (_imageFeed->readImage(imageRGB, camIntrinsics, mediaPath, hasIntrinsics));
}

bool ImageFeed::readImage(image::Image<float>& imageGray, camera::Pinhole& camIntrinsics, std::string& mediaPath, bool& hasIntrinsics)
{
    return (_imageFeed->readImage(imageGray, camIntrinsics, mediaPath, hasIntrinsics));
}

bool ImageFeed::readImage(image::Image<unsigned char>& imageGray, camera::Pinhole& camIntrinsics, std::string& mediaPath, bool& hasIntrinsics)
{
    return (_imageFeed->readImage(imageGray, camIntrinsics, mediaPath, hasIntrinsics));
}

std::size_t ImageFeed::nbFrames() const { return _imageFeed->nbFrames(); }

bool ImageFeed::goToFrame(const unsigned int frame) { return _imageFeed->goToFrame(frame); }

bool ImageFeed::goToNextFrame() { return _imageFeed->goToNextFrame(); }

bool ImageFeed::isInit() const { return (_imageFeed->isInit()); }

bool ImageFeed::isSupported(const std::string& extension)
{
    std::string ext = boost::to_lower_copy(extension);
    if (ext == ".txt")
    {
        return true;
    }
    else
    {
        return (image::isSupported(ext) && !image::isVideoExtension(ext));
    }
}

ImageFeed::~ImageFeed() {}

}  // namespace dataio
}  // namespace aliceVision
