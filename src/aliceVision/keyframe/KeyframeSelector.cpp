// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "KeyframeSelector.hpp"
#include <aliceVision/image/all.hpp>
#include <aliceVision/sensorDB/parseDatabase.hpp>
#include <aliceVision/feature/sift/ImageDescriber_SIFT.hpp>
#include <aliceVision/system/Logger.hpp>

#include <boost/filesystem.hpp>

#include <random>
#include <tuple>
#include <cassert>
#include <cstdlib>
#include <iomanip>
namespace fs = boost::filesystem;

namespace aliceVision {
namespace keyframe {


/**
 * @brief Get a random int in order to generate uid.
 * @warning The random don't use a repeatable seed to avoid conflicts between different launches on different data sets.
 * @return int between 0 and std::numeric_limits<int>::max()
 */
int getRandomInt()
{
    std::random_device rd;  // will be used to obtain a seed for the random number engine
    std::mt19937 randomTwEngine(rd()); // standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<> randomDist(0, std::numeric_limits<int>::max());
    return randomDist(randomTwEngine);
}

KeyframeSelector::KeyframeSelector(const std::vector<std::string>& mediaPaths,
                                   const std::string& sensorDbPath,
                                   const std::string& outputFolder)
    : _mediaPaths(mediaPaths)
    , _sensorDbPath(sensorDbPath)
    , _outputFolder(outputFolder)
{
    // Check that a least one media file path has been provided
    if (mediaPaths.empty()) {
        ALICEVISION_THROW(std::invalid_argument, "Cannot create KeyframeSelector without at least one media file path!");
    }
}

void KeyframeSelector::processRegular()
{
    _selectedKeyframes.clear();

    std::size_t nbFrames = std::numeric_limits<unsigned int>::max();
    std::vector<std::unique_ptr<dataio::FeedProvider>> feeds;

    for (std::size_t mediaIndex = 0; mediaIndex < _mediaPaths.size(); ++mediaIndex) {
        const auto& path = _mediaPaths.at(mediaIndex);

        // Create a feed provider per mediaPaths
        feeds.push_back(std::make_unique<dataio::FeedProvider>(path));
        const auto& feed = *feeds.back();

        // Check if feed is initialized
        if (!feed.isInit()) {
            ALICEVISION_THROW(std::invalid_argument, "Cannot initialize the FeedProvider with " << path);
        }

        // Update minimum number of frames
        nbFrames = std::min(nbFrames, static_cast<std::size_t>(feed.nbFrames()));
    }

    // Check if minimum number of frame is zero
    if (nbFrames == 0) {
        ALICEVISION_THROW(std::invalid_argument, "One or multiple medias can't be found or empty!");
    }

    unsigned int step = _minFrameStep;
    if (_maxFrameStep > 0) {
        // By default, if _maxFrameStep is set, set the step to be right between _minFrameStep and _maxFrameStep
        step = step + static_cast<unsigned int>((_maxFrameStep - _minFrameStep) / 2);
    }

    /**
     * To respect the _minFrameStep, _maxFrameStep and _maxOutFrame constraints as much as possible:
     * - if _maxOutFrame is set and the current step is too small to sample over the entire sequence,
     *   the step should be increased;
     * - if _maxOutFrame is set and the adjusted step became too big and does not respect _maxFrameStep anymore,
     *   the step should be set to _maxFrameStep - in that case, _maxOutFrame might be reached before the end of
     *   the sequence
     */
    if (_maxOutFrame > 0 && nbFrames / _maxOutFrame > step) {
        step = (nbFrames / _maxOutFrame) + 1;  // + 1 to prevent ending up with more than _maxOutFrame selected frames
        if (_maxFrameStep > 0 && step > _maxFrameStep) {
            step = _maxFrameStep;
        }
    }

    for (unsigned int id = 0; id < nbFrames; id += step) {
        ALICEVISION_LOG_DEBUG("Selecting frame with ID " << id);
        _selectedKeyframes.push_back(id);
        if (_maxOutFrame > 0 && _selectedKeyframes.size() >= _maxOutFrame)
            break;
    }
}

bool KeyframeSelector::writeSelection(const std::vector<std::string>& brands,
                                      const std::vector<std::string>& models,
                                      const std::vector<float>& mmFocals) const
{
    image::Image<image::RGBColor> image;
    camera::PinholeRadialK3 queryIntrinsics;
    bool hasIntrinsics = false;
    std::string currentImgName;

    for (std::size_t id = 0; id < _mediaPaths.size(); ++id) {
        const auto& path = _mediaPaths.at(id);

        // Create a feed provider per mediaPaths
        dataio::FeedProvider feed(path);

        // Check if feed is initialized
        if (!feed.isInit()) {
            ALICEVISION_LOG_ERROR("Cannot initialize the FeedProvider with " << path);
            return false;
        }

        std::string processedOutputFolder = _outputFolder;
        if (_mediaPaths.size() > 1) {
            const std::string rigFolder = _outputFolder + "/rig/";
            if (!fs::exists(rigFolder)) {
                fs::create_directory(rigFolder);
            }

            processedOutputFolder = rigFolder + std::to_string(id);
            if (!fs::exists(processedOutputFolder)) {
                fs::create_directory(processedOutputFolder);
            }
        }

        for (const auto pos : _selectedKeyframes) {
            if (!feed.goToFrame(pos)) {
                ALICEVISION_LOG_ERROR("Invalid frame position. Ignoring this frame.");
                continue;
            }

            if (!feed.readImage(image, queryIntrinsics, currentImgName, hasIntrinsics)) {
                ALICEVISION_LOG_ERROR("Error reading image");
                return false;
            }

            oiio::ParamValueList metadata;
            metadata.push_back(oiio::ParamValue("Make", brands[id]));
            metadata.push_back(oiio::ParamValue("Model", models[id]));
            metadata.push_back(oiio::ParamValue("Exif:BodySerialNumber", std::to_string(getRandomInt())));
            metadata.push_back(oiio::ParamValue("Exif:FocalLength", mmFocals[id]));
            metadata.push_back(oiio::ParamValue("Exif:ImageUniqueID", std::to_string(getRandomInt())));

            fs::path folder = _outputFolder;
            std::ostringstream filenameSS;
            filenameSS << std::setw(5) << std::setfill('0') << pos << ".exr";
            const auto filepath = (processedOutputFolder / fs::path(filenameSS.str())).string();

            image::ImageWriteOptions options;
            // If the feed is a video, frames are read as OpenCV RGB matrices before being converted to image::ImageRGB
            if (feed.isVideo()) {
                options.fromColorSpace(image::EImageColorSpace::SRGB);
                options.toColorSpace(image::EImageColorSpace::AUTO);
            } else {  // Otherwise, the frames have been read without any conversion, they should be written as such
                if (outputExtension == "exr")
                    options.toColorSpace(image::EImageColorSpace::NO_CONVERSION);
                else
                    options.toColorSpace(image::EImageColorSpace::AUTO);
            }

            image::writeImage(filepath, image, options, metadata);
        }
    }

    return true;
}


} // namespace keyframe 
} // namespace aliceVision
