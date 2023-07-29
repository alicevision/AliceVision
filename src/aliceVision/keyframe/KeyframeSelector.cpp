// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "KeyframeSelector.hpp"
#include <aliceVision/sfmDataIO/viewIO.hpp>
#include <aliceVision/system/Logger.hpp>

#include <boost/filesystem.hpp>

#include <random>
#include <tuple>
#include <cassert>
#include <cstdlib>
#include <iomanip>
#include <fstream>


namespace fs = boost::filesystem;

namespace aliceVision {
namespace keyframe {

/**
 * @brief Get a random int in order to generate uid.
 * @warning The random doesn't use a repeatable seed to avoid conflicts between different launches on different data sets
 * @return int between 0 and std::numeric_limits<int>::max()
 */
int getRandomInt()
{
    std::random_device rd;  // will be used to obtain a seed for the random number engine
    std::mt19937 randomTwEngine(rd());  // standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<> randomDist(0, std::numeric_limits<int>::max());
    return randomDist(randomTwEngine);
}

/**
 * @brief Find the median value in an unsorted vector of double values.
 * @param[in] vec The unsorted vector of double values
 * @return double the median value
 */
double findMedian(const std::vector<double>& vec)
{
    std::vector<double> vecCopy = vec;
    if (vecCopy.size() > 0 && vecCopy.size() % 2 == 0) {
        const auto medianIt1 = vecCopy.begin() + vecCopy.size() / 2 - 1;
        std::nth_element(vecCopy.begin(), medianIt1, vecCopy.end());
        const auto med1 = *medianIt1;

        const auto medianIt2 = vecCopy.begin() + vecCopy.size() / 2;
        std::nth_element(vecCopy.begin(), medianIt2, vecCopy.end());
        const auto med2 = *medianIt2;

        return (med1 + med2) / 2.0;
    } else if (vecCopy.size() > 0) {
        const auto medianIt = vecCopy.begin() + vecCopy.size() / 2;
        std::nth_element(vecCopy.begin(), medianIt, vecCopy.end());
        return *medianIt;
    }

    return 0.0;
}

KeyframeSelector::KeyframeSelector(const std::vector<std::string>& mediaPaths,
                                   const std::string& sensorDbPath,
                                   const std::string& outputFolder,
                                   const std::string& outputSfmKeyframes,
                                   const std::string& outputSfmFrames)
    : _mediaPaths(mediaPaths)
    , _sensorDbPath(sensorDbPath)
    , _outputFolder(outputFolder)
    , _outputSfmKeyframesPath(outputSfmKeyframes)
    , _outputSfmFramesPath(outputSfmFrames)
{
    // Check that a least one media file path has been provided
    if (mediaPaths.empty()) {
        ALICEVISION_THROW(std::invalid_argument, "Cannot create KeyframeSelector without at least one media file path!");
    }

    scoresMap["Sharpness"] = &_sharpnessScores;
    scoresMap["OpticalFlow"] = &_flowScores;

    // Parse the sensor database if the path is not empty
    if (!_sensorDbPath.empty() && sensorDB::parseDatabase(_sensorDbPath, _sensorDatabase)) {
        _parsedSensorDb = true;
    }
}

void KeyframeSelector::processRegular()
{
    _selectedKeyframes.clear();
    _selectedFrames.clear();
    _keyframesPaths.clear();
    _outputSfmKeyframes.clear();
    _outputSfmFrames.clear();

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

    // All frames are unselected so far
    _selectedFrames.resize(nbFrames);
    std::fill(_selectedFrames.begin(), _selectedFrames.end(), '0');

    unsigned int step = _minFrameStep;
    if (_maxFrameStep > 0) {
        // By default, if _maxFrameStep is set, set the step to be right between _minFrameStep and _maxFrameStep
        step = step + static_cast<unsigned int>((_maxFrameStep - _minFrameStep) / 2);
    }

    /**
     * To respect the _minFrameStep, _maxFrameStep and _maxOutFrames constraints as much as possible:
     * - if _maxOutFrames is set and the current step is too small to sample over the entire sequence,
     *   the step should be increased;
     * - if _maxOutFrames is set and the adjusted step became too big and does not respect _maxFrameStep anymore,
     *   the step should be set to _maxFrameStep - in that case, _maxOutFrames might be reached before the end of
     *   the sequence
     */
    if (_maxOutFrames > 0 && nbFrames / _maxOutFrames > step) {
        step = (nbFrames / _maxOutFrames) + 1;  // + 1 to prevent ending up with more than _maxOutFrame selected frames
        if (_maxFrameStep > 0 && step > _maxFrameStep) {
            step = _maxFrameStep;
        }
    }

    for (unsigned int id = 0; id < nbFrames; id += step) {
        ALICEVISION_LOG_INFO("Selecting frame with ID " << id);
        _selectedKeyframes.push_back(id);
        _selectedFrames.at(id) = '1';
        if (_maxOutFrames > 0 && _selectedKeyframes.size() >= _maxOutFrames)
            break;
    }

    ALICEVISION_LOG_INFO("Finished selecting all the keyframes! " << _selectedKeyframes.size() << "/" <<
                         nbFrames << " frames have been selected.");
}

void KeyframeSelector::processSmart(const float pxDisplacement, const std::size_t rescaledWidthSharpness,
                                    const std::size_t rescaledWidthFlow, const std::size_t sharpnessWindowSize,
                                    const std::size_t flowCellSize, const bool skipSharpnessComputation)
{
    _selectedKeyframes.clear();
    _selectedFrames.clear();
    _keyframesPaths.clear();
    _outputSfmKeyframes.clear();
    _outputSfmFrames.clear();

    // Step 0: compute all the scores
    computeScores(rescaledWidthSharpness, rescaledWidthFlow, sharpnessWindowSize, flowCellSize, skipSharpnessComputation);

    // Step 1: determine subsequences based on the motion accumulation
    std::vector<unsigned int> subsequenceLimits;
    subsequenceLimits.push_back(0);  // Always use the first frame as the starting point

    std::size_t sequenceSize = _sharpnessScores.size();

    // All frames are unselected so far
    _selectedFrames.resize(sequenceSize);
    std::fill(_selectedFrames.begin(), _selectedFrames.end(), '0');

    float step = pxDisplacement * std::min(_frameWidth, _frameHeight) / 100.0;
    double motionAcc = 0.0;

    /* Starts at 1 because the first frame's motion score will be -1.
     * Ends at sequenceSize - 1 to ensure the last frame cannot be pushed twice. */
    for (std::size_t i = 1; i < sequenceSize - 1; ++i) {
        motionAcc += _flowScores.at(i) > -1.f ? _flowScores.at(i) : 0.f;
        if (motionAcc >= step) {
            subsequenceLimits.push_back(i);
            motionAcc = 0.0;  // Reset the motion accumulator
        }
    }
    subsequenceLimits.push_back(sequenceSize - 1);

    // Step 2: check whether the min/max output frames constraints are respected
    if (!(subsequenceLimits.size() - 1 >= _minOutFrames && subsequenceLimits.size() - 1 <= _maxOutFrames)) {
        ALICEVISION_LOG_INFO("Preliminary selection does not provide the right number of frames ("
                             << subsequenceLimits.size() - 1 << " keyframes, should be between " << _minOutFrames
                             << " and " << _maxOutFrames << ").");

        std::vector<unsigned int> newLimits = subsequenceLimits;  // Prevents first 'newLimits.size() - 1' from overflowing
        const double displacementDiff = 0.5;  // The displacement must be 0.5px smaller/bigger than the previous one

        if (subsequenceLimits.size() - 1 < _minOutFrames) {
            // Not enough frames, reduce the motion step
            ALICEVISION_LOG_INFO("Not enough keyframes, the motion step will be reduced of " << displacementDiff
                                 << "%.");
            bool sampleRegularly = false;
            while (newLimits.size() - 1 < _minOutFrames) {
                newLimits.clear();
                newLimits.push_back(0);
                step = std::max(0.0, step - displacementDiff);

                if (step == 0.0) {  // The criterion does not make sense anymore, exit to sample regularly instead
                    sampleRegularly = true;
                    break;
                }
                motionAcc = 0.0;

                for (std::size_t i = 1; i < sequenceSize - 1; ++i) {
                    motionAcc += _flowScores.at(i) > -1.f ? _flowScores.at(i) : 0.f;
                    if (motionAcc >= step) {
                        newLimits.push_back(i);
                        motionAcc = 0.0;
                    }
                }
                newLimits.push_back(sequenceSize - 1);
            }

            if (sampleRegularly) {
                // Sample regularly the whole sequence to get minOutFrames subsequences
                ALICEVISION_LOG_INFO("The motion step has been reduced to 0 and cannot be used anymore. Keyframes will "
                                     "be sampled regularly instead.");
                newLimits.clear();
                newLimits.push_back(0);
                std::size_t stepSize = (sequenceSize / _minOutFrames) + 1;

                for (std::size_t i = 1; i < sequenceSize - 1; i += stepSize)
                    newLimits.push_back(i);
                newLimits.push_back(sequenceSize - 1);
            }
        } else {
            // Too many frames, increase the motion step
            ALICEVISION_LOG_INFO("Too many keyframes, the motion step will be increased of " << displacementDiff
                                 << "%.");
            while (newLimits.size() - 1 > _maxOutFrames) {
                newLimits.clear();
                newLimits.push_back(0);
                step = step + displacementDiff;
                motionAcc = 0.0;

                for (std::size_t i = 1; i < sequenceSize - 1; ++i) {
                    motionAcc += _flowScores.at(i) > -1.f ? _flowScores.at(i) : 0.f;
                    if (motionAcc >= step) {
                        newLimits.push_back(i);
                        motionAcc = 0.0;
                    }
                }
                newLimits.push_back(sequenceSize - 1);
            }
        }

        subsequenceLimits.clear();
        subsequenceLimits = newLimits;
    }

    // Step 3: for each subsequence, find the keyframe
    for (std::size_t i = 1; i < subsequenceLimits.size(); ++i) {
        double bestSharpness = 0.0;
        std::size_t bestIndex = 0;
        std::size_t subsequenceSize = subsequenceLimits.at(i) - subsequenceLimits.at(i - 1);
        ALICEVISION_LOG_DEBUG("Subsequence [" << subsequenceLimits.at(i - 1) << ", " << subsequenceLimits.at(i) << "]");

        // Weights for the whole subsequence [1.0; 2.0] (1.0 is on the subsequence's limits, 2.0 on its center)
        std::deque<double> weights;
        const double weightStep = 1.f / (static_cast<double>(subsequenceSize - 1) / 2.f);
        weights.push_back(2.0);  // The frame in the middle of the subsequence has the biggest weight
        if (subsequenceSize % 2 == 0)
            weights.push_back(2.0);  // For subsequences of even size, two frames are equally in the middle

        float currentWeight = 2.0;
        while (weights.size() != subsequenceSize) {
            currentWeight -= weightStep;
            weights.push_front(currentWeight);
            weights.push_back(currentWeight);
        }

        std::size_t weightPosition = 0;
        for (std::size_t j = subsequenceLimits.at(i - 1); j < subsequenceLimits.at(i); ++j) {
            auto sharpness = _sharpnessScores.at(j) * weights.at(weightPosition);
            ++weightPosition;
            if (sharpness > bestSharpness) {
                bestIndex = j;
                bestSharpness = sharpness;
            }
        }
        ALICEVISION_LOG_INFO("Selecting frame with ID " << bestIndex);
        _selectedKeyframes.push_back(bestIndex);
        _selectedFrames.at(bestIndex) = '1';  // The frame has been selected, flip it to 1
    }

    ALICEVISION_LOG_INFO("Finished selecting all the keyframes! " << _selectedKeyframes.size() << "/" <<
                         sequenceSize << " frames have been selected.");
}

bool KeyframeSelector::computeScores(const std::size_t rescaledWidthSharpness, const std::size_t rescaledWidthFlow,
                                     const std::size_t sharpnessWindowSize, const std::size_t flowCellSize,
                                     const bool skipSharpnessComputation)
{
    // Reset the computed scores
    _sharpnessScores.clear();
    _flowScores.clear();

    // Reset the frame size
    _frameWidth = 0;
    _frameHeight = 0;

    // Create feeds and count minimum number of frames
    std::size_t nbFrames = std::numeric_limits<std::size_t>::max();
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
        nbFrames = std::min(nbFrames, (size_t)feed.nbFrames());
    }

    // Check if minimum number of frame is zero
    if (nbFrames == 0) {
        ALICEVISION_THROW(std::invalid_argument, "One or multiple medias can't be found or is empty!");
    }

    // Feed provider variables
    image::Image<image::RGBColor> image;     // original image
    camera::PinholeRadialK3 queryIntrinsics; // image associated camera intrinsics
    bool hasIntrinsics = false;              // true if queryIntrinsics is valid
    std::string currentImgName;              // current image name

    // Feed and metadata initialization
    for (std::size_t mediaIndex = 0; mediaIndex < feeds.size(); ++mediaIndex) {
        // First frame with offset
        feeds.at(mediaIndex)->goToFrame(0);

        if (!feeds.at(mediaIndex)->readImage(image, queryIntrinsics, currentImgName, hasIntrinsics)) {
            ALICEVISION_THROW(std::invalid_argument, "Cannot read media first frame " << _mediaPaths[mediaIndex]);
        }
    }

    std::size_t currentFrame = 0;
    cv::Mat currentMatSharpness;  // OpenCV matrix for the sharpness computation
    cv::Mat previousMatFlow, currentMatFlow;  // OpenCV matrices for the optical flow computation
    auto ptrFlow = cv::optflow::createOptFlow_DeepFlow();

    while (currentFrame < nbFrames) {
        double minimalSharpness = skipSharpnessComputation ? 1.0f : std::numeric_limits<double>::max();
        double minimalFlow = std::numeric_limits<double>::max();

        for (std::size_t mediaIndex = 0; mediaIndex < feeds.size(); ++mediaIndex) {
            auto& feed = *feeds.at(mediaIndex);

            if (currentFrame > 0) {  // Get currentFrame - 1 for the optical flow computation
                previousMatFlow = readImage(feed, rescaledWidthFlow);
                feed.goToNextFrame();
            }

            /* Handle input feeds that may have invalid or missing frames:
             *   - catch the "invalid argument" exception thrown by "readImage" if a frame is invalid or missing
             *   - try reading the next frame instead
             *   - if the next frame is correctly read, then push dummy scores for the invalid frame and go on with
             *     the process
             *   - otherwise (feed not correctly moved to the next frame), throw a runtime error exception as something
             *     is wrong with the video
             */
            if (!skipSharpnessComputation) {
                try {
                    // Read image for sharpness and rescale it if requested
                    currentMatSharpness = readImage(feed, rescaledWidthSharpness);
                } catch (const std::invalid_argument& ex) {
                    // currentFrame + 1 = currently evaluated frame with indexing starting at 1, for display reasons
                    // currentFrame + 2 = next frame to evaluate with indexing starting at 1, for display reasons
                    ALICEVISION_LOG_WARNING("Invalid or missing frame " << currentFrame + 1
                                            << ", attempting to read frame " << currentFrame + 2 << ".");
                    bool success = feed.goToFrame(++currentFrame);
                    if (success) {
                        // Will throw an exception if next frame is also invalid
                        currentMatSharpness = readImage(feed, rescaledWidthSharpness);
                        // If no exception has been thrown, push dummy scores for the frame that was skipped
                        _sharpnessScores.push_back(-1.f);
                        _flowScores.push_back(-1.f);
                    } else
                        ALICEVISION_THROW_ERROR("Could not go to frame " << currentFrame + 1
                                                << " either. The feed might be corrupted.");
                }
            }

            if (rescaledWidthSharpness == rescaledWidthFlow && !skipSharpnessComputation) {
                currentMatFlow = currentMatSharpness;
            } else {
                currentMatFlow = readImage(feed, rescaledWidthFlow);
            }

            if (_frameWidth == 0 && _frameHeight == 0) {  // Will be used later on to determine the motion accumulation step
                _frameWidth = currentMatFlow.size().width;
                _frameHeight = currentMatFlow.size().height;
            }

            // Compute sharpness
            if (!skipSharpnessComputation) {
                const double sharpness = computeSharpness(currentMatSharpness, sharpnessWindowSize);
                minimalSharpness = std::min(minimalSharpness, sharpness);
            }

            // Compute optical flow
            if (currentFrame > 0) {
                const double flow = estimateFlow(ptrFlow, currentMatFlow, previousMatFlow, flowCellSize);
                minimalFlow = std::min(minimalFlow, flow);
            }

            ALICEVISION_LOG_INFO("Finished processing frame " << currentFrame + 1 << "/" << nbFrames);
        }

        // Save scores for the current frame
        _sharpnessScores.push_back(minimalSharpness);
        _flowScores.push_back(currentFrame > 0 ? minimalFlow : -1.f);
        ++currentFrame;
    }

    return true;
}

bool KeyframeSelector::writeSelection(const std::vector<std::string>& brands,
                                      const std::vector<std::string>& models,
                                      const std::vector<float>& mmFocals,
                                      const bool renameKeyframes,
                                      const std::string& outputExtension,
                                      const image::EStorageDataType storageDataType)
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

        // Ensure that we do want to write the keyframes on disk before going through this
        if (outputExtension != "none") {
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

            unsigned int outputKeyframeCnt = 0;  // Used if the "renameKeyframes" option is enabled
            for (const auto pos : _selectedKeyframes) {
                if (!feed.goToFrame(pos)) {
                    ALICEVISION_LOG_ERROR("Invalid frame position " << pos << ". Ignoring this frame.");
                    continue;
                }

                if (!feed.readImage(image, queryIntrinsics, currentImgName, hasIntrinsics)) {
                    ALICEVISION_LOG_ERROR("Error reading image");
                    return false;
                }

                oiio::ImageSpec inputSpec;
                inputSpec.extra_attribs = image::readImageMetadata(currentImgName);
                int orientation = inputSpec.get_int_attribute("Orientation", 1);
                float pixelAspectRatio = inputSpec.get_float_attribute("PixelAspectRatio", 1.0f);
                std::string colorspace = inputSpec.get_string_attribute("oiio:Colorspace", "");

                oiio::ParamValueList metadata;
                metadata.push_back(oiio::ParamValue("Make", brands[id]));
                metadata.push_back(oiio::ParamValue("Model", models[id]));
                metadata.push_back(oiio::ParamValue("Exif:BodySerialNumber", std::to_string(getRandomInt())));
                metadata.push_back(oiio::ParamValue("Exif:FocalLength", mmFocals[id]));
                metadata.push_back(oiio::ParamValue("Exif:ImageUniqueID", std::to_string(getRandomInt())));
                metadata.push_back(oiio::ParamValue("Orientation", orientation));  // Will not propagate for PNG outputs
                if (outputExtension != "jpg")  // TODO: propagate pixelAspectRatio properly for JPG
                    metadata.push_back(oiio::ParamValue("PixelAspectRatio", pixelAspectRatio));

                fs::path folder = _outputFolder;
                std::ostringstream filenameSS;
                if (renameKeyframes)
                    filenameSS << std::setw(5) << std::setfill('0') << outputKeyframeCnt++ << "." << outputExtension;
                else
                    filenameSS << std::setw(5) << std::setfill('0') << pos << "." << outputExtension;
                const auto filepath = (processedOutputFolder / fs::path(filenameSS.str())).string();

                image::ImageWriteOptions options;
                // If the feed is a video, frames are read as OpenCV RGB matrices before being converted to image::ImageRGB
                if (feed.isVideo()) {
                    options.fromColorSpace(image::EImageColorSpace::SRGB);
                    options.toColorSpace(image::EImageColorSpace::AUTO);
                } else {  // Otherwise, the frames have been read without any conversion, they should be written as such
                    if (colorspace == "sRGB")
                            options.fromColorSpace(image::EImageColorSpace::SRGB);

                    if (outputExtension == "exr")
                        options.toColorSpace(image::EImageColorSpace::NO_CONVERSION);
                    else
                        options.toColorSpace(image::EImageColorSpace::AUTO);
                }

                if (storageDataType != image::EStorageDataType::Undefined && outputExtension == "exr") {
                    options.storageDataType(storageDataType);
                }

                image::writeImage(filepath, image, options, metadata);
                ALICEVISION_LOG_DEBUG("Wrote selected keyframe " << pos);

                _keyframesPaths[id].push_back(filepath);
            }
        }

        // If the current media is a video and there is no output keyframe, the corresponding SfMData file will not be written
        if (feed.isVideo() && outputExtension == "none") {
            ALICEVISION_THROW(std::invalid_argument, "The keyframes selected from the input video have not been " <<
                              "written on disk. The keyframes' SfMData file cannot be written.");
        }

        if (!writeSfMData(path, feed, brands, models, mmFocals))
            ALICEVISION_LOG_ERROR("Failed to write the output SfMData files.");
    }

    return true;
}

bool KeyframeSelector::exportScoresToFile(const std::string& filename, const bool exportSelectedFrames) const
{
    std::size_t sequenceSize = scoresMap.begin()->second->size();
    if (sequenceSize == 0) {
        ALICEVISION_LOG_ERROR("Nothing to export, scores do not seem to have been computed!");
        return false;
    }

    std::ofstream os;
    os.open((fs::path(_outputFolder) / filename).string(), std::ios::app);

    if (!os.is_open()) {
        ALICEVISION_LOG_ERROR("Unable to open the scores file: " << filename << ".");
        return false;
    }

    ALICEVISION_LOG_DEBUG("Exporting scores as CSV file: " << filename << " (export selected frames: "
                          << exportSelectedFrames << ")");

    os.seekp(0, std::ios::end);  // Put the cursor at the end of the file
    if (os.tellp() == std::streampos(0)) {  // 'tellp' returns the cursor's position
        // If the file does not exist yet, add a header
        std::string header = "FrameNb;";
        for (const auto& mapIterator : scoresMap)
            header += mapIterator.first + ";";

        if (exportSelectedFrames)
            header += "Selected;";

        os << header << "\n";
    }

    for (std::size_t index = 0; index < sequenceSize; ++index) {
        os << index << ";";  // First column: frame index

        for (const auto& mapIterator : scoresMap)
            os << mapIterator.second->at(index) << ";";
        if (exportSelectedFrames)
            os << _selectedFrames.at(index);
        os << "\n";
    }

    os.close();
    return true;
}

bool KeyframeSelector::exportFlowVisualisation(const std::size_t rescaledWidth)
{
    // Create feeds and count minimum number of frames
    std::size_t nbFrames = std::numeric_limits<std::size_t>::max();
    std::vector<std::unique_ptr<dataio::FeedProvider>> feeds;
    std::vector<std::string> outputFolders;

    for (std::size_t mediaIndex = 0; mediaIndex < _mediaPaths.size(); ++mediaIndex) {
        const auto& path = _mediaPaths.at(mediaIndex);

        // Create a feed provider per mediaPaths
        feeds.emplace_back(new dataio::FeedProvider(path));
        auto& feed = *feeds.back();

        // Check if feed is initialized
        if (!feed.isInit()) {
            ALICEVISION_LOG_ERROR("Cannot initialize the FeedProvider with " << path);
            return false;
        }

        feed.goToFrame(0);

        // Update minimum number of frames
        nbFrames = std::min(nbFrames, (size_t)feed.nbFrames());

        // If there is a rig, create the corresponding folders
        std::string processedOutputFolder = _outputFolder;
        if (_mediaPaths.size() > 1) {
            const std::string rigFolder = _outputFolder + "/rig/";
            if (!fs::exists(rigFolder)) {
                fs::create_directory(rigFolder);
            }

            processedOutputFolder = rigFolder + std::to_string(mediaIndex);
            if (!fs::exists(processedOutputFolder)) {
                fs::create_directory(processedOutputFolder);
            }
        }

        // Save the output paths
        outputFolders.push_back(processedOutputFolder);
    }

    if (nbFrames == 0) {
        ALICEVISION_LOG_ERROR("No frame to visualise optical flow from!");
        return false;
    }

    size_t currentFrame = 0;
    cv::Mat previousMat, currentMat;  // OpenCV matrices for the optical flow computation
    auto ptrFlow = cv::optflow::createOptFlow_DeepFlow();

    /* To be able to handle the rigs and to avoid storing the optical flow results for all frames in case
     * we might want to export them, we need to recompute the optical flow for all the frames, even if it has already
     * been computed in computeScores(). */
    while (currentFrame < nbFrames) {
        for (std::size_t mediaIndex = 0; mediaIndex < feeds.size(); ++mediaIndex) {
            auto& feed = *feeds.at(mediaIndex);

            if (currentFrame > 0) {  // Get currentFrame - 1 for the optical flow computation
                previousMat = readImage(feed, rescaledWidth);
                feed.goToNextFrame();
            }

            // Handle invalid or missing frames
            try {
                currentMat = readImage(feed, rescaledWidth);  // Read image and rescale it if requested
            } catch (const std::invalid_argument& ex) {
                ALICEVISION_LOG_WARNING("Invalid or missing frame " << currentFrame + 1
                                        << ", attempting to read frame " << currentFrame + 2 << ".");
                bool success = feed.goToFrame(++currentFrame);
                if (success)
                    currentMat = readImage(feed, rescaledWidth);
                else
                    ALICEVISION_THROW_ERROR("Could not go to frame " << currentFrame + 1
                                            << " either. The feed might be corrupted.");
            }

            if (currentFrame > 0) {
                cv::Mat flow;
                ptrFlow->calc(currentMat, previousMat, flow);

                cv::Mat flowParts[2];
                cv::split(flow, flowParts);
                cv::Mat magnitude, angle, magnNorm;
                cv::cartToPolar(flowParts[0], flowParts[1], magnitude, angle, true);
                cv::normalize(magnitude, magnNorm, 0.0f, 1.0f, cv::NORM_MINMAX);
                angle *= ((1.f / 360.f) * (180.f / 255.f));

                cv::Mat _hsv[3], hsv, hsv8, bgr;
                _hsv[0] = angle;
                _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
                _hsv[2] = magnNorm;
                cv::merge(_hsv, 3, hsv);
                hsv.convertTo(hsv8, CV_8U, 255.0);
                cv::cvtColor(hsv8, bgr, cv::COLOR_HSV2BGR);

                std::ostringstream filenameSS;
                filenameSS << std::setw(5) << std::setfill('0') << currentFrame << ".png";
                cv::imwrite(outputFolders.at(mediaIndex) + "/OF_" + filenameSS.str(), bgr);
                ALICEVISION_LOG_DEBUG("Wrote OF_" << filenameSS.str() << "!");
            }
        }
        ++currentFrame;
    }

    return true;
}

cv::Mat KeyframeSelector::readImage(dataio::FeedProvider &feed, std::size_t width)
{
    image::Image<image::RGBColor> image;
    camera::PinholeRadialK3 queryIntrinsics;
    bool hasIntrinsics = false;
    std::string currentImgName;

    if (!feed.readImage(image, queryIntrinsics, currentImgName, hasIntrinsics)) {
        ALICEVISION_THROW(std::invalid_argument, "Cannot read frame '" << currentImgName << "'!");
    }

    // Convert content to OpenCV
    cv::Mat cvFrame(cv::Size(image.cols(), image.rows()), CV_8UC3, image.data(), image.cols() * 3);

    // Convert to grayscale
    cv::Mat cvGrayscale;
    cv::cvtColor(cvFrame, cvGrayscale, cv::COLOR_BGR2GRAY);

    // Resize to smaller size if requested
    if (width == 0)
        return cvGrayscale;

    cv::Mat cvRescaled;
    if (cvGrayscale.cols > width && width > 0) {
        cv::resize(cvGrayscale, cvRescaled,
                   cv::Size(width, double(cvGrayscale.rows) * double(width) / double(cvGrayscale.cols)));
    }

    return cvRescaled;
}

double KeyframeSelector::computeSharpness(const cv::Mat& grayscaleImage, const std::size_t windowSize)
{
    if (windowSize > grayscaleImage.size().width || windowSize > grayscaleImage.size().height) {
        ALICEVISION_THROW(std::invalid_argument,
                          "Cannot use a sliding window bigger than the image (sliding window size: "
                          << windowSize << ", image size: " << grayscaleImage.size().width << "x"
                          << grayscaleImage.size().height << ")");
    }

    cv::Mat sum, squaredSum, laplacian;
    cv::Laplacian(grayscaleImage, laplacian, CV_64F);
    cv::integral(laplacian, sum, squaredSum);

    double totalCount = windowSize * windowSize;
    double maxstd = 0.0;

    // TODO: do not slide the window pixel by pixel to speed up computations
    // Starts at 1 because the integral image is padded with 0s on the top and left borders
    for (int y = 1; y < sum.rows - windowSize; ++y) {
        for (int x = 1; x < sum.cols - windowSize; ++x) {
            double tl = sum.at<double>(y, x);
            double tr = sum.at<double>(y, x + windowSize);
            double bl = sum.at<double>(y + windowSize, x);
            double br = sum.at<double>(y + windowSize, x + windowSize);
            const double s1 = br + tl - tr - bl;

            tl = squaredSum.at<double>(y, x);
            tr = squaredSum.at<double>(y, x + windowSize);
            bl = squaredSum.at<double>(y + windowSize, x);
            br = squaredSum.at<double>(y + windowSize, x + windowSize);
            const double s2 = br + tl - tr - bl;

            const double std2 = std::sqrt((s2 - (s1 * s1) / totalCount) / totalCount);
            maxstd = std::max(maxstd, std2);
        }
    }

    return maxstd;
}

double KeyframeSelector::estimateFlow(const cv::Ptr<cv::DenseOpticalFlow>& ptrFlow, const cv::Mat& grayscaleImage,
                                      const cv::Mat& previousGrayscaleImage, const std::size_t cellSize)
{
    if (cellSize > grayscaleImage.size().width) {  // If the cell size is bigger than the height, it will be adjusted
        ALICEVISION_THROW(std::invalid_argument,
                          "Cannot use a cell size bigger than the image's width (cell size: " << cellSize
                          << ", image's width: " << grayscaleImage.size().width << ")");
    }

    if (grayscaleImage.size() != previousGrayscaleImage.size()) {
        ALICEVISION_THROW(std::invalid_argument,
                          "The images used for the optical flow computation have different sizes ("
                          << grayscaleImage.size().width << "x" << grayscaleImage.size().height << " and "
                          << previousGrayscaleImage.size().width << "x" << previousGrayscaleImage.size().height
                          << ")");
    }

    cv::Mat flow;
    ptrFlow->calc(grayscaleImage, previousGrayscaleImage, flow);

    cv::Mat sumflow;
    cv::integral(flow, sumflow, CV_64F);

    double norm;
    std::vector<double> motionByCell;

    // Starts at 1 because the integral matrix is padded with 0s on the top and left borders
    for (std::size_t y = 1; y < sumflow.size().height; y += cellSize) {
        std::size_t maxCellSizeHeight = cellSize;
        if (std::min(sumflow.size().height, int(y + cellSize)) == sumflow.size().height)
            maxCellSizeHeight = sumflow.size().height - y;

        for (std::size_t x = 1; x < sumflow.size().width; x += cellSize) {
            std::size_t maxCellSizeWidth = cellSize;
            if (std::min(sumflow.size().width, int(x + cellSize)) == sumflow.size().width)
                maxCellSizeWidth = sumflow.size().width - x;
            cv::Point2d tl = sumflow.at<cv::Point2d>(y, x);
            cv::Point2d tr = sumflow.at<cv::Point2d>(y, x + maxCellSizeWidth - 1);
            cv::Point2d bl = sumflow.at<cv::Point2d>(y + maxCellSizeHeight - 1, x);
            cv::Point2d br = sumflow.at<cv::Point2d>(y + maxCellSizeHeight - 1, x + maxCellSizeWidth - 1);
            cv::Point2d s = br + tl - tr - bl;
            norm = std::hypot(s.x, s.y) / (maxCellSizeHeight * maxCellSizeWidth);
            motionByCell.push_back(norm);
        }
    }

    return findMedian(motionByCell);
}

bool KeyframeSelector::writeSfMData(const std::string& mediaPath, dataio::FeedProvider &feed,
                                    const std::vector<std::string>& brands, const std::vector<std::string>& models,
                                    const std::vector<float>& mmFocals)
{
    bool filledOutputs = false;

    if (!feed.isSfMData()) {
        filledOutputs = writeSfMDataFromSequences(mediaPath, feed, brands, models, mmFocals);
    } else {
        filledOutputs = writeSfMDataFromSfMData(mediaPath);
    }

    if (!filledOutputs) {
        ALICEVISION_LOG_ERROR("Error while filling the output SfMData files.");
        return false;
    }

    if (!sfmDataIO::Save(_outputSfmKeyframes, _outputSfmKeyframesPath, sfmDataIO::ESfMData::ALL)) {
        ALICEVISION_LOG_ERROR("The output SfMData file '" << _outputSfmKeyframesPath << "' could not be written.");
        return false;
    }

    if (!feed.isVideo()) {
        if (!sfmDataIO::Save(_outputSfmFrames, _outputSfmFramesPath, sfmDataIO::ESfMData::ALL)) {
            ALICEVISION_LOG_ERROR("The output SfMData file '" << _outputSfmFramesPath << "' could not be written.");
            return false;
        }
    } else {
        ALICEVISION_LOG_DEBUG("The input feed is a video. The SfMData file containing the unselected frames will not"
                              " be written.");
    }

    return true;
}

bool KeyframeSelector::writeSfMDataFromSfMData(const std::string& mediaPath)
{
    auto& keyframesViews = _outputSfmKeyframes.getViews();
    auto& framesViews = _outputSfmFrames.getViews();

    auto& keyframesIntrinsics = _outputSfmKeyframes.getIntrinsics();
    auto& framesIntrinsics = _outputSfmFrames.getIntrinsics();

    IndexT viewId;
    IndexT intrinsicId;

    sfmData::SfMData inputSfm;
    std::vector<std::shared_ptr<sfmData::View>> views;
    if (!sfmDataIO::Load(inputSfm, mediaPath, sfmDataIO::ESfMData::ALL)) {
        ALICEVISION_LOG_ERROR("Could not open input SfMData file " << mediaPath << ".");
        return false;
    }

    // Order the views according to the frame ID and the intrinsics serial number
    std::map<std::string, std::vector<std::shared_ptr<sfmData::View>>> viewSequences;
    auto& intrinsics = inputSfm.getIntrinsics();
    for(auto it = inputSfm.getViews().begin(); it != inputSfm.getViews().end(); ++it) {
        auto view = it->second;
        auto serialNumber = intrinsics[view->getIntrinsicId()]->serialNumber();
        viewSequences[serialNumber].push_back(view);
    }

    // Sort the views with the same intrinsics together based on their frame ID and add them to the final global vector
    for(auto& view : viewSequences) {
        std::sort(view.second.begin(), view.second.end(),
                  [](std::shared_ptr<sfmData::View> v1, std::shared_ptr<sfmData::View> v2) {
                    return v1->getFrameId() < v2->getFrameId();
                  });
        views.insert(views.end(), view.second.begin(), view.second.end());
    }

    for (int i = 0; i < views.size(); ++i) {
        viewId = views[i]->getViewId();
        intrinsicId = views[i]->getIntrinsicId();
        if (_selectedFrames[i] == '1') {
            keyframesViews[viewId] = views[i];
            keyframesIntrinsics[intrinsicId] = inputSfm.getIntrinsics()[intrinsicId];
        } else {
            framesViews[viewId] = views[i];
            framesIntrinsics[intrinsicId] = inputSfm.getIntrinsics()[intrinsicId];
        }
    }

    return true;
}

bool KeyframeSelector::writeSfMDataFromSequences(const std::string& mediaPath, dataio::FeedProvider &feed,
                                                 const std::vector<std::string>& brands,
                                                 const std::vector<std::string>& models,
                                                 const std::vector<float>& mmFocals)
{
    static std::size_t mediaIndex = 0;
    static IndexT intrinsicId = 0;

    auto& keyframesViews = _outputSfmKeyframes.getViews();
    auto& framesViews = _outputSfmFrames.getViews();

    auto& keyframesIntrinsics = _outputSfmKeyframes.getIntrinsics();
    auto& framesIntrinsics = _outputSfmFrames.getIntrinsics();

    auto& keyframesRigs = _outputSfmKeyframes.getRigs();
    auto& framesRigs = _outputSfmFrames.getRigs();

    const IndexT rigId = 0;   // 0 by convention
    IndexT viewId = 0;  // Will be used to distinguish frames coming from videos
    IndexT previousFrameId = UndefinedIndexT;

    feed.goToFrame(0);

    // Feed provider variables
    image::Image<image::RGBColor> image;
    camera::PinholeRadialK3 queryIntrinsics;
    bool hasIntrinsics = false;
    std::string currentImgName;

    std::size_t selectedKeyframesCounter = 0;  // Used to find the path of the written images when the feed is video
    std::shared_ptr<camera::IntrinsicBase> previousIntrinsic = nullptr;

    // Create rig structure if it is needed and does not already exist
    // A rig structure is needed when there is more than one input path
    if (_mediaPaths.size() > 1 && keyframesRigs.size() == 0 && framesRigs.size() == 0) {
        sfmData::Rig rig(_mediaPaths.size());
        keyframesRigs[rigId] = rig;
        framesRigs[rigId] = rig;
    }

    for (std::size_t i = 0; i < feed.nbFrames(); ++i) {
        // Need to read the image to get its size and path
        if (!feed.readImage(image, queryIntrinsics, currentImgName, hasIntrinsics)) {
            ALICEVISION_LOG_ERROR("Error reading image");
            return false;
        }

        // Create the view
        auto view = createView(currentImgName, intrinsicId, previousFrameId, image.Width(), image.Height());
        previousFrameId = view->getFrameId();

        // If there is a rig, the view's rig and sub-pose IDs need to be set once it has been completed
        if (keyframesRigs.size() > 0 && framesRigs.size() > 0) {
            view->setRigAndSubPoseId(rigId, mediaIndex);
        }

        // Prepare settings for the intrinsics
        double focalLength = view->getMetadataFocalLength();
        if (focalLength == -1 && mmFocals[mediaIndex] != 0)
            focalLength = mmFocals[mediaIndex];
        std::string make = view->getMetadataMake();
        if (make.empty() && !brands[mediaIndex].empty())
            make = brands[mediaIndex];
        std::string model = view->getMetadataModel();
        if (model.empty() && !models[mediaIndex].empty())
            model = models[mediaIndex];

        const double imageRatio = static_cast<double>(image.Width()) / static_cast<double>(image.Height());
        double sensorWidth = -1.0;
        sensorDB::Datasheet datasheet;

        if (_parsedSensorDb && !make.empty() && !model.empty() &&
            sensorDB::getInfo(make, model, _sensorDatabase, datasheet)) {
            sensorWidth = datasheet._sensorWidth;
        }

        // Create the intrinsic for the view
        auto intrinsic = createIntrinsic(*view, focalLength == -1.0 ? 0 : focalLength, sensorWidth, mediaIndex,
                                         imageRatio);

        // Update intrinsics ID if this is a new one
        if (previousIntrinsic != nullptr && *previousIntrinsic != *intrinsic)
            view->setIntrinsicId(++intrinsicId);

        // Fill the SfMData files
        if (_selectedFrames[i] == '1') {
            if (feed.isVideo()) {
                // If the feed is a video, all views will have the same view ID by default, this needs to be fixed
                view->setViewId(view->getViewId() + viewId++);
                view->setPoseId(view->getViewId());
                // The path for the view will be the video's; it needs to be replaced with the corresponding keyframe's
                view->setImagePath(_keyframesPaths[mediaIndex][selectedKeyframesCounter++]);
            }
            keyframesViews[view->getViewId()] = view;
            keyframesIntrinsics[intrinsicId] = intrinsic;
        } else {
            // No rejected frames if the feed is a video one, as they are not written on disk
            if (!feed.isVideo()) {
                framesViews[view->getViewId()] = view;
                framesIntrinsics[intrinsicId] = intrinsic;
            }
        }

        previousIntrinsic = intrinsic;
        feed.goToNextFrame();
    }

    ++mediaIndex;
    ++intrinsicId;

    return true;
}

std::shared_ptr<sfmData::View> KeyframeSelector::createView(const std::string& imagePath, IndexT intrinsicId,
                                IndexT previousFrameId, std::size_t imageWidth, std::size_t imageHeight)
{
    // Create the View object: most attributes are set with default values and will be updated later on
    auto view = std::make_shared<sfmData::View>(
        imagePath,                           // filepath
        UndefinedIndexT,                     // view ID
        intrinsicId,                         // intrinsics ID
        UndefinedIndexT,                     // pose ID
        imageWidth,                          // image width
        imageHeight,                         // image height
        UndefinedIndexT,                     // rig ID
        UndefinedIndexT,                     // sub-pose ID
        std::map<std::string, std::string>() // metadata
    );

    // Complete the View attributes
    sfmDataIO::EViewIdMethod viewIdMethod = sfmDataIO::EViewIdMethod::METADATA;
    std::string viewIdRegex = ".*?(\\d+)";
    sfmDataIO::updateIncompleteView(*(view.get()), viewIdMethod, viewIdRegex);

    // Set the frame ID
    IndexT frameId;
    std::string prefix;
    std::string suffix;
    // Use the filename to determine the frame ID (if available)
    if (sfmDataIO::extractNumberFromFileStem(fs::path(view->getImagePath()).stem().string(), frameId, prefix, suffix)) {
        view->setFrameId(frameId);
    }
    // Otherwise, set it fully manually
    if (view->getFrameId() == 1 && previousFrameId != UndefinedIndexT) {
        view->setFrameId(previousFrameId + 1);
    }

    return view;
}

std::shared_ptr<camera::IntrinsicBase> KeyframeSelector::createIntrinsic(const sfmData::View& view, const double focalLength,
                const double sensorWidth, const double imageRatio, const std::size_t mediaIndex)
{
    auto intrinsic = sfmDataIO::getViewIntrinsic(view, focalLength, sensorWidth);
    if (imageRatio > 1.0 && sensorWidth > -1.0) {
        intrinsic->setSensorWidth(sensorWidth);
        intrinsic->setSensorHeight(sensorWidth / imageRatio);
    } else if (imageRatio <= 1.0 && sensorWidth > -1.0) {
        intrinsic->setSensorWidth(sensorWidth);
        intrinsic->setSensorHeight(sensorWidth * imageRatio);
    } else {
        // Set default values for the sensor width and sensor height
        intrinsic->setSensorWidth(36.0);
        intrinsic->setSensorHeight(24.0);
    }

    if (intrinsic->serialNumber().empty())  // Likely to happen with video feeds
        intrinsic->setSerialNumber(fs::path(view.getImagePath()).parent_path().string() + std::to_string(mediaIndex));

    return intrinsic;
}

} // namespace keyframe 
} // namespace aliceVision
