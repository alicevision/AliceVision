// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/dataio/FeedProvider.hpp>
#include <aliceVision/image/all.hpp>
#include <aliceVision/sensorDB/parseDatabase.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <OpenImageIO/imageio.h>
#include <opencv2/optflow.hpp>
#include <opencv2/imgcodecs.hpp>

#include <string>
#include <map>
#include <mutex>
#include <vector>
#include <memory>
#include <limits>

namespace aliceVision {
namespace image {

template<typename T>
class Image;

} // namespace image

namespace keyframe {

namespace oiio = OIIO;

class KeyframeSelector
{
public:
    /**
     * @brief KeyframeSelector constructor
     * @param[in] mediaPaths video file path, image sequence directory or SfMData file
     * @param[in] maskPaths paths to directories containing masks to apply to input frames
     * @param[in] sensorDbPath camera sensor width database path
     * @param[in] outputFolder output keyframes directory
     * @param[in] outputSfmKeyframes output SfMData file containing the keyframes
     * @param[in] outputSfmFrames output SfMData file containing all the non-selected frames
     */
    KeyframeSelector(const std::vector<std::string>& mediaPaths,
                     const std::vector<std::string>& maskPaths,
                     const std::string& sensorDbPath,
                     const std::string& outputFolder,
                     const std::string& outputSfmKeyframes,
                     const std::string& outputSfmFrames);

    /**
     * @brief KeyframeSelector copy constructor - NO COPY
     * @param[in] copy keyframeSelector
     */
    KeyframeSelector(const KeyframeSelector& copy) = delete;

    /**
     * @brief Process media paths and build a list of selected keyframes using a regular sampling over time
     */
    void processRegular();

    /**
     * @brief Process media paths and build a list of selected keyframes using a smart method based on sharpness
     * and optical flow estimation. The whole process can be described as follows:
     *        - Step 0: compute the sharpness and optical flow scores for all the frames in all the sequences
     *        - Step 1: split the whole sequence into subsequences depending on the accumulated movement ("motion step")
     *        - Step 2: check whether the number of subsequences corresponds to what we want
     *                  - if we do not have enough frames, we reduce the motion step until we get the required
     *                    number of frames
     *                  - if we have too many frames, we increase the motion step until we get the required number of
     *                    frames
     *        - Step 3: for each subsequence, find the frame that best fit both a sharpness criteria (as sharp as
     *                  possible) and a temporal criteria (as in the middle of the subsequence as possible); the goal
     *                  of these criteria is to avoid the following cases:
     *                  - the selected frame is well located temporally but is blurry
     *                  - the selected frame is very sharp but is located at the very beginning or very end of the
     *                    subsequence, meaning that it is likely adjacent to another very sharp frame in another
     *                    subsequence; in that case, we might select two very sharp frames that are consecutive with no
     *                    significant differences in their motion
     *        - Step 4: push the selected frames' IDs
     * @param[in] pxDisplacement in percent, the minimum of displaced pixels in the image since the last selected frame
     * @param[in] rescaledWidthSharpness to resize the input frames to before using them to compute the
     *            sharpness scores (if equal to 0, no rescale will be performed)
     * @param[in] rescaledWidthFlow the width to resize the input frames to before using them to compute the
     *            motion scores (if equal to 0, no rescale will be performed)
     * @param[in] sharpnessWindowSize the size of the sliding window used to compute sharpness scores, in pixels
     * @param[in] flowCellSize the size of the cells within a frame that are used to compute the optical flow scores,
     *            in pixels
     * @param[in] skipSharpnessComputation if true, the sharpness score computations will not be performed and a fixed
     *            sharpness score will be given to all the input frames
     */
    void processSmart(const float pxDisplacement, const std::size_t rescaledWidthSharpness,
                      const std::size_t rescaledWidthFlow, const std::size_t sharpnessWindowSize,
                      const std::size_t flowCellSize, const bool skipSharpnessComputation = false);

    /**
     * @brief Compute the sharpness and optical flow scores for the input media paths
     * @param[in] rescaledWidthSharpness the width to resize the input frames to before using them to compute the
     *            sharpness scores (if equal to 0, no rescale will be performed)
     * @param[in] rescaledWidthFlow the width to resize the input frames to before using them to compute the
     *            motion scores (if equal to 0, no rescale will be performed)
     * @param[in] sharpnessWindowSize the size of the sliding window used to compute sharpness scores, in pixels
     * @param[in] flowCellSize the size of the cells within a frame that are used to compute the optical flow scores,
     *            in pixels
     * @param[in] skipSharpnessComputation if true, the sharpness score computations will not be performed and a fixed
     *            sharpness score will be given to all the input frames
     * @return true if the scores have been successfully computed for all frames, false otherwise
     */
    bool computeScores(const std::size_t rescaledWidthSharpness, const std::size_t rescaledWidthFlow,
                       const std::size_t sharpnessWindowSize, const std::size_t flowCellSize,
                       const bool skipSharpnessComputation);

    /**
     * @brief Write the selected keyframes in the output folder
     * @param[in] brands brand name for each camera
     * @param[in] models model name for each camera
     * @param[in] mmFocals focal in millimeters for each camera
     * @param[in] renameKeyframes name output keyframes as consecutive frames instead of using their index as a name
     * @param[in] outputExtension file extension of the written keyframes
     * @param[in] storageDataType EXR storage data type for the output keyframes (ignored when the extension is not EXR)
     * @return true if all the selected keyframes were successfully written, false otherwise
     */
    bool writeSelection(const std::vector<std::string>& brands, const std::vector<std::string>& models,
                        const std::vector<float>& mmFocals, const bool renameKeyframes,
                        const std::string& outputExtension,
                        const image::EStorageDataType storageDataType = image::EStorageDataType::Undefined);

    /**
     * @brief Export the computed sharpness and optical flow scores to a CSV file
     * @param[in] filename the name of the CSV file (e.g. "scores.csv"), which will be written in the output folder
     * @param[in] exportSelectedFrames add a column with 1s and 0s depending on whether the frame has been selected
     * @return true if the CSV was correctly written to disk, false otherwise
     */
    bool exportScoresToFile(const std::string& filename, const bool exportSelectedFrames = false) const;

    /**
     * @brief Export optical flow HSV visualisation for each frame as a PNG image
     * @param[in] rescaledWidth the width to resize the input frames to before computing the optical flow (if equal
     *            to 0, no rescale will be performed)
     * @return true if the frames have been correctly exported, false otherwise
     */
    bool exportFlowVisualisation(const std::size_t rescaledWidth);

    /**
     * @brief Set the minimum frame step parameter for the processing algorithm
     * @param[in] frameStep minimum number of frames between two keyframes
     */
    void setMinFrameStep(unsigned int frameStep)
    {
        _minFrameStep = frameStep;
    }

    /**
     * @brief Set the maximum frame step parameter for the processing algorithm
     * @param[in] frameStep maximum number of frames between two keyframes
     */
    void setMaxFrameStep(unsigned int frameStep)
    {
        _maxFrameStep = frameStep;
    }

    /**
     * @brief Set the minimum output frame number parameter for the processing algorithm
     * @param[in] nbFrames minimum number of output frames
     */
    void setMinOutFrames(unsigned int nbFrames)
    {
        _minOutFrames = nbFrames;
    }

    /**
     * @brief Set the maximum output frame number parameter for the processing algorithm
     * @param[in] nbFrames maximum number of output frames (if 0, no limit for the regular algorithm)
     */
    void setMaxOutFrames(unsigned int nbFrames)
    {
        _maxOutFrames = nbFrames;
    }

    /**
     * @brief Set the minimum size of the blocks of frames for the multi-threading
     * @param[in] blockSize minimum number of frames in a block for a thread to be spawned
     */
    void setMinBlockSize(std::size_t blockSize)
    {
        _minBlockSize = blockSize;
    }

    /**
     * @brief Get the minimum frame step parameter for the processing algorithm
     * @return minimum number of frames between two keyframes
     */
    unsigned int getMinFrameStep() const
    {
        return _minFrameStep;
    }

    /**
     * @brief Get the maximum output frame number parameter for the processing algorithm
     * @return maximum number of frames between two keyframes
     */
    unsigned int getMaxFrameStep() const
    {
        return _maxFrameStep;
    }

    /**
     * @brief Get the minimum output frame for the processing algorithm
     * @return minimum number of output frames
     */
    unsigned int getMinOutFrames() const
    {
        return _minOutFrames;
    }

    /**
     * @brief Get the maximum output frame number for the processing algorithm
     * @return maximum number of output frames (if 0, no limit for the regular algorithm)
     */
    unsigned int getMaxOutFrames() const
    {
        return _maxOutFrames;
    }
    
private:
    /**
     * @brief Read an image from a feed provider into a grayscale OpenCV matrix, and rescale it if a size is provided
     * @param[in] feed The feed provider
     * @param[in] width The width to resize the input image to. The height will be adjusted with respect to the size ratio.
     *                  There will be no resizing if this parameter is set to 0
     * @return An OpenCV Mat object containing the image
     */
    cv::Mat readImage(dataio::FeedProvider &feed, std::size_t width = 0);


    /**
     * @brief Compute the sharpness and optical flow scores for the input media paths for a given range of frames
     * @param[in] startFrame the index of the first frame to compute the scores for
     * @param[in] endFrame the index of the last frame to compute the scores for
     * @param[in] nbFrames the total number of frames in the sequence
     * @param[in] rescaledWidthSharpness the width to resize the input frames to before using them to compute the
     *            sharpness scores (if equal to 0, no rescale will be performed)
     * @param[in] rescaledWidthFlow the width to resize the input frames to before using them to compute the
     *            motion scores (if equal to 0, no rescale will be performed)
     * @param[in] sharpnessWindowSize the size of the sliding window used to compute sharpness scores, in pixels
     * @param[in] flowCellSize the size of the cells within a frame that are used to compute the optical flow scores,
     *            in pixels
     * @param[in] skipSharpnessComputation if true, the sharpness score computations will not be performed and a fixed
     *            sharpness score will be given to all the input frames
     * @return true if the scores have been successfully computed for all frames, false otherwise
     */
    bool computeScoresProc(const std::size_t startFrame, const std::size_t endFrame, const std::size_t nbFrames,
                           const std::size_t rescaledWidthSharpness, const std::size_t rescaledWidthFlow,
                           const std::size_t sharpnessWindowSize, const std::size_t flowCellSize,
                           const bool skipSharpnessComputation);

    /**
     * @brief Compute the sharpness scores for an input grayscale frame with a sliding window
     * @param[in] grayscaleImage the input grayscale matrix of the frame
     * @param[in] windowSize the size of the sliding window
     * @param[in] mask the mask associated to the input frame if it exists, an empty cv::Mat otherwise
     * @return a double value representing the sharpness score of the sharpest tile in the image
     */
    double computeSharpness(const cv::Mat& grayscaleImage, const std::size_t windowSize, const cv::Mat& mask);

    /**
     * @brief Compute the standard deviation of the local averaged Laplacian in an image
     * @param[in] sum the (masked) integral image of the Laplacian of a given image
     * @param[in] squaredSum the (masked) squared integral image of the Laplacian of a given image
     * @param[in] x the x-coordinate of the top-left corner of the window for the local standard deviation computation
     * @param[in] y the y-coordinate of the top-left corner of the window for the local standard deviation computation
     * @param[in] windowSize the size of the window along the x- and y-axis for the local standard deviation computation
     * @param[in] mask the mask associated to the frame the integral and integral images were calculated from
     * @return a const double value representating the local standard deviation of the Laplacian
     */
    const double computeSharpnessStd(const cv::Mat& sum, const cv::Mat& squaredSum, const int x, const int y,
                                     const int windowSize, const cv::Mat& mask);

    /**
     * @brief Estimate the optical flow score for an input grayscale frame based on its previous frame cell by cell
     * @param[in] ptrFlow the OpenCV's DenseOpticalFlow object
     * @param[in] grayscaleImage the grayscale matrix of the current frame
     * @param[in] previousGrayscaleImage the grayscale matrix of the previous frame
     * @param[in] cellSize the size of the evaluated cells within the frame
     * @param[in] mask the mask associated to the current frame if it exists, an empty cv::Mat otherwise
     * @return a double value representing the median motion of all the image's cells
     */
    double estimateFlow(const cv::Ptr<cv::DenseOpticalFlow>& ptrFlow, const cv::Mat& grayscaleImage,
                        const cv::Mat& previousGrayscaleImage, const std::size_t cellSize, const cv::Mat& mask);

    /**
     * @brief Write the output SfMData files with the selected and non-selected keyframes information
     * @param[in] mediaPath input video file path, image sequence directory or SfMData file
     * @param[in] feed the feed provider
     * @param[in] brands brand name for each camera
     * @param[in] models model name for each camera
     * @param[in] mmFocals focal in millimeters for each camera
     * @return true if the output SfMData files were written as expected, false otherwise
     */
    bool writeSfMData(const std::string& mediaPath, dataio::FeedProvider &feed, const std::vector<std::string>& brands,
                      const std::vector<std::string>& models, const std::vector<float>& mmFocals);

    /**
     * @brief Copy the relevant information from the input SfMData file and fill the output SfMData files that will
     *        contain the selected and non-selected keyframes information (rigs are not supported)
     * @param[in] mediaPath input SfMData file
     * @return true if the output SfMData files have successfully been filled, false otherwise
     */
    bool writeSfMDataFromSfMData(const std::string& mediaPath);

    /**
     * @brief Fill the output SfMData files with the information about the selected keyframes and the non-selected
     *        frames (rigs are supported, but non-selected frames will not be written in the corresponding output
     *        SfMData file if the input is a video)
     * @param mediaPath input video file path or image sequence directory
     * @param feed the feed provider
     * @param brands brand name for each camera
     * @param models model name for each camera
     * @param mmFocals focal in millimiters for each camera
     * @return true if the output SfMData files have successfully been filled, false otherwise
     */
    bool writeSfMDataFromSequences(const std::string& mediaPath, dataio::FeedProvider &feed,
                                   const std::vector<std::string>& brands, const std::vector<std::string>& models,
                                   const std::vector<float>& mmFocals);

    /**
     * @brief Create a View object for the SfMData files
     * @param imagePath the path of the image corresponding to the view to create
     * @param intrinsicId the intrinsic ID for the view to create
     * @param previousFrameId the frame ID of the last created view
     * @param imageWidth the width of the image corresponding to the view to create
     * @param imageHeight the height of the image corresponding to the view to create
     * @return a shared pointer to the created View
     */
    std::shared_ptr<sfmData::View> createView(const std::string& imagePath, IndexT intrinsicId, IndexT previousFrameId,
                    std::size_t imageWidth, std::size_t imageHeight);

    /**
     * @brief Create an Intrinsic object associated to a specific View
     * @param view the View that the intrinsic will be associated to
     * @param focalLength the focal length in millimiter (if 0, default value will be used)
     * @param sensorWidth the sensor width
     * @param imageRatio the width over height ratio for the View's image
     * @param mediaIndex the media index
     * @return a shared pointer to the created Intrinsic
     */
    std::shared_ptr<camera::IntrinsicBase> createIntrinsic(const sfmData::View& view, const double focalLength, const double sensorWidth,
                        const double imageRatio, std::size_t mediaIndex);

    /// Selected keyframes IDs
    std::vector<unsigned int> _selectedKeyframes;

    /// Media paths
    std::vector<std::string> _mediaPaths;
    /// Mask paths
    std::vector<std::string> _maskPaths;
    /// Camera sensor width database
    std::string _sensorDbPath;
    /// Output folder for keyframes
    std::string _outputFolder;
    /// Path of the output SfMData with keyframes
    std::string _outputSfmKeyframesPath;
    /// Path of the output SfMData with non-selected frames
    std::string _outputSfmFramesPath;

    // Parameters common to both the regular and smart methods
    /// Maximum number of output frames (0 = no limit)
    unsigned int _maxOutFrames = 0;

    // Regular algorithm parameters
    /// Minimum number of frames between two keyframes
    unsigned int _minFrameStep = 12;
    /// Maximum number of frames between two keyframes
    unsigned int _maxFrameStep = 36;

    // Smart algorithm parameters
    /// Minimum number of output frames
    unsigned int _minOutFrames = 10;

    /// Minimum block size for multi-threading
    std::size_t _minBlockSize = 10;

    /// Sharpness scores for each frame
    std::map<std::size_t, double> _sharpnessScores;
    /// Optical flow scores for each frame
    std::map<std::size_t, double> _flowScores;
    /// Vector containing 1s for frames that have been selected, 0 for those which have not
    std::vector<char> _selectedFrames;

    /// Output SfMData containing the keyframes
    sfmData::SfMData _outputSfmKeyframes;
    /// Output SfMData containing the non-selected frames
    sfmData::SfMData _outputSfmFrames;

    /// Size of the frame (afer rescale, if any is applied)
    unsigned int _frameWidth = 0;
    unsigned int _frameHeight = 0;

    /// Parsed sensor database
    std::vector<sensorDB::Datasheet> _sensorDatabase;
    bool _parsedSensorDb = false;

    /// Map media path index with names of the output images (used when the input medias are videos)
    std::map<std::size_t, std::vector<std::string>> _keyframesPaths;

    /// Map score vectors with names for export
    std::map<const std::string, const std::map<std::size_t, double>*> scoresMap;

    /// Mutex to ensure thread-safe operations
    mutable std::mutex _mutex;
};

} // namespace keyframe 
} // namespace aliceVision
