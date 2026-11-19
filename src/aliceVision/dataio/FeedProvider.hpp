// This file is part of the AliceVision project.
// Copyright (c) 2015 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "IFeed.hpp"

#include <string>
#include <memory>

namespace aliceVision {
namespace dataio {

/**
 * @brief Open a feed using the first feed registered in FeedFactory that supports the given path.
 * Feeds must have been registered beforehand (e.g. registerImageFeed(), registerVideoFeed(), sfmDataIO::registerSfMDataFeed()).
 */
class FeedProvider
{
  public:
    /**
     * @throw std::invalid_argument if the path is empty or no registered feed supports it
     */
    FeedProvider(const std::string& feedPath, const std::string& calibPath = "");

    /**
     * @brief Provide a new RGB image from the feed.
     *
     * @param[out] imageRGB The new image from the feed.
     * @param[out] camIntrinsics The associated camera intrinsics.
     * @param[out] mediaPath The original media path, for a video is the path to the
     * file, for an image sequence is the path to the single image.
     * @param[out] hasIntrinsics True if \p camIntrinsics is valid, otherwise there
     * is no intrinsics associated to \p imageRGB.
     * @return True if there is a new image, false otherwise.
     */
    bool readImage(image::Image<image::RGBColor>& imageRGB, camera::Pinhole& camIntrinsics, std::string& mediaPath, bool& hasIntrinsics);

    /**
     * @brief Provide a new float grayscale image from the feed.
     *
     * @param[out] imageGray The new image from the feed.
     * @param[out] camIntrinsics The associated camera intrinsics.
     * @param[out] mediaPath The original media path, for a video is the path to the
     * file, for an image sequence is the path to the single image.
     * @param[out] hasIntrinsics True if \p camIntrinsics is valid, otherwise there
     * is no intrinsics associated to \p imageGray.
     * @return True if there is a new image, false otherwise.
     */
    bool readImage(image::Image<float>& imageGray, camera::Pinhole& camIntrinsics, std::string& mediaPath, bool& hasIntrinsics);

    /**
     * @brief Provide a new grayscale image from the feed.
     *
     * @param[out] imageGray The new image from the feed.
     * @param[out] camIntrinsics The associated camera intrinsics.
     * @param[out] mediaPath The original media path, for a video is the path to the
     * file, for an image sequence is the path to the single image.
     * @param[out] hasIntrinsics True if \p camIntrinsics is valid, otherwise there
     * is no intrinsics associated to \p imageGray.
     * @return True if there is a new image, false otherwise.
     */
    bool readImage(image::Image<unsigned char>& imageGray, camera::Pinhole& camIntrinsics, std::string& mediaPath, bool& hasIntrinsics);

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

    virtual ~FeedProvider();

  private:
    std::unique_ptr<IFeed> _feeder;
};

}  // namespace dataio
}  // namespace aliceVision
