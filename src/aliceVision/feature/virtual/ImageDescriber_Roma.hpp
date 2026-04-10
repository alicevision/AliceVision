// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/feature/ImageDescriber.hpp>
#include <aliceVision/feature/regionsFactory.hpp>
#include <aliceVision/types.hpp>

#include <iostream>

namespace aliceVision {
namespace feature {

/**
 * @brief Create an ImageDescriber interface for Roma deep dense feature extractor.
 *
 * This is a "virtual" describer: it does not perform extraction itself but
 * provides the AliceVision interface to load/save pre-computed Roma regions.
 * Actual feature extraction is expected to be performed externally (e.g. via
 * a dedicated Python pipeline) and the resulting .feat/.desc files consumed
 * through this describer.
 */
class ImageDescriber_Roma : public ImageDescriber
{
  public:
    ImageDescriber_Roma() = default;

    ~ImageDescriber_Roma() override = default;

    /**
     * @brief Check if the image describer uses CUDA
     * @return False – extraction is performed externally
     */
    bool useCuda() const override { return false; }

    /**
     * @brief Check if the image describer uses a float image
     * @return False – extraction is performed externally
     */
    bool useFloatImage() const override { return false; }

    /**
     * @brief Get the corresponding EImageDescriberType
     * @return EImageDescriberType::ROMA
     */
    EImageDescriberType getDescriberType() const override { return EImageDescriberType::ROMA; }

    /**
     * @brief Get the total amount of RAM needed for a feature extraction of an
     *        image of the given dimension.
     * @param[in] width  The image width
     * @param[in] height The image height
     * @return total amount of memory needed (conservative upper bound)
     */
    std::size_t getMemoryConsumption(std::size_t width, std::size_t height) const override
    {
        return 3 * width * height * sizeof(unsigned char);
    }

    /**
     * @brief Use a preset to control the number of detected regions
     * @param[in] preset The preset configuration
     */
    void setConfigurationPreset(ConfigurationPreset preset) override;

    /**
     * @brief Detect regions on the 8-bit image and compute their attributes.
     *
     * For this virtual describer the function is a no-op stub: Roma features
     * are expected to be pre-computed externally. Calling this will throw.
     *
     * @param[in]  image   Input 8-bit grayscale image (unused)
     * @param[out] regions Output regions (unused)
     * @param[in]  mask    Optional keypoint mask (unused)
     * @return False – not implemented
     */
    bool describe(const image::Image<unsigned char>& image,
                  std::unique_ptr<Regions>& regions,
                  const image::Image<unsigned char>* mask = nullptr) override;

    /**
     * @brief Allocate Regions type for Roma
     * @param[in,out] regions
     */
    void allocate(std::unique_ptr<Regions>& regions) const override;
};

}  // namespace feature
}  // namespace aliceVision
