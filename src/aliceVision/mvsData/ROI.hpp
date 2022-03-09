// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

// allows code sharing between NVCC and other compilers
#if defined(__NVCC__)
#define CUDA_HOST_DEVICE __host__ __device__
#define CUDA_HOST __host__
#define CUDA_CEIL(f)  ceil(f)
#define CUDA_FLOOR(f) floor(f)
#else
#define CUDA_HOST_DEVICE
#define CUDA_HOST
#define CUDA_CEIL(f)  std::ceil(f)
#define CUDA_FLOOR(f) std::floor(f)
#include <cmath>
#endif

namespace aliceVision {
namespace depthMap {

/*
 * @struct Range
 * @brief Small CPU and GPU host / device struct descibing a 1d range.
 */
struct Range
{
    unsigned int begin = 0;
    unsigned int end = 0;

    // default constructor
    Range() = default;

    /**
     * @brief Range constructor
     * @param[in] in_begin the range begin index
     * @param[in] in_end the range end index
     */
    CUDA_HOST_DEVICE Range(unsigned int in_begin, 
                           unsigned int in_end)
        : begin(in_begin)
        , end(in_end)
    {}
    
    /**
     * @brief Return true if the given index is contained in the Range.
     * @param[in] i the given index
     * @return true if the given index point is contained in the Range
     */
    CUDA_HOST_DEVICE inline unsigned int size() const { return end - begin; }

    /**
     * @brief Return true if the given index is contained in the Range.
     * @param[in] i the given index
     * @return true if the given index point is contained in the Range
     */
    CUDA_HOST inline bool contains(unsigned int i) const
    {
        return ((begin <= i) && (end > i));
    }
};

/*
 * @struct ROI
 * @brief Small CPU and GPU host / device struct descibing a rectangular 2d region of interest.
 */
struct ROI
{
    Range x, y;

    // default constructor
    ROI() = default;

    /**
     * @brief ROI constructor
     * @param[in] in_beginX the range X begin index
     * @param[in] in_endX the range X end index
     * @param[in] in_beginY the range Y begin index
     * @param[in] in_endY the range Y end index
     */
    CUDA_HOST_DEVICE ROI(unsigned int in_beginX, 
                         unsigned int in_endX,
                         unsigned int in_beginY,
                         unsigned int in_endY)
        : x(in_beginX, in_endX)
        , y(in_beginY, in_endY)
    {}

    /**
     * @brief ROI constructor
     * @param[in] in_rangeX the X index range
     * @param[in] in_rangeY the Y index range
     */
    CUDA_HOST_DEVICE ROI(const Range& in_rangeX, 
                         const Range& in_rangeY)
        : x(in_rangeX)
        , y(in_rangeY)
    {}

    /**
     * @brief Get the ROI width
     * @return the X range size
     */
    CUDA_HOST_DEVICE inline unsigned int width()  const { return x.size(); }

    /**
     * @brief Get the ROI height
     * @return the Y range size
     */
    CUDA_HOST_DEVICE inline unsigned int height() const { return y.size(); }

    /**
     * @brief Return true if the given 2d point is contained in the ROI.
     * @param[in] in_x the given 2d point X coordinate
     * @param[in] in_y the given 2d point Y coordinate
     * @return true if the given 2d point is contained in the ROI
     */
    CUDA_HOST inline bool contains(unsigned int in_x, unsigned int in_y) const
    {
        return (x.contains(in_x) && y.contains(in_y));
    }
};

/**
 * @brief check if a given ROI is valid and can be contained in a given image
 * @param[in] roi the given ROI
 * @param[in] width the given image width
 * @param[in] height the given image height
 * @return true if valid
 */
CUDA_HOST inline bool checkImageROI(const ROI& roi, int width, int height)
{
    return ((roi.x.end <= (unsigned int)(width))  && (roi.x.begin < roi.x.end) &&
            (roi.y.end <= (unsigned int)(height)) && (roi.y.begin < roi.y.end));
}

/**
 * @brief Downscale the given Range with the given downscale factor
 * @param[in] range the given Range
 * @param[in] downscale the downscale factor to apply
 * @return the downscaled Range
 */
CUDA_HOST inline Range downscaleRange(const Range& range, float downscale)
{
    return Range(CUDA_FLOOR(range.begin / downscale), CUDA_CEIL(range.end / downscale));
}

/**
 * @brief Downscale the given ROI with the given downscale factor
 * @param[in] roi the given ROI
 * @param[in] downscale the downscale factor to apply
 * @return the downscaled ROI
 */
CUDA_HOST inline ROI downscaleROI(const ROI& roi, float downscale)
{ 
    return ROI(downscaleRange(roi.x, downscale), 
               downscaleRange(roi.y, downscale));
}

} // namespace depthMap
} // namespace aliceVision

