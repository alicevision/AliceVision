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
#define CUDA_CEIL(f) ceil(f)
#else
#define CUDA_HOST_DEVICE
#define CUDA_HOST
#define CUDA_CEIL(f) std::ceil(f)
#include <cmath>
#endif

namespace aliceVision {
namespace depthMap {

/*
 * @struct Range
 * @brief Small host / device struct descibing a 1d range.
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
 * @brief Small host / device struct descibing a rectangular 2d / 3d region of interest.
 */
struct ROI
{
    Range x, y, z;

    // default constructor
    ROI() = default;

    CUDA_HOST_DEVICE ROI(unsigned int in_beginX, 
                         unsigned int in_endX,
                         unsigned int in_beginY,
                         unsigned int in_endY,
                         unsigned int in_beginZ,
                         unsigned int in_endZ)
        : x(in_beginX, in_endX)
        , y(in_beginY, in_endY)
        , z(in_beginZ, in_endZ)
    {}

    CUDA_HOST_DEVICE ROI(unsigned int in_beginX, 
                         unsigned int in_endX,
                         unsigned int in_beginY,
                         unsigned int in_endY)
        : x(in_beginX, in_endX)
        , y(in_beginY, in_endY)
    {}

    CUDA_HOST_DEVICE ROI(const Range& rangeX, 
                         const Range& rangeY)
        : x(rangeX)
        , y(rangeY)
    {}

    CUDA_HOST_DEVICE inline unsigned int width()  const { return x.size(); }
    CUDA_HOST_DEVICE inline unsigned int height() const { return y.size(); }
    CUDA_HOST_DEVICE inline unsigned int depth()  const { return z.size(); }


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

    /**
     * @brief Return true if the given 3d point is contained in the ROI.
     * @param[in] in_x the given 3d point X coordinate
     * @param[in] in_y the given 3d point Y coordinate
     * @param[in] in_z the given 3d point Z coordinate
     * @return true if the given 3d point is contained in the ROI
     */
    CUDA_HOST inline bool contains(unsigned int in_x, unsigned int in_y, unsigned int in_z) const
    {
        return (x.contains(in_x) && 
                y.contains(in_y) &&
                z.contains(in_z));
    }
};

/**
 * @brief check if a given 3d ROI is valid and can be contained in a given volume
 * @param[in] roi the given ROI
 * @param[in] volDimX the given volume width
 * @param[in] volDimY the given volume height
 * @param[in] volDimZ the given volume depth
 * @return true if valid
 */
CUDA_HOST inline bool checkVolumeROI(const ROI& roi, size_t volDimX, size_t volDimY, size_t volDimZ)
{
    return ((roi.x.end <= volDimX) && (roi.x.begin < roi.x.end) &&
            (roi.y.end <= volDimY) && (roi.y.begin < roi.y.end) &&
            (roi.z.end <= volDimZ) && (roi.z.begin < roi.z.end));
}

/**
 * @brief check if a given 2d ROI is valid and can be contained in a given image
 * @param[in] roi the given ROI
 * @param[in] width the given image width
 * @param[in] height the given image height
 * @return true if valid
 */
CUDA_HOST inline bool checkImageROI(const ROI& roi, int width, int height)
{
    return ((roi.x.end <= (unsigned int)(width))  && (roi.x.begin < roi.x.end) &&
            (roi.y.end <= (unsigned int)(height)) && (roi.y.begin < roi.y.end) &&
            (roi.z.begin == 0) && (roi.z.end == 0));
}

/**
 * @brief Downscale the given Range with the given downscale factor
 * @param[in] range the given Range
 * @param[in] downscale the downscale factor to apply
 * @return the downscaled Range
 */
CUDA_HOST inline Range downscaleRange(const Range& range, float downscale)
{
    return Range(CUDA_CEIL(range.begin / downscale), 
                 CUDA_CEIL(range.end   / downscale));
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

