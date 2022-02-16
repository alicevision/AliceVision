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
 * @struct ROI
 * @brief Small host / device struct descibing a rectangular 2d / 3d region of interest.
 */
struct ROI
{
    unsigned int beginX = 0;
    unsigned int beginY = 0;
    unsigned int beginZ = 0;
    unsigned int endX = 0;
    unsigned int endY = 0;
    unsigned int endZ = 0;

    ROI() = default;

    CUDA_HOST_DEVICE ROI(unsigned int in_beginX, 
                         unsigned int in_endX,
                         unsigned int in_beginY,
                         unsigned int in_endY,
                         unsigned int in_beginZ,
                         unsigned int in_endZ)
        : beginX(in_beginX)
        , beginY(in_beginY)
        , beginZ(in_beginZ)
        , endX(in_endX)
        , endY(in_endY)
        , endZ(in_endZ)
    {}

    CUDA_HOST_DEVICE ROI(unsigned int in_beginX, 
                         unsigned int in_endX,
                         unsigned int in_beginY,
                         unsigned int in_endY)
        : beginX(in_beginX)
        , beginY(in_beginY)
        , endX(in_endX)
        , endY(in_endY)
    {}

    CUDA_HOST_DEVICE inline unsigned int width()  const { return endX - beginX; }
    CUDA_HOST_DEVICE inline unsigned int height() const { return endY - beginY; }
    CUDA_HOST_DEVICE inline unsigned int depth()  const { return endZ - beginZ; }


    /**
     * @brief Return true if the given 2d point is contained in the ROI.
     * @param[in] x the given 2d point X coordinate
     * @param[in] y the given 2d point Y coordinate
     * @return true if the given 2d point is contained in the ROI
     */
    CUDA_HOST inline bool contains(unsigned int x, unsigned int y) const
    {
        return ((beginX <= x) && (endX > x) && 
                (beginY <= y) && (endY > y));
    }

    /**
     * @brief Return true if the given 3d point is contained in the ROI.
     * @param[in] x the given 3d point X coordinate
     * @param[in] y the given 3d point Y coordinate
     * @param[in] z the given 3d point Z coordinate
     * @return true if the given 3d point is contained in the ROI
     */
    CUDA_HOST inline bool contains(unsigned int x, unsigned int y, unsigned int z) const
    {
        return ((beginX <= x) && (endX > x) && 
                (beginY <= y) && (endY > y) && 
                (beginZ <= z) && (endZ > z));
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
    return ((roi.endX <= volDimX) && (roi.beginX < roi.endX) &&
            (roi.endY <= volDimY) && (roi.beginY < roi.endY) &&
            (roi.endZ <= volDimZ) && (roi.beginZ < roi.endZ));
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
    return ((roi.endX <= (unsigned int)(width))  && (roi.beginX < roi.endX) &&
            (roi.endY <= (unsigned int)(height)) && (roi.beginY < roi.endY) &&
            (roi.beginZ == 0) && (roi.endZ == 0));
}

/**
 * @brief Downscale the given ROI with the given downscale factor
 * @param[in] roi the given ROI
 * @param[in] downscale the downscale factor to apply
 * @return the downscaled ROI
 */
CUDA_HOST inline ROI downscaleROI(const ROI& roi, float downscale)
{ 
    return ROI(CUDA_CEIL(roi.beginX / downscale), CUDA_CEIL(roi.endX / downscale),
               CUDA_CEIL(roi.beginY / downscale), CUDA_CEIL(roi.endY / downscale),
               CUDA_CEIL(roi.beginZ / downscale), CUDA_CEIL(roi.endZ / downscale));
}

} // namespace depthMap
} // namespace aliceVision

