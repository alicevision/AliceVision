// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

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

    __host__ __device__ ROI(unsigned int in_beginX, 
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

    __host__ __device__ ROI(unsigned int in_beginX, 
                            unsigned int in_endX,
                            unsigned int in_beginY,
                            unsigned int in_endY)
        : beginX(in_beginX)
        , beginY(in_beginY)
        , endX(in_endX)
        , endY(in_endY)
    {}

    __host__ __device__ inline unsigned int width()  const { return endX - beginX; }
    __host__ __device__ inline unsigned int height() const { return endY - beginY; }
    __host__ __device__ inline unsigned int depth()  const { return endZ - beginZ; }
};

/**
 * @brief check if a given 3d ROI is valid and can be contained in a given volume
 * @param[in] roi the given ROI
 * @param[in] volDimX the given volume width
 * @param[in] volDimY the given volume height
 * @param[in] volDimZ the given volume depth
 * @return true if valid
 */
__host__ inline bool checkVolumeROI(const ROI& roi, size_t volDimX, size_t volDimY, size_t volDimZ)
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
__host__ inline bool checkImageROI(const ROI& roi, int width, int height)
{
    return ((roi.endX <= (unsigned int)(width))  && (roi.beginX < roi.endX) &&
            (roi.endY <= (unsigned int)(height)) && (roi.beginY < roi.endY) &&
            (roi.beginZ == 0) && (roi.endZ == 0));
}

} // namespace depthMap
} // namespace aliceVision

