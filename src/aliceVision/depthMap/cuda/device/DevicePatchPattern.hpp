// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

// maximum number of patch pattern subparts
// note: each patch pattern subpart gives one similarity
#define ALICEVISION_DEVICE_PATCH_MAX_SUBPARTS 4

// maximum number of coordinates per patch pattern subpart
#define ALICEVISION_DEVICE_PATCH_MAX_COORDS_PER_SUBPARTS 24

namespace aliceVision {
namespace depthMap {

/**
 * @struct DevicePatchPatternSubpart
 *
 * @brief Support class to maintain a subpart of a patch pattern in gpu constant memory.
 *        Each patch pattern subpart gives one similarity score.
 *
 * @note Should be entirely initialize from host memory.
 *       CUDA doesn't support default initialization for struct in constant memory.
 */
struct DevicePatchPatternSubpart
{
    float2 coordinates[ALICEVISION_DEVICE_PATCH_MAX_COORDS_PER_SUBPARTS];  //< subpart coordinate list
    int nbCoordinates;                                                     //< subpart number of coordinate
    float level;                                                           //< subpart related mipmap level (>=0)
    float downscale;                                                       //< subpart related mipmap downscale (>=1)
    float weight;                                                          //< subpart related similarity weight in range (0, 1)
    bool isCircle;                                                         //< subpart is a circle
    int wsh;                                                               //< subpart half-width (full and circle)
};

/**
 * @struct DevicePatchPattern
 * @brief Support class to maintain a patch pattern in gpu constant memory.
 */
struct DevicePatchPattern
{
    DevicePatchPatternSubpart subparts[ALICEVISION_DEVICE_PATCH_MAX_SUBPARTS];  //< patch pattern subparts (one similarity per subpart)
    int nbSubparts;                                                             //< patch pattern number of subparts (>0)
};

// patch pattern symbol in CUDA constant memory
extern __constant__ DevicePatchPattern constantPatchPattern_d;

}  // namespace depthMap
}  // namespace aliceVision
