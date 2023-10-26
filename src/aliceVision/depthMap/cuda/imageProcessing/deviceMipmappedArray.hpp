// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/cuda/host/memory.hpp>

namespace aliceVision {
namespace depthMap {

/**
 * @brief Create CUDA mipmapped array from a given image buffer.
 * @param[out] out_mipmappedArrayPtr The output allocated and initialized CUDA mipmapped array
 * @param[in] in_img_dmp The given image buffer in device memory (used at level 0)
 * @param[in] levels The number of levels to generate in the CUDA mipmapped array
 */
extern void cuda_createMipmappedArrayFromImage(cudaMipmappedArray_t* out_mipmappedArrayPtr,
                                               const CudaDeviceMemoryPitched<CudaRGBA, 2>& in_img_dmp,
                                               const unsigned int levels);

/**
 * @brief Create CUDA mipmapped array texture object.
 * @note Mipmapped array texture coordinates are normalized (0, 1).
 * @param[out] out_mipmappedArray_texPtr The output CUDA texture object
 * @param[in] in_mipmappedArray The given CUDA mipmapped array in device memory
 * @param[in] levels The number of levels generated in the CUDA mipmapped array
 */
extern void cuda_createMipmappedArrayTexture(cudaTextureObject_t* out_mipmappedArray_texPtr,
                                             const cudaMipmappedArray_t in_mipmappedArray,
                                             const unsigned int levels);

/**
 * @brief Create a debug flat image (of all levels) from a given CUDA mipmapped array texture object.
 * @param[out] out_flatImage_dmp The output debug flat image buffer in device memory
 * @param[in] in_mipmappedArray_tex The CUDA mipmapped array texture object
 * @param[in] levels The number of levels generated in the CUDA mipmapped array
 * @param[in] firstLevelWidth The CUDA mipmapped array level 0 width
 * @param[in] firstLevelHeight TThe CUDA mipmapped array level 0 height
 * @param[in] stream the stream for gpu execution
 */
extern void cuda_createMipmappedArrayDebugFlatImage(CudaDeviceMemoryPitched<CudaRGBA, 2>& out_flatImage_dmp,
                                                    const cudaTextureObject_t in_mipmappedArray_tex,
                                                    const unsigned int levels,
                                                    const int firstLevelWidth,
                                                    const int firstLevelHeight,
                                                    cudaStream_t stream);

}  // namespace depthMap
}  // namespace aliceVision
