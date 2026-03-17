// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/imageProcessing/imageProcessing.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>

namespace aliceVision {
namespace imageProcessing {

/**
 * @brief Applies a bilateral filter to the image using OpenCV.
 *
 * The bilateral filter smooths the image while preserving edges by combining
 * a spatial Gaussian filter with a range (intensity) Gaussian filter. Pixels
 * that are both spatially close and have similar intensity are averaged together,
 * making it effective for noise reduction without blurring sharp edges.
 */
class BilateralFilterProcess : public ImageProcess
{
public:
    /**
     * @brief Construct a bilateral filter process.
     * @param[in] distance   Diameter of each pixel neighbourhood (0 = auto-computed from sigmaSpace).
     * @param[in] sigmaColor Filter sigma in the colour/intensity space; larger values mix more distant colours.
     * @param[in] sigmaSpace Filter sigma in the coordinate space; larger values include more distant pixels.
     */
    BilateralFilterProcess(int distance = 0, float sigmaColor = 0.0f, float sigmaSpace = 0.0f)
        : _distance(distance),
          _sigmaColor(sigmaColor),
          _sigmaSpace(sigmaSpace)
    {}

private:
    bool processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera, image::Image<image::RGBAfColor> & image, bool dryRun);

private:
    int _distance = 0;          ///< Pixel neighbourhood diameter.
    float _sigmaColor = 0.0f;   ///< Sigma for the colour/intensity Gaussian.
    float _sigmaSpace = 0.0f;   ///< Sigma for the spatial Gaussian.
};

/**
 * @brief Applies Contrast-Limited Adaptive Histogram Equalisation (CLAHE) using OpenCV.
 *
 * CLAHE divides the image into tiles and equalises the histogram of each tile
 * independently, with a contrast limit to prevent over-amplification of noise.
 * This improves local contrast and is particularly useful for images with
 * uneven illumination or low dynamic range.
 */
class ClaheFilterProcess : public ImageProcess
{
public:
    /**
     * @brief Construct a CLAHE filter process.
     * @param[in] tileGridSize Size of the grid of tiles (e.g. 8 means an 8×8 grid).
     * @param[in] clipLimit    Contrast limit threshold for histogram clipping per tile.
     */
    ClaheFilterProcess(size_t tileGridSize = 8, size_t clipLimit = 4)
        : _tileGridSize(tileGridSize),
          _clipLimit(clipLimit)
    {}

private:
    bool processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera, image::Image<image::RGBAfColor> & image, bool dryRun);

private:
    size_t _tileGridSize = 8;  ///< Number of tiles along each dimension.
    size_t _clipLimit = 4;     ///< Contrast limit for histogram clipping.
};

/**
 * @brief Applies Non-Local Means (NLM) denoising using OpenCV.
 *
 * NLM denoising replaces each pixel with a weighted average of pixels from
 * the surrounding search window whose template patches are similar. It is
 * highly effective at removing Gaussian noise while preserving texture and
 * fine details better than local filters. The colour variant processes
 * luminance and chrominance channels with independent filter strengths.
 */
class NlmFilterProcess : public ImageProcess
{
public:
    /**
     * @brief Construct a Non-Local Means denoising process.
     * @param[in] filterStrength      Luminance denoising strength; higher values remove more noise but may lose detail.
     * @param[in] filterStrengthColor Chrominance denoising strength; higher values remove more colour noise.
     * @param[in] templateWindowSize  Size of the template patch (should be odd, e.g. 7).
     * @param[in] searchWindowSize    Size of the search area around each pixel (should be odd, e.g. 21).
     */
    NlmFilterProcess(float filterStrength = 5.0f, float filterStrengthColor = 10.0f, size_t templateWindowSize = 7, size_t searchWindowSize = 21)
        : _filterStrength(filterStrength),
          _filterStrengthColor(filterStrengthColor),
          _templateWindowSize(templateWindowSize),
          _searchWindowSize(searchWindowSize)
    {}

private:
    bool processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera, image::Image<image::RGBAfColor> & image, bool dryRun);

private:
    float _filterStrength = 5.0f;        ///< Luminance denoising strength.
    float _filterStrengthColor = 10.0f;  ///< Chrominance denoising strength.
    size_t _templateWindowSize = 7;      ///< Template patch size (pixels).
    size_t _searchWindowSize = 21;       ///< Search window size (pixels).
};


}  // namespace imageProcessing
}  // namespace aliceVision
