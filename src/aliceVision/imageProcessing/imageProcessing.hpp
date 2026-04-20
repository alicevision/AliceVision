// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once


#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/image/Image.hpp>

namespace aliceVision {
namespace imageProcessing {

/**
 * @brief Abstract base class for all image processing operations.
 *
 * Provides a common interface for in-place image processing steps applied during the
 * image preparation pipeline. Subclasses implement the actual processing logic by
 * overriding the pure virtual `processInternal` method.
 *
 * Each processing step receives the full SfMData context, the current View, the
 * associated camera intrinsics, and the RGBA float image to transform. A `dryRun`
 * flag allows subclasses to update metadata or image dimensions without performing
 * the actual pixel-level computation.
 */
class ImageProcess
{
public:

    virtual ~ImageProcess() = default;

    /**
     * @brief Execute the processing operation in place on the given image.
     * @param[in] sfmData The global SfMData providing scene-wide information (e.g. median exposure).
     * @param[in,out] view The View associated with the image, may be updated with new metadata or dimensions.
     * @param[in,out] camera The camera intrinsics, may be updated (e.g. after undistortion or resize).
     * @param[in,out] image The RGBA float image to process in place.
     * @param[in] dryRun If true, update metadata/dimensions only without modifying pixel data.
     * @return true on success.
     */
    bool processInPlace(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera, image::Image<image::RGBAfColor> & image, bool dryRun);
protected:
    /**
     * @brief Internal processing method to be implemented by each concrete subclass.
     * @param[in] sfmData The global SfMData.
     * @param[in,out] view The View associated with the image.
     * @param[in,out] camera The camera intrinsics.
     * @param[in,out] image The RGBA float image to process.
     * @param[in] dryRun If true, skip pixel-level computation.
     * @return true on success.
     */
    virtual bool processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera, image::Image<image::RGBAfColor> & image, bool dryRun) = 0;
};

/**
 * @brief Normalizes image exposure based on the median camera exposure across the dataset.
 *
 * Computes a compensation factor from the ratio of the median camera exposure
 * to the current view's exposure, and multiplies each pixel's RGB channels
 * by that factor. This helps ensure consistent brightness across images
 * captured with different exposure settings.
 */
class ExposureProcess : public ImageProcess
{
private:
    bool processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera, image::Image<image::RGBAfColor> & image, bool dryRun);
};

/**
 * @brief Fixes non-finite pixel values (NaN, Inf) in the image.
 *
 * Uses OpenImageIO's `fixNonFinite` with a 3x3 box filter to replace
 * non-finite pixels with the average of their finite neighbours.
 * This is useful for sanitizing images before further processing.
 */
class FixHolesProcess : public ImageProcess
{
private:
    bool processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera, image::Image<image::RGBAfColor> & image, bool dryRun);
};


/**
 * @brief Corrects lens vignetting based on per-view vignetting parameters.
 *
 * Reads a polynomial vignetting model (up to 4th order in squared radius)
 * from the view's metadata and applies a per-pixel gain to compensate for
 * brightness fall-off towards the image edges. If no vignetting parameters
 * are available, the image is left unchanged.
 */
class RemoveVignettingProcess : public ImageProcess
{
public:
    RemoveVignettingProcess()
    {

    }

private:
    bool processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera, image::Image<image::RGBAfColor> & image, bool dryRun);
};

/**
 * @brief Corrects lateral chromatic aberration using per-channel rectilinear distortion models.
 *
 * Reads chromatic aberration parameters (green, blue-green, red-green rectilinear models)
 * from the view's metadata and re-samples R, G, and B channels independently to align
 * them. Optionally undistorts the green channel geometry as well when `undistortGeometry`
 * is enabled. If no valid chromatic aberration parameters are found, the image is unchanged.
 */
class UndistortChromaticAberrationsProcess : public ImageProcess
{
public:
    /**
     * @brief Construct a chromatic aberration correction process.
     * @param[in] undistortGeometry If true, also undistort the green channel's geometric distortion.
     */
    UndistortChromaticAberrationsProcess(bool undistortGeometry = false)
        : _undistortGeometry(undistortGeometry)
    {}

private:
    bool processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera, image::Image<image::RGBAfColor> & image, bool dryRun);

private:
    image::Image<image::RGBAfColor> _imageBuf;  ///< Temporary buffer for the re-sampled image.
    bool _undistortGeometry = false;             ///< Whether to also undistort the green channel geometry.
};

/**
 * @brief Removes lens geometric distortion using the camera intrinsics model.
 *
 * Undistorts the image using the distortion parameters stored in the camera intrinsics.
 * If the camera has no distortion model or no camera is provided, the image is left unchanged.
 * The undistorted image replaces the original in place.
 */
class UndistortProcess : public ImageProcess
{
private:
    bool processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera, image::Image<image::RGBAfColor> & image, bool dryRun);

private:
    image::Image<image::RGBAfColor> _imageBuf;  ///< Temporary buffer for the undistorted image.
};

/**
 * @brief Resizes images based on maximum dimensions, a scale factor, and pixel aspect ratio.
 *
 * Computes the effective scale factor from the combination of `maxWidth`, `maxHeight`,
 * and `scaleFactor` constraints (the most restrictive one wins). Pixel aspect ratio (PAR)
 * handling is supported: when `parEnabled` is true, the PAR from the camera intrinsics is
 * taken into account, either by adjusting pixel dimensions (`parDecimation`) or by
 * compensating the width scale.
 *
 * After resizing, the View's width/height and the camera intrinsics are updated accordingly.
 * In `dryRun` mode the image is resized (allocated) but pixels are not resampled.
 */
class ResizeProcess : public ImageProcess
{
public:
    /**
     * @brief Construct a resize process.
     * @param[in] maxWidth  Maximum allowed output width (0 = no constraint).
     * @param[in] maxHeight Maximum allowed output height (0 = no constraint).
     * @param[in] scaleFactor Global scale factor applied to both dimensions.
     * @param[in] parEnabled Whether pixel aspect ratio compensation is active.
     * @param[in] parDecimation If true, apply PAR by decimating height; otherwise adjust width.
     */
    ResizeProcess(size_t maxWidth, size_t maxHeight, double scaleFactor, bool parEnabled, bool parDecimation)
        : _maxWidth(maxWidth),
          _maxHeight(maxHeight),
          _scaleFactor(scaleFactor),
          _parEnabled(parEnabled),
          _parDecimation(parDecimation)
    {}

private:
    bool processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera, image::Image<image::RGBAfColor> & image, bool dryRun);

private:
    image::Image<image::RGBAfColor> _imageBuf;  ///< Temporary buffer for the resized image.
    size_t _maxWidth = 1;                       ///< Maximum output width constraint.
    size_t _maxHeight = 1;                      ///< Maximum output height constraint.
    double _scaleFactor = 1.0;                  ///< Global scale factor.
    bool _parEnabled = false;                   ///< Whether pixel aspect ratio is enabled.
    bool _parDecimation = false;                ///< Whether PAR is applied via height decimation.
};

/**
 * @brief Reorients the image according to its EXIF Orientation metadata tag.
 *
 * Reads the "Orientation" EXIF tag from the view's metadata and applies the
 * corresponding rotation / flip using OpenImageIO's `reorient`. For orientations
 * 5–8 (which involve a 90° or 270° rotation), the view dimensions, camera intrinsics
 * dimensions, and sensor width/height are swapped accordingly.
 * The Orientation metadata is then reset to "1" (normal).
 */
class ReorientProcess : public ImageProcess
{
private:
    bool processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera, image::Image<image::RGBAfColor> & image, bool dryRun);

private:
    image::Image<image::RGBAfColor> _imageBuf;  ///< Temporary buffer for the reoriented image.
};

/**
 * @brief Adjusts image contrast using OpenImageIO's contrast remapping.
 *
 * Remaps pixel values through a contrast curve controlled by the `contrast`
 * parameter, using `oiio::ImageBufAlgo::contrast_remap` with a [0,1] input
 * and output range. A contrast value of 1.0 produces a linear (identity) mapping.
 */
class ContrastProcess : public ImageProcess
{
public:
    /**
     * @brief Construct a contrast adjustment process.
     * @param[in] contrast The contrast amount (1.0 = no change, >1.0 = increased contrast).
     */
    ContrastProcess(float contrast = 1.0)
        : _contrast(contrast)
    {}

private:
    bool processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera, image::Image<image::RGBAfColor> & image, bool dryRun);

private:
    image::Image<image::RGBAfColor> _imageBuf;  ///< Temporary buffer for the contrast-adjusted image.
    float _contrast = 1.0;                      ///< Contrast parameter.
};

/**
 * @brief Applies a median filter to the image for noise reduction.
 *
 * Uses OpenImageIO's `median_filter` with the specified kernel size.
 * The median filter replaces each pixel with the median value in its
 * neighbourhood, effectively removing salt-and-pepper noise while
 * preserving edges.
 */
class MedianFilterProcess : public ImageProcess
{
public:
    /**
     * @brief Construct a median filter process.
     * @param[in] medianFilter The kernel size for the median filter (0.0 = no filtering).
     */
    MedianFilterProcess(float medianFilter = 0.0)
        : _medianFilter(medianFilter)
    {}

private:
    bool processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera, image::Image<image::RGBAfColor> & image, bool dryRun);

private:
    image::Image<image::RGBAfColor> _imageBuf;  ///< Temporary buffer for the filtered image.
    float _medianFilter = 0.0;                  ///< Median filter kernel size.
};

/**
 * @brief Sharpens the image using an unsharp mask with a Gaussian kernel.
 *
 * Applies OpenImageIO's `unsharp_mask` algorithm with a Gaussian kernel of
 * the given `width`, sharpening `contrast` (strength), and a luminance
 * `threshold` below which no sharpening is applied. This enhances edges and
 * fine details in the image.
 */
class SharpenProcess : public ImageProcess
{
public:
    /**
     * @brief Construct a sharpening process.
     * @param[in] width     The width (radius) of the Gaussian blur kernel.
     * @param[in] contrast  The sharpening strength (amount of edge enhancement).
     * @param[in] threshold The minimum luminance difference for sharpening to be applied.
     */
    SharpenProcess(size_t width = 0, float contrast = 0.0, float threshold = 0.0)
        : _width(width),
          _contrast(contrast),
          _threshold(threshold)
    {}

private:
    bool processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera, image::Image<image::RGBAfColor> & image, bool dryRun);

private:
    image::Image<image::RGBAfColor> _imageBuf;  ///< Temporary buffer for the sharpened image.
    size_t _width = 0;                        ///< Gaussian kernel radius.
    float _contrast = 0.0;                      ///< Sharpening strength.
    float _threshold = 0.0;                     ///< Minimum luminance threshold for sharpening.
};

/**
 * @brief Fills transparent or missing regions in the image using a push-pull algorithm.
 *
 * Premultiplies the image by its alpha channel, then uses OpenImageIO's
 * `fillholes_pushpull` to propagate colour information from opaque regions
 * into transparent or zero-alpha areas. This is useful for filling holes
 * left by undistortion or other geometric transformations.
 */
class FillHolesProcess : public ImageProcess
{
private:
    bool processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera, image::Image<image::RGBAfColor> & image, bool dryRun);

private:
    image::Image<image::RGBAfColor> _imageBuf;  ///< Temporary buffer for the filled image.
};

/**
 * @brief Adds synthetic noise to the image.
 *
 * Uses OpenImageIO's `noise` function to inject noise according to the chosen
 * distribution method (e.g. "uniform", "gaussian", "salt"). Parameters `A` and
 * `B` control the distribution (e.g. mean/stddev for Gaussian, min/max for uniform).
 * When `mono` is true, the same noise value is applied to all RGB channels per pixel.
 */
class NoiseProcess : public ImageProcess
{
public:
    /**
     * @brief Construct a noise injection process.
     * @param[in] method The noise distribution type ("uniform", "gaussian", "salt", etc.).
     * @param[in] A First distribution parameter (e.g. mean for Gaussian, min for uniform).
     * @param[in] B Second distribution parameter (e.g. stddev for Gaussian, max for uniform).
     * @param[in] mono If true, apply identical noise to all RGB channels per pixel.
     */
    NoiseProcess(const std::string & method, float A = 0.0f, float B = 1.0f, bool mono = true)
        : _method(method),
          _A(A),
          _B(B),
          _mono(mono)
    {}

private:
    bool processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera, image::Image<image::RGBAfColor> & image, bool dryRun);

private:
    std::string _method = "uniform";  ///< Noise distribution method.
    float _A = 0.0f;                  ///< First distribution parameter.
    float _B = 1.0f;                  ///< Second distribution parameter.
    bool _mono = true;                ///< Whether noise is monochromatic.
};

/**
 * @brief Applies DCP (DNG Camera Profile) color correction and color temperature processing.
 *
 * This process handles two complementary tasks:
 * - **Color temperature estimation/override**: when `enableColorTempProcessing` is true, it
 *   either stores a user-supplied correlated color temperature (CCT) or derives one from
 *   the camera's white-balance multipliers using the DCP profile.
 * - **DCP linear color matrix application**: when `applyDcpMetadata` is true, it reads
 *   DCP color/forward matrices from the view's metadata and applies linear color correction
 *   to the image pixels, optionally using only the color matrix (`useDCPColorMatrixOnly`).
 *
 * The resulting correlated color temperature is stored in the view's metadata as
 * "AliceVision:ColorTemperature".
 */
class ColorTemperatureProcess : public ImageProcess
{
public:
    /**
     * @brief Construct a color temperature / DCP processing step.
     * @param[in] applyDcpMetadata          If true, apply the DCP linear colour correction to image pixels.
     * @param[in] useDCPColorMatrixOnly     If true, use only the colour matrix (skip forward matrices).
     * @param[in] enableColorTempProcessing If true, estimate or set the correlated colour temperature.
     * @param[in] correlatedColorTemperature User-supplied CCT value; negative means auto-detect.
     */
    ColorTemperatureProcess(bool applyDcpMetadata, bool useDCPColorMatrixOnly, bool enableColorTempProcessing, float correlatedColorTemperature)
    : _correlatedColorTemperature(correlatedColorTemperature),
    _applyDcpMetadata(applyDcpMetadata),
    _useDCPColorMatrixOnly(useDCPColorMatrixOnly),
    _enableColorTempProcessing(enableColorTempProcessing)
    {}

private:
    bool processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera, image::Image<image::RGBAfColor> & image, bool dryRun);

private:
    double _correlatedColorTemperature = -1.0;  ///< Target correlated colour temperature (negative = auto).
    bool _applyDcpMetadata = false;             ///< Whether to apply DCP linear colour correction.
    bool _useDCPColorMatrixOnly = false;        ///< Whether to skip forward matrices.
    bool _enableColorTempProcessing = false;    ///< Whether colour temperature estimation/override is enabled.
};


}  // namespace imageProcessing
}  // namespace aliceVision
