// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <string>

#include <OpenImageIO/paramlist.h>

namespace oiio = OIIO;

namespace aliceVision {

class rgb;
class Color;
class Image;

namespace imageIO {

/**
 * @brief Available image file types for pipeline output
 */
enum class EImageFileType
{
  JPEG,
  PNG,
  TIFF,
  EXR
};

/**
 * @brief Available image color space for pipeline input / output
 */
enum class EImageColorSpace
{
  AUTO,
  LINEAR,
  SRGB,
  NO_CONVERSION
};


struct OutputFileColorSpace
{
    EImageColorSpace from{EImageColorSpace::LINEAR};
    EImageColorSpace to{EImageColorSpace::AUTO};

    OutputFileColorSpace(EImageColorSpace from_, EImageColorSpace to_)
        : from(from_)
        , to(to_)
    {
    }
    /// @brief Assumes that @p from is LINEAR
    OutputFileColorSpace(EImageColorSpace to_)
    {
        if(to_ == EImageColorSpace::NO_CONVERSION)
            to = from;
        else
            to = to_;
    }
    OutputFileColorSpace() = default;
};

/**
 * @brief Available image qualities for pipeline output
 */
enum class EImageQuality
{
  OPTIMIZED,
  LOSSLESS
};

std::string EImageColorSpace_enumToString(const EImageColorSpace colorSpace);

/**
 * @brief get informations about each image quality
 * @return String
 */
std::string EImageQuality_informations();

/**
 * @brief returns the EImageQuality enum from a string.
 * @param[in] imageQuality the input string.
 * @return the associated EImageQuality enum.
 */
EImageQuality EImageQuality_stringToEnum(const std::string& imageQuality);

/**
 * @brief converts an EImageQuality enum to a string.
 * @param[in] imageQuality the EImageQuality enum to convert.
 * @return the string associated to the EImageQuality enum.
 */
std::string EImageQuality_enumToString(const EImageQuality imageQuality);

/**
 * @brief write an EImageQuality enum into a stream by converting it to a string.
 * @param[in] os the stream where to write the imageType.
 * @param[in] imageQuality the EImageQuality enum to write.
 * @return the modified stream.
 */
std::ostream& operator<<(std::ostream& os, EImageQuality imageQuality);

/**
 * @brief read a EImageQuality enum from a stream.
 * @param[in] in the stream from which the enum is read.
 * @param[out] imageQuality the EImageQuality enum read from the stream.
 * @return the modified stream without the read enum.
 */
std::istream& operator>>(std::istream& in, EImageQuality& imageQuality);

/**
 * @brief convert a metadata string map into an oiio::ParamValueList
 * @param[in] metadataMap string map
 * @return oiio::ParamValueList
 */
oiio::ParamValueList getMetadataFromMap(const std::map<std::string, std::string>& metadataMap);

/**
 * @brief get informations about each image file type
 * @return String
 */
std::string EImageFileType_informations();

/**
 * @brief returns the EImageFileType enum from a string.
 * @param[in] imageFileType the input string.
 * @return the associated EImageFileType enum.
 */
EImageFileType EImageFileType_stringToEnum(const std::string& imageFileType);

/**
 * @brief converts an EImageFileType enum to a string.
 * @param[in] imageFileType the EImageFileType enum to convert.
 * @return the string associated to the EImageFileType enum.
 */
std::string EImageFileType_enumToString(const EImageFileType imageFileType);

/**
 * @brief write an EImageFileType enum into a stream by converting it to a string.
 * @param[in] os the stream where to write the imageType.
 * @param[in] imageFileType the EImageFileType enum to write.
 * @return the modified stream.
 */
std::ostream& operator<<(std::ostream& os, EImageFileType imageFileType);

/**
 * @brief read a EImageFileType enum from a stream.
 * @param[in] in the stream from which the enum is read.
 * @param[out] imageFileType the EImageFileType enum read from the stream.
 * @return the modified stream without the read enum.
 */
std::istream& operator>>(std::istream& in, EImageFileType& imageFileType);

/**
 * @brief read image dimension from a given path
 * @param[in] path The given path to the image
 * @param[out] width The image width
 * @param[out] height The image height
 * @param[out] nchannels The image channel number
 */
void readImageSpec(const std::string& path, int& width, int& height, int& nchannels);

/**
 * @brief read image metadata from a given path
 * @param[in] path The given path to the image
 * @param[out] metadata The image metadata
 */
void readImageMetadata(const std::string& path, oiio::ParamValueList& metadata);

/**
 * @brief Test if the extension is supported for undistorted images.
 * @param[in] ext The extension with the dot (eg ".png")
 * @return \p true if the extension is supported.
 */
bool isSupportedUndistortFormat(const std::string &ext);

/**
 * @brief read an image with a given path and buffer
 * @param[in] path The given path to the image
 * @param[out] width The output image width
 * @param[out] height The output image height
 * @param[out] buffer The output image buffer
 * @param[in] image color space
 */
void readImage(const std::string& path, int& width, int& height, std::vector<unsigned char>& buffer, EImageColorSpace imageColorSpace);
void readImage(const std::string& path, int& width, int& height, std::vector<unsigned short>& buffer, EImageColorSpace imageColorSpace);
void readImage(const std::string& path, int& width, int& height, std::vector<rgb>& buffer, EImageColorSpace imageColorSpace);
void readImage(const std::string& path, int& width, int& height, std::vector<float>& buffer, EImageColorSpace imageColorSpace);
void readImage(const std::string& path, int& width, int& height, std::vector<Color>& buffer, EImageColorSpace imageColorSpace);
void readImage(const std::string& path, Image& image, EImageColorSpace imageColorSpace);

/**
 * @brief write an image with a given path and buffer
 * @param[in] path The given path to the image
 * @param[in] width The input image width
 * @param[in] height The input image height
 * @param[in] buffer The input image buffer
 */
void writeImage(const std::string& path, int width, int height, const std::vector<unsigned char>& buffer, EImageQuality imageQuality, OutputFileColorSpace colorspace, const oiio::ParamValueList& metadata = oiio::ParamValueList());
void writeImage(const std::string& path, int width, int height, const std::vector<unsigned short>& buffer, EImageQuality imageQuality, OutputFileColorSpace colorspace, const oiio::ParamValueList& metadata = oiio::ParamValueList());
void writeImage(const std::string& path, int width, int height, const std::vector<rgb>& buffer, EImageQuality imageQuality, OutputFileColorSpace colorspace, const oiio::ParamValueList& metadata = oiio::ParamValueList());
void writeImage(const std::string& path, int width, int height, const std::vector<float>& buffer, EImageQuality imageQuality, OutputFileColorSpace colorspace, const oiio::ParamValueList& metadata = oiio::ParamValueList());
void writeImage(const std::string& path, int width, int height, const std::vector<Color>& buffer, EImageQuality imageQuality, OutputFileColorSpace colorspace, const oiio::ParamValueList& metadata = oiio::ParamValueList());
void writeImage(const std::string& path, Image& image, EImageQuality imageQuality, OutputFileColorSpace colorspace, const oiio::ParamValueList& metadata = oiio::ParamValueList());

/**
 * @brief transpose a given image buffer
 * @param[in] width The image buffer width
 * @param[in] height The image buffer height
 * @param[in,out] buffer The image buffer
 */
void transposeImage(int width, int height, std::vector<unsigned char>& buffer);
void transposeImage(int width, int height, std::vector<rgb>& buffer);
void transposeImage(int width, int height, std::vector<float>& buffer);
void transposeImage(int width, int height, std::vector<Color>& buffer);
void transposeImage(Image& image);

/**
 * @brief resize a given image buffer
 * @param[in] inWidth The input image buffer width
 * @param[in] inHeight The input image buffer height
 * @param[in] downscale The resize downscale
 * @param[in] inBuffer The input image buffer
 * @param[out] outBuffer The output image buffer
 * @param[in] filter The name of a high-quality filter to use when resampling
 *            Default is bilinear resampling
 *            See openImageIO documentation "ImageBufAlgo filtername"
 * @param[in] filterSize The resize filter size
 */
void resizeImage(int inWidth, int inHeight, int downscale, const std::vector<unsigned char>& inBuffer, std::vector<unsigned char>& outBuffer, const std::string& filter = "", float filterSize = 0);
void resizeImage(int inWidth, int inHeight, int downscale, const std::vector<rgb>& inBuffer, std::vector<rgb>& outBuffer, const std::string& filter = "", float filterSize = 0);
void resizeImage(int inWidth, int inHeight, int downscale, const std::vector<float>& inBuffer, std::vector<float>& outBuffer, const std::string& filter = "", float filterSize = 0);
void resizeImage(int inWidth, int inHeight, int downscale, const std::vector<Color>& inBuffer, std::vector<Color>& outBuffer, const std::string& filter = "", float filterSize = 0);
void resizeImage(int downscale, const Image& inImage, Image& outImage,  const std::string& filter = "", float filterSize = 0);

/**
 * @brief convolve a given image buffer
 * @param[in] inWidth The input image buffer width
 * @param[in] inHeight The input image buffer heightt
 * @param[in] inBuffer The input image buffer
 * @param[out] outBuffer outBuffer The output image buffer
 * @param[in] kernel The kernel name, can be "gaussian", "sharp-gaussian", "box", ...
 *            Default is gaussian kernel
 *            See openImageIO documentation "ImageBufAlgo.make_kernel"
 * @param[in] kernelWidth The kernel width
 * @param[in] kernelHeight The kernal height
 */
void convolveImage(int inWidth, int inHeight, const std::vector<unsigned char>& inBuffer, std::vector<unsigned char>& outBuffer, const std::string& kernel = "gaussian", float kernelWidth = 5.0f, float kernelHeight = 5.0f);
void convolveImage(int inWidth, int inHeight, const std::vector<rgb>& inBuffer, std::vector<rgb>& outBuffer, const std::string& kernel = "gaussian", float kernelWidth = 5.0f, float kernelHeight = 5.0f);
void convolveImage(int inWidth, int inHeight, const std::vector<float>& inBuffer, std::vector<float>& outBuffer, const std::string& kernel = "gaussian", float kernelWidth = 5.0f, float kernelHeight = 5.0f);
void convolveImage(int inWidth, int inHeight, const std::vector<Color>& inBuffer, std::vector<Color>& outBuffer, const std::string& kernel = "gaussian", float kernelWidth = 5.0f, float kernelHeight = 5.0f);
void convolveImage(const Image& inImage, Image& outImage, const std::string& kernel = "gaussian", float kernelWidth = 5.0f, float kernelHeight = 5.0f);

/**
 * @brief fill holes in a given image buffer with plausible values
 * @param[in] inWidth The input image buffer width
 * @param[in] inHeight The input image buffer height
 * @param[in,out] colorBuffer The image buffer to fill
 * @param[in] alphaBuffer The input alpha buffer containing 0.0/1.0 for empty/valid pixels
 */
void fillHoles(int inWidth, int inHeight, std::vector<Color>& colorBuffer, const std::vector<float>& alphaBuffer);
void fillHoles(Image& image, const std::vector<float>& alphaBuffer);

} // namespace imageIO
} // namespace aliceVision
