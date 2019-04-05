// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/image/Image.hpp>
#include <aliceVision/image/pixelTypes.hpp>

#include <OpenImageIO/paramlist.h>
#include <OpenImageIO/imagebuf.h>

#include <string>

namespace oiio = OIIO;

namespace aliceVision {
namespace image {

/**
 * @brief Available image color space for pipeline input
 */
enum class EImageColorSpace
{
  AUTO,
  LINEAR,
  SRGB,
  NO_CONVERSION
};

/**
 * @brief Available image file type for pipeline output
 */
enum class EImageFileType
{
  JPEG,
  PNG,
  TIFF,
  EXR
};

/**
 * @brief get informations about each image file type
 * @return String
 */
std::string EImageFileType_informations();

/**
 * @brief It returns the EImageFileType enum from a string.
 * @param[in] imageFileType the input string.
 * @return the associated EImageFileType enum.
 */
EImageFileType EImageFileType_stringToEnum(const std::string& imageFileType);

/**
 * @brief It converts a EImageFileType enum to a string.
 * @param[in] imageFileType the EImageFileType enum to convert.
 * @return the string associated to the EImageFileType enum.
 */
std::string EImageFileType_enumToString(const EImageFileType imageFileType);

/**
 * @brief It write a EImageFileType enum into a stream by converting it to a string.
 * @param[in] os the stream where to write the imageType.
 * @param[in] imageFileType the EImageFileType enum to write.
 * @return the modified stream.
 */
std::ostream& operator<<(std::ostream& os, EImageFileType imageFileType);

/**
 * @brief It read a EImageFileType enum from a stream.
 * @param[in] in the stream from which the enum is read.
 * @param[out] imageFileType the EImageFileType enum read from the stream.
 * @return the modified stream without the read enum.
 */
std::istream& operator>>(std::istream& in, EImageFileType& imageFileType);

/**
 * @brief convert a metadata string map into an oiio::ParamValueList
 * @param[in] metadataMap string map
 * @return oiio::ParamValueList
 */
oiio::ParamValueList getMetadataFromMap(const std::map<std::string, std::string>& metadataMap);

/**
 * @brief extract metadata from an image for a given path
 * @param[in] path The given path to the image
 * @return metadata All metadata find in the image
 */
oiio::ParamValueList readImageMetadata(const std::string& path);

/**
 * @brief extract metadata from an image for a given path
 * @param[in] path The given path to the image
 * @param[out] width The image header width
 * @param[out] height The image header height
 * @param[out] metadata All metadata find in the image
 */
void readImageMetadata(const std::string& path, int& width, int& height, std::map<std::string, std::string>& metadata);

/**
 * @brief get OIIO buffer from an AliceVision image
 * @param[in] image Image class
 * @param[out] buffer OIIO buffer
 */
void getBufferFromImage(Image<float>& image, oiio::ImageBuf& buffer);
void getBufferFromImage(Image<unsigned char>& image, oiio::ImageBuf& buffer);
void getBufferFromImage(Image<RGBAColor>& image, oiio::ImageBuf& buffer);
void getBufferFromImage(Image<RGBfColor>& image, oiio::ImageBuf& buffer);
void getBufferFromImage(Image<RGBColor>& image, oiio::ImageBuf& buffer);

/**
 * @brief read an image with a given path and buffer
 * @param[in] path The given path to the image
 * @param[out] image The output image buffer
 * @param[in] image color space
 */
void readImage(const std::string& path, Image<float>& image, EImageColorSpace imageColorSpace);
void readImage(const std::string& path, Image<unsigned char>& image, EImageColorSpace imageColorSpace);
void readImage(const std::string& path, Image<RGBAColor>& image, EImageColorSpace imageColorSpace);
void readImage(const std::string& path, Image<RGBfColor>& image, EImageColorSpace imageColorSpace);
void readImage(const std::string& path, Image<RGBColor>& image, EImageColorSpace imageColorSpace);

/**
 * @brief write an image with a given path and buffer
 * @param[in] path The given path to the image
 * @param[in] image The output image buffer
 */
void writeImage(const std::string& path, const Image<unsigned char>& image, EImageColorSpace imageColorSpace, const oiio::ParamValueList& metadata = oiio::ParamValueList());
void writeImage(const std::string& path, const Image<RGBAColor>& image, EImageColorSpace imageColorSpace, const oiio::ParamValueList& metadata = oiio::ParamValueList());
void writeImage(const std::string& path, const Image<RGBfColor>& image, EImageColorSpace imageColorSpace, const oiio::ParamValueList& metadata = oiio::ParamValueList());
void writeImage(const std::string& path, const Image<RGBColor>& image, EImageColorSpace imageColorSpace, const oiio::ParamValueList& metadata = oiio::ParamValueList());

}  // namespace image
}  // namespace aliceVision
