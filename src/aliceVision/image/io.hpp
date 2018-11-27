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

#include <string>

namespace oiio = OIIO;

namespace aliceVision {
namespace image {

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
 * @brief extract metadata from an image for a given path
 * @param[in] path The given path to the image
 * @param[out] width The image header width
 * @param[out] height The image header height
 * @param[out] metadata All metadata find in the image
 */
void readImageMetadata(const std::string& path, int& width, int& height, std::map<std::string, std::string>& metadata);

/**
 * @brief read an image with a given path and buffer
 * @param[in] path The given path to the image
 * @param[out] image The output image buffer
 */
void readImage(const std::string& path, Image<float>& image);
void readImage(const std::string& path, Image<unsigned char>& image);
void readImage(const std::string& path, Image<RGBAColor>& image);
void readImage(const std::string& path, Image<RGBfColor>& image);
void readImage(const std::string& path, Image<RGBColor>& image);

/**
 * @brief write an image with a given path and buffer
 * @param[in] path The given path to the image
 * @param[in] image The output image buffer
 */
void writeImage(const std::string& path, const Image<unsigned char>& image, const oiio::ParamValueList& metadata = oiio::ParamValueList());
void writeImage(const std::string& path, const Image<RGBAColor>& image, const oiio::ParamValueList& metadata = oiio::ParamValueList());
void writeImage(const std::string& path, const Image<RGBfColor>& image, const oiio::ParamValueList& metadata = oiio::ParamValueList());
void writeImage(const std::string& path, const Image<RGBColor>& image, const oiio::ParamValueList& metadata = oiio::ParamValueList());

}  // namespace image
}  // namespace aliceVision
