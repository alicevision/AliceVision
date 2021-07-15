// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/image/Image.hpp>
#include <aliceVision/image/pixelTypes.hpp>
#include <aliceVision/types.hpp>

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
 * @brief aggregate for multiple image reading options
 */
struct ImageReadOptions
{  
  ImageReadOptions(EImageColorSpace colorSpace = EImageColorSpace::AUTO, bool useWhiteBalance = true, const oiio::ROI & roi = oiio::ROI()) :
  outputColorSpace(colorSpace), applyWhiteBalance(useWhiteBalance), subROI(roi)
  {
  }

  EImageColorSpace outputColorSpace;
  bool applyWhiteBalance;

  //ROI for this image.
  //If the image contains an roi, this is the roi INSIDE the roi.
  oiio::ROI subROI;
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
 * @brief Return a list of extensions supported by openImageIO ie exists in extension_list from imageio.h
 * @return A vector containing all supported extensions
 */
std::vector<std::string> getSupportedExtensions();

/**
 * @brief Check if input image extension is supported by openImageIO ie exists in extension_list from imageio.h
 * @param[in] ext - image extension
 * @return true if valid extension
 */
bool isSupported(const std::string& ext);


/**
* @brief Data type use to write the image
*/
enum class EStorageDataType
{
    Float, //< Use full floating point precision to store
    Half, //< Use half (values our of range could become inf or nan)
    HalfFinite, //< Use half, but ensures out-of-range pixels are clamps to keep finite pixel values
    Auto //< Use half if all pixels can be stored in half without clamp, else use full float
};

std::string EStorageDataType_informations();
EStorageDataType EStorageDataType_stringToEnum(const std::string& dataType);
std::string EStorageDataType_enumToString(const EStorageDataType dataType);
std::ostream& operator<<(std::ostream& os, EStorageDataType dataType);
std::istream& operator>>(std::istream& in, EStorageDataType& dataType);


/**
 * @brief convert a metadata string map into an oiio::ParamValueList
 * @param[in] metadataMap string map
 * @return oiio::ParamValueList
 */
oiio::ParamValueList getMetadataFromMap(const std::map<std::string, std::string>& metadataMap);

/**
 * @brief extract metadata from an image for a given path
 * @param[in] path The given path to the image
 * @param[out] width The image header width
 * @param[out] height The image header height
 * @return metadata All metadata find in the image
 */
oiio::ParamValueList readImageMetadata(const std::string& path, int& width, int& height);

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
 * @brief return the size of the image for a given path
 * @param path The given path to the image
 * @param[out] width The image header width
 * @param[out] height The image header height
 */
void readImageSize(const std::string& path, int& width, int& height);

/**
 * @brief get OIIO buffer from an AliceVision image
 * @param[in] image Image class
 * @param[out] buffer OIIO buffer
 */
void getBufferFromImage(Image<float>& image, oiio::ImageBuf& buffer);
void getBufferFromImage(Image<unsigned char>& image, oiio::ImageBuf& buffer);
void getBufferFromImage(Image<RGBAfColor>& image, oiio::ImageBuf& buffer);
void getBufferFromImage(Image<RGBAColor>& image, oiio::ImageBuf& buffer);
void getBufferFromImage(Image<RGBfColor>& image, oiio::ImageBuf& buffer);
void getBufferFromImage(Image<RGBColor>& image, oiio::ImageBuf& buffer);

/**
 * @brief read an image with a given path and buffer
 * @param[in] path The given path to the image
 * @param[out] image The output image buffer
 * @param[in] image color space
 */
void readImage(const std::string& path, Image<float>& image, const ImageReadOptions & imageReadOptions);
void readImage(const std::string& path, Image<unsigned char>& image, const ImageReadOptions & imageReadOptions);
void readImage(const std::string& path, Image<IndexT>& image, const ImageReadOptions & imageReadOptions);
void readImage(const std::string& path, Image<RGBAfColor>& image, const ImageReadOptions & imageReadOptions);
void readImage(const std::string& path, Image<RGBAColor>& image, const ImageReadOptions & imageReadOptions);
void readImage(const std::string& path, Image<RGBfColor>& image, const ImageReadOptions & imageReadOptions);
void readImage(const std::string& path, Image<RGBColor>& image, const ImageReadOptions & imageReadOptions);

/**
 * @brief read an image with a given path and buffer without any processing such as color conversion
 * @param[in] path The given path to the image
 * @param[out] image The output image buffer
 */
void readImageDirect(const std::string& path, Image<IndexT>& image);
void readImageDirect(const std::string& path, Image<unsigned char>& image);

/**
 * @brief write an image with a given path and buffer
 * @param[in] path The given path to the image
 * @param[in] image The output image buffer
 */
void writeImage(const std::string& path, const Image<float>& image, EImageColorSpace imageColorSpace,const oiio::ParamValueList& metadata = oiio::ParamValueList(),const oiio::ROI& roi = oiio::ROI());
void writeImage(const std::string& path, const Image<unsigned char>& image, EImageColorSpace imageColorSpace, const oiio::ParamValueList& metadata = oiio::ParamValueList());
void writeImage(const std::string& path, const Image<int>& image, EImageColorSpace imageColorSpace, const oiio::ParamValueList& metadata = oiio::ParamValueList());
void writeImage(const std::string& path, const Image<IndexT>& image, EImageColorSpace imageColorSpace, const oiio::ParamValueList& metadata = oiio::ParamValueList());
void writeImage(const std::string& path, const Image<RGBAfColor>& image, EImageColorSpace imageColorSpace,const oiio::ParamValueList& metadata = oiio::ParamValueList(),const oiio::ROI& roi = oiio::ROI());
void writeImage(const std::string& path, const Image<RGBAColor>& image, EImageColorSpace imageColorSpace, const oiio::ParamValueList& metadata = oiio::ParamValueList());
void writeImage(const std::string& path, const Image<RGBfColor>& image, EImageColorSpace imageColorSpace,const oiio::ParamValueList& metadata = oiio::ParamValueList(),const oiio::ROI& roi = oiio::ROI());
void writeImage(const std::string& path, const Image<RGBColor>& image, EImageColorSpace imageColorSpace, const oiio::ParamValueList& metadata = oiio::ParamValueList());


template <typename T>
struct ColorTypeInfo
{
    // no size parameter, so no default value.
    // An error will be raise at compile time if this type traits is not defined.
};

template <>
struct ColorTypeInfo<unsigned char>
{
    static const int size = 1;
    static const oiio::TypeDesc::BASETYPE typeDesc = oiio::TypeDesc::UINT8;
};
template <>
struct ColorTypeInfo<float>
{
    static const int size = 1;
    static const oiio::TypeDesc::BASETYPE typeDesc = oiio::TypeDesc::FLOAT;
};
template <>
struct ColorTypeInfo<RGBColor>
{
    static const int size = 3;
    static const oiio::TypeDesc::BASETYPE typeDesc = oiio::TypeDesc::UINT8;
};
template <>
struct ColorTypeInfo<RGBfColor>
{
    static const int size = 3;
    static const oiio::TypeDesc::BASETYPE typeDesc = oiio::TypeDesc::FLOAT;
};
template <>
struct ColorTypeInfo<RGBAColor>
{
    static const int size = 4;
    static const oiio::TypeDesc::BASETYPE typeDesc = oiio::TypeDesc::UINT8;
};
template <>
struct ColorTypeInfo<RGBAfColor>
{
    static const int size = 4;
    static const oiio::TypeDesc::BASETYPE typeDesc = oiio::TypeDesc::FLOAT;
};

}  // namespace image
}  // namespace aliceVision
