// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/image/io.hpp>
#include <OpenImageIO/paramlist.h>
#include <string>

namespace oiio = OIIO;

namespace aliceVision {

class rgb;
class ColorRGBf;
class ColorRGBAf;
template<typename Color> class Image;
using ImageRGBf = Image<ColorRGBf>;
using ImageRGBAf = Image<ColorRGBAf>;

namespace imageIO {

struct OutputFileColorSpace
{
    image::EImageColorSpace from{image::EImageColorSpace::LINEAR};
    image::EImageColorSpace to{image::EImageColorSpace::AUTO};

    OutputFileColorSpace(image::EImageColorSpace from_, image::EImageColorSpace to_)
        : from(from_)
        , to(to_)
    {
    }
    /// @brief Assumes that @p from is LINEAR
    explicit OutputFileColorSpace(image::EImageColorSpace to_)
    {
        if(to_ == image::EImageColorSpace::NO_CONVERSION)
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
void readImage(const std::string& path, int& width, int& height, std::vector<unsigned char>& buffer,
               image::EImageColorSpace toColorSpace);
void readImage(const std::string& path, int& width, int& height, std::vector<unsigned short>& buffer,
               image::EImageColorSpace toColorSpace);
void readImage(const std::string& path, int& width, int& height, std::vector<rgb>& buffer,
               image::EImageColorSpace toColorSpace);
void readImage(const std::string& path, int& width, int& height, std::vector<float>& buffer,
               image::EImageColorSpace toColorSpace);
void readImage(const std::string& path, int& width, int& height, std::vector<ColorRGBf>& buffer,
               image::EImageColorSpace toColorSpace);
void readImage(const std::string& path, int& width, int& height, std::vector<ColorRGBAf>& buffer,
               image::EImageColorSpace toColorSpace);
void readImage(const std::string& path, ImageRGBf& image, image::EImageColorSpace toColorSpace);
void readImage(const std::string& path, ImageRGBAf& image, image::EImageColorSpace toColorSpace);

/**
 * @brief write an image with a given path and buffer
 * @param[in] path The given path to the image
 * @param[in] width The input image width
 * @param[in] height The input image height
 * @param[in] buffer The input image buffer
 */
void writeImage(const std::string& path, int width, int height, const std::vector<unsigned char>& buffer, EImageQuality imageQuality, const OutputFileColorSpace& colorspace, const oiio::ParamValueList& metadata = oiio::ParamValueList());
void writeImage(const std::string& path, int width, int height, const std::vector<unsigned short>& buffer, EImageQuality imageQuality, const OutputFileColorSpace& colorspace, const oiio::ParamValueList& metadata = oiio::ParamValueList());
void writeImage(const std::string& path, int width, int height, const std::vector<rgb>& buffer, EImageQuality imageQuality, const OutputFileColorSpace& colorspace, const oiio::ParamValueList& metadata = oiio::ParamValueList());
void writeImage(const std::string& path, int width, int height, const std::vector<float>& buffer, EImageQuality imageQuality, const OutputFileColorSpace& colorspace, const oiio::ParamValueList& metadata = oiio::ParamValueList());
void writeImage(const std::string& path, int width, int height, const std::vector<ColorRGBf>& buffer, EImageQuality imageQuality, const OutputFileColorSpace& colorspace, const oiio::ParamValueList& metadata = oiio::ParamValueList());
void writeImage(const std::string& path, ImageRGBf& image, EImageQuality imageQuality, const OutputFileColorSpace& colorspace, const oiio::ParamValueList& metadata = oiio::ParamValueList());

} // namespace imageIO
} // namespace aliceVision

