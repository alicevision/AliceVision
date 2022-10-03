// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "imageIO.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/image/Color.hpp>
#include <aliceVision/image/LegacyImage.hpp>
#include <aliceVision/mvsData/Rgb.hpp>
#include <aliceVision/image/imageAlgo.hpp>

#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>

#include <aliceVision/half.hpp>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/case_conv.hpp>

#include <iostream>
#include <algorithm>
#include <stdexcept>

#include <array>
#include <stdexcept>
#include <memory>
#include <string>


namespace fs = boost::filesystem;

namespace aliceVision {

namespace imageIO {

std::string EImageQuality_informations()
{
  return "Image quality :\n"
         "* optimized \n"
         "* lossless ";
}

EImageQuality EImageQuality_stringToEnum(const std::string& imageQuality)
{
  std::string type = imageQuality;
  std::transform(type.begin(), type.end(), type.begin(), ::tolower); //tolower

  if(type == "optimized") return EImageQuality::OPTIMIZED;
  if(type == "lossless")  return EImageQuality::LOSSLESS;

  throw std::out_of_range("Invalid image quality : " + imageQuality);
}

std::string EImageQuality_enumToString(const EImageQuality imageQuality)
{
  switch(imageQuality)
  {
    case EImageQuality::OPTIMIZED:  return "optimized";
    case EImageQuality::LOSSLESS:   return "lossless";
  }
  throw std::out_of_range("Invalid EImageQuality enum");
}

std::ostream& operator<<(std::ostream& os, EImageQuality imageQuality)
{
  return os << EImageQuality_enumToString(imageQuality);
}

std::istream& operator>>(std::istream& in, EImageQuality& imageQuality)
{
  std::string token;
  in >> token;
  imageQuality = EImageQuality_stringToEnum(token);
  return in;
}

bool isSupportedUndistortFormat(const std::string &ext)
{
  static const std::array<std::string, 6> supportedExtensions = {".jpg", ".jpeg", ".png",  ".tif", ".tiff", ".exr"};
  const auto start = supportedExtensions.begin();
  const auto end = supportedExtensions.end();
  return(std::find(start, end, boost::to_lower_copy(ext)) != end);
}

template<typename T>
void readImage(const std::string& path,
               oiio::TypeDesc typeDesc,
               int nchannels,
               int& width,
               int& height,
               std::vector<T>& buffer,
               image::EImageColorSpace toColorSpace)
{
    ALICEVISION_LOG_DEBUG("[IO] Read Image: " << path);

    // check requested channels number
    if (nchannels == 0)
        throw std::runtime_error("Requested channels is 0. Image file: '" + path + "'.");
    if (nchannels == 2)
        throw std::runtime_error("Load of 2 channels is not supported. Image file: '" + path + "'.");

    oiio::ImageSpec configSpec;

    // libRAW configuration
    configSpec.attribute("raw:auto_bright", 0);       // don't want exposure correction
    configSpec.attribute("raw:use_camera_wb", 1);     // want white balance correction
    configSpec.attribute("raw:use_camera_matrix", 3); // want to use embeded color profile
    configSpec.attribute("raw:ColorSpace", "Linear");   // want linear colorspace with sRGB primaries

    oiio::ImageBuf inBuf(path, 0, 0, NULL, &configSpec);

    inBuf.read(0, 0, true, oiio::TypeDesc::FLOAT); // force image convertion to float (for grayscale and color space convertion)

    if(!inBuf.initialized())
        throw std::runtime_error("Cannot find/open image file '" + path + "'.");

    const oiio::ImageSpec& inSpec = inBuf.spec();

    // check picture channels number
    if (inSpec.nchannels == 0)
        throw std::runtime_error("No channel in the input image file: '" + path + "'.");
    if (inSpec.nchannels == 2)
        throw std::runtime_error("Load of 2 channels is not supported. Image file: '" + path + "'.");

    // color conversion
    if (toColorSpace == image::EImageColorSpace::AUTO)
        throw std::runtime_error("You must specify a requested color space for image file '" + path + "'.");

    const std::string& fromColorSpaceName = inSpec.get_string_attribute("oiio:ColorSpace", "sRGB"); // default image color space is sRGB
    ALICEVISION_LOG_TRACE("Read image " << path << " (encoded in " << fromColorSpaceName << " colorspace).");
    const auto fromColorSpace = image::EImageColorSpace_OIIOstringToEnum(fromColorSpaceName);

    if (toColorSpace != image::EImageColorSpace::NO_CONVERSION)
        imageAlgo::colorconvert(inBuf, fromColorSpace, toColorSpace);

    // convert to grayscale if needed
    if(nchannels == 1 && inSpec.nchannels >= 3)
    {
        // convertion region of interest (for inSpec.nchannels > 3)
        oiio::ROI convertionROI = inBuf.roi();
        convertionROI.chbegin = 0;
        convertionROI.chend = 3;

        // compute luminance via a weighted sum of R,G,B
        // (assuming Rec709 primaries and a linear scale)
        const float weights[3] = {.2126, .7152, .0722};
        oiio::ImageBuf grayscaleBuf;
        oiio::ImageBufAlgo::channel_sum(grayscaleBuf, inBuf, weights, convertionROI);
        inBuf.copy(grayscaleBuf);

        // TODO: if inSpec.nchannels == 4: premult?
    }

    // duplicate first channel for RGB
    if (nchannels >= 3 && inSpec.nchannels == 1)
    {
        oiio::ImageSpec requestedSpec(inSpec.width, inSpec.height, 3, typeDesc);
        oiio::ImageBuf requestedBuf(requestedSpec);
        int channelOrder[] = { 0, 0, 0 };
        float channelValues[] = { 0 /*ignore*/, 0 /*ignore*/, 0 /*ignore*/ };
        oiio::ImageBufAlgo::channels(requestedBuf, inBuf, 3, channelOrder, channelValues);
        inBuf.swap(requestedBuf);
    }

    // Add an alpha channel if needed
    if (nchannels == 4 && inBuf.spec().nchannels == 3)
    {
        oiio::ImageSpec requestedSpec(inSpec.width, inSpec.height, 3, typeDesc);
        oiio::ImageBuf requestedBuf(requestedSpec);
        int channelOrder[] = { 0, 1, 2, -1 /*constant value*/ };
        float channelValues[] = { 0 /*ignore*/, 0 /*ignore*/, 0 /*ignore*/, 1.0 };
        oiio::ImageBufAlgo::channels(requestedBuf, inBuf,
                                     4, // create an image with 4 channels
                                     channelOrder,
                                     channelValues); // only the 4th value is used
        inBuf.swap(requestedBuf);
    }

    width = inSpec.width;
    height = inSpec.height;

    buffer.resize(inSpec.width * inSpec.height);

    {
        oiio::ROI exportROI = inBuf.roi();
        exportROI.chbegin = 0;
        exportROI.chend = nchannels;

        inBuf.get_pixels(exportROI, typeDesc, buffer.data());
    }
}

void readImage(const std::string& path, int& width, int& height, std::vector<unsigned char>& buffer,
               image::EImageColorSpace toColorSpace)
{
    readImage(path, oiio::TypeDesc::UCHAR, 1, width, height, buffer, toColorSpace);
}

void readImage(const std::string& path, int& width, int& height, std::vector<unsigned short>& buffer,
               image::EImageColorSpace toColorSpace)
{
    readImage(path, oiio::TypeDesc::UINT16, 1, width, height, buffer, toColorSpace);
}

void readImage(const std::string& path, int& width, int& height, std::vector<rgb>& buffer,
               image::EImageColorSpace toColorSpace)
{
    readImage(path, oiio::TypeDesc::UCHAR, 3, width, height, buffer, toColorSpace);
}

void readImage(const std::string& path, int& width, int& height, std::vector<float>& buffer,
               image::EImageColorSpace toColorSpace)
{
    readImage(path, oiio::TypeDesc::FLOAT, 1, width, height, buffer, toColorSpace);
}

void readImage(const std::string& path, int& width, int& height, std::vector<ColorRGBf>& buffer,
               image::EImageColorSpace toColorSpace)
{
    readImage(path, oiio::TypeDesc::FLOAT, 3, width, height, buffer, toColorSpace);
}

void readImage(const std::string& path, int& width, int& height, std::vector<ColorRGBAf>& buffer,
               image::EImageColorSpace toColorSpace)
{
    readImage(path, oiio::TypeDesc::FLOAT, 4, width, height, buffer, toColorSpace);
}

void readImage(const std::string& path, ImageRGBf& image, image::EImageColorSpace toColorSpace)
{
    int width, height;
    readImage(path, width, height, image.data(), toColorSpace);
    image.setWidth(width);
    image.setHeight(height);
}

void readImage(const std::string& path, ImageRGBAf& image, image::EImageColorSpace toColorSpace)
{
    int width, height;
    readImage(path, width, height, image.data(), toColorSpace);
    image.setWidth(width);
    image.setHeight(height);
}

template<typename T>
void writeImage(const std::string& path,
                oiio::TypeDesc typeDesc,
                int width,
                int height,
                int nchannels,
                const std::vector<T>& buffer,
                EImageQuality imageQuality,
                OutputFileColorSpace colorspace,
                const oiio::ParamValueList& metadata)
{
    const fs::path bPath = fs::path(path);
    const std::string extension = bPath.extension().string();
    const std::string tmpPath = (bPath.parent_path() / bPath.stem()).string() + "." + fs::unique_path().string() + extension;
    const bool isEXR = (extension == ".exr");
    //const bool isTIF = (extension == ".tif");
    const bool isJPG = (extension == ".jpg");
    const bool isPNG = (extension == ".png");

    if (colorspace.to == image::EImageColorSpace::AUTO)
    {
      if(isJPG || isPNG)
            colorspace.to = image::EImageColorSpace::SRGB;
      else
            colorspace.to = image::EImageColorSpace::LINEAR;
    }


    ALICEVISION_LOG_DEBUG("[IO] Write Image: " << path << std::endl
                       << "\t- width: " << width << std::endl
                       << "\t- height: " << height << std::endl
                       << "\t- channels: " << nchannels);

    oiio::ImageSpec imageSpec(width, height, nchannels, typeDesc);
    imageSpec.extra_attribs = metadata; // add custom metadata

    imageSpec.attribute("jpeg:subsampling", "4:4:4");           // if possible, always subsampling 4:4:4 for jpeg
    imageSpec.attribute("compression", isEXR ? "zips" : "none"); // if possible, set compression (zips for EXR, none for the other)

    const oiio::ImageBuf imgBuf = oiio::ImageBuf(imageSpec, const_cast<T*>(buffer.data())); // original image buffer
    const oiio::ImageBuf* outBuf = &imgBuf;  // buffer to write

    oiio::ImageBuf colorspaceBuf;  // buffer for image colorspace modification
    imageAlgo::colorconvert(colorspaceBuf, *outBuf, colorspace.from, colorspace.to);
    outBuf = &colorspaceBuf;

    oiio::ImageBuf formatBuf; // buffer for image format modification
    if(imageQuality == EImageQuality::OPTIMIZED && isEXR)
    {
      formatBuf.copy(*outBuf, oiio::TypeDesc::HALF); // override format, use half instead of float
      outBuf = &formatBuf;
    }

    // write image
    if(!outBuf->write(tmpPath))
      throw std::runtime_error("Can't write output image file '" + path + "'.");

    // rename temporary filename
    fs::rename(tmpPath, path);
}

void writeImage(const std::string& path, int width, int height, const std::vector<unsigned char>& buffer, EImageQuality imageQuality, const OutputFileColorSpace& colorspace, const oiio::ParamValueList& metadata)
{
    writeImage(path, oiio::TypeDesc::UCHAR, width, height, 1, buffer, imageQuality, colorspace, metadata);
}

void writeImage(const std::string& path, int width, int height, const std::vector<unsigned short>& buffer, EImageQuality imageQuality, const OutputFileColorSpace& colorspace, const oiio::ParamValueList& metadata)
{
    writeImage(path, oiio::TypeDesc::UINT16, width, height, 1, buffer, imageQuality, colorspace, metadata);
}

void writeImage(const std::string& path, int width, int height, const std::vector<rgb>& buffer, EImageQuality imageQuality, const OutputFileColorSpace& colorspace, const oiio::ParamValueList& metadata)
{
    writeImage(path, oiio::TypeDesc::UCHAR, width, height, 3, buffer, imageQuality, colorspace, metadata);
}

void writeImage(const std::string& path, int width, int height, const std::vector<float>& buffer, EImageQuality imageQuality, const OutputFileColorSpace& colorspace, const oiio::ParamValueList& metadata)
{
    writeImage(path, oiio::TypeDesc::FLOAT, width, height, 1, buffer, imageQuality, colorspace, metadata);
}

void writeImage(const std::string& path, int width, int height, const std::vector<ColorRGBf>& buffer, EImageQuality imageQuality, const OutputFileColorSpace& colorspace, const oiio::ParamValueList& metadata)
{
    writeImage(path, oiio::TypeDesc::FLOAT, width, height, 3, buffer, imageQuality, colorspace, metadata);
}

void writeImage(const std::string &path, ImageRGBf &image, EImageQuality imageQuality, const OutputFileColorSpace& colorspace, const oiio::ParamValueList& metadata)
{
    writeImage(path, oiio::TypeDesc::FLOAT, image.width(), image.height(), 3, image.data(), imageQuality, colorspace, metadata);
}

} // namespace imageIO
} // namespace aliceVision
