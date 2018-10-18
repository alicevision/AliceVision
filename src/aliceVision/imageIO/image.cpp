// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "image.hpp"
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsData/Color.hpp>
#include <aliceVision/mvsData/Rgb.hpp>

#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>

#include <OpenEXR/half.h>

#include <boost/filesystem.hpp>

#include <stdexcept>
#include <memory>

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

oiio::ParamValueList getMetadataFromMap(const std::map<std::string, std::string>& metadataMap)
{
  oiio::ParamValueList metadata;
  for(const auto& metadataPair : metadataMap)
    metadata.push_back(oiio::ParamValue(metadataPair.first, metadataPair.second));
  return metadata;
}

void readImageSpec(const std::string& path,
                   int& width,
                   int& height,
                   int& nchannels)
{
  ALICEVISION_LOG_DEBUG("[IO] Read Image Spec: " << path);
  std::unique_ptr<oiio::ImageInput> in(oiio::ImageInput::open(path));

  if(!in)
    throw std::runtime_error("Can't find/open image file '" + path + "'.");

  const oiio::ImageSpec &spec = in->spec();

  width = spec.width;
  height = spec.height;
  nchannels = spec.nchannels;

  in->close();
}

void readImageMetadata(const std::string& path, oiio::ParamValueList& metadata)
{
  ALICEVISION_LOG_DEBUG("[IO] Read Image Metadata: " << path);
  std::unique_ptr<oiio::ImageInput> in(oiio::ImageInput::open(path));

  if(!in)
    throw std::runtime_error("Can't find/open image file '" + path + "'.");

  metadata = in->spec().extra_attribs;

  in->close();
}

template<typename T>
void readImage(const std::string& path,
               oiio::TypeDesc typeDesc,
               int nchannels,
               int& width,
               int& height,
               std::vector<T>& buffer)
{
    ALICEVISION_LOG_DEBUG("[IO] Read Image: " << path);

    // check requested channels number
    assert(nchannels == 1 || nchannels >= 3);

    oiio::ImageSpec configSpec;

    // libRAW configuration
    configSpec.attribute("raw:auto_bright", 0);       // don't want exposure correction
    configSpec.attribute("raw:use_camera_wb", 1);     // want white balance correction
    configSpec.attribute("raw:ColorSpace", "sRGB");   // want colorspace sRGB
    configSpec.attribute("raw:use_camera_matrix", 3); // want to use embeded color profile

    oiio::ImageBuf inBuf(path, 0, 0, NULL, &configSpec);

    if(!inBuf.initialized())
        throw std::runtime_error("Can't find/open image file '" + path + "'.");

    const oiio::ImageSpec& inSpec = inBuf.spec();

    // check picture channels number
    if(inSpec.nchannels != 1 && inSpec.nchannels < 3)
        throw std::runtime_error("Can't load channels of image file '" + path + "'.");

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
    }

    // add missing channels
    if(nchannels > inSpec.nchannels)
    {
        oiio::ImageSpec requestedSpec(inSpec.width, inSpec.height, nchannels, typeDesc);
        oiio::ImageBuf requestedBuf(requestedSpec);

        // duplicate first channel for RGB
        if(requestedSpec.nchannels >= 3 && inSpec.nchannels < 3)
        {
            oiio::ImageBufAlgo::paste(requestedBuf, 0, 0, 0, 0, inBuf);
            oiio::ImageBufAlgo::paste(requestedBuf, 0, 0, 0, 1, inBuf);
            oiio::ImageBufAlgo::paste(requestedBuf, 0, 0, 0, 2, inBuf);
        }

        inBuf.copy(requestedBuf);
    }

    width = inSpec.width;
    height = inSpec.height;

    buffer.resize(inSpec.width * inSpec.height * nchannels);

    {
        oiio::ROI exportROI = inBuf.roi();
        exportROI.chbegin = 0;
        exportROI.chend = nchannels;

        inBuf.get_pixels(exportROI, typeDesc, buffer.data());
    }
}

void readImage(const std::string& path, int& width, int& height, std::vector<unsigned char>& buffer)
{
    readImage(path, oiio::TypeDesc::UCHAR, 1, width, height, buffer);
}

void readImage(const std::string& path, int& width, int& height, std::vector<unsigned short>& buffer)
{
    readImage(path, oiio::TypeDesc::UINT16, 1, width, height, buffer);
}

void readImage(const std::string& path, int& width, int& height, std::vector<rgb>& buffer)
{
    readImage(path, oiio::TypeDesc::UCHAR, 3, width, height, buffer);
}

void readImage(const std::string& path, int& width, int& height, std::vector<float>& buffer)
{
    readImage(path, oiio::TypeDesc::FLOAT, 1, width, height, buffer);
}

void readImage(const std::string& path, int& width, int& height, std::vector<Color>& buffer)
{
    readImage(path, oiio::TypeDesc::FLOAT, 3, width, height, buffer);
}

template<typename T>
void writeImage(const std::string& path,
                oiio::TypeDesc typeDesc,
                int width,
                int height,
                int nchannels,
                const std::vector<T>& buffer,
                EImageQuality imageQuality,
                const oiio::ParamValueList& metadata)
{
    const fs::path bPath = fs::path(path);
    const std::string extension = bPath.extension().string();
    const std::string tmpPath = (bPath.parent_path() / bPath.stem()).string() + "." + fs::unique_path().string() + extension;
    const bool isEXR = (extension == ".exr");

    ALICEVISION_LOG_DEBUG("[IO] Write Image: " << path << std::endl
      << "\t- width: " << width << std::endl
      << "\t- height: " << height << std::endl
      << "\t- channels: " << nchannels);

    oiio::ImageSpec imageSpec(width, height, nchannels, typeDesc);
    imageSpec.extra_attribs = metadata;

    if(isEXR)
    {
        imageSpec.attribute("compression", "piz");   // if possible, PIZ compression for openEXR
    }
    else
    {
        imageSpec.attribute("jpeg:subsampling", "4:4:4"); // if possible, always subsampling 4:4:4 for jpeg
        imageSpec.attribute("CompressionQuality", 100);   // if possible, best compression quality
        imageSpec.attribute("compression", "none");       // if possible, no compression
    }

    const oiio::ImageBuf outBuf(imageSpec, const_cast<T*>(buffer.data()));

    if(isEXR && imageQuality == EImageQuality::OPTIMIZED)
    {
        imageSpec.format = oiio::TypeDesc::HALF; // override format

        // conversion to half
        oiio::ImageBuf halfBuf(imageSpec);
        if(!halfBuf.copy_pixels(outBuf))
          throw std::runtime_error("Can't convert output image file to half '" + path + "'.");

        // write image
        if(!halfBuf.write(tmpPath))
          throw std::runtime_error("Can't write output image file '" + path + "'.");
    }
    else
    {
        // write image
        if(!outBuf.write(tmpPath))
          throw std::runtime_error("Can't write output image file '" + path + "'.");
    }

    // rename temporay filename
    fs::rename(tmpPath, path);
}

void writeImage(const std::string& path, int width, int height, const std::vector<unsigned char>& buffer, EImageQuality imageQuality, const oiio::ParamValueList& metadata)
{
    writeImage(path, oiio::TypeDesc::UCHAR, width, height, 1, buffer, imageQuality, metadata);
}

void writeImage(const std::string& path, int width, int height, const std::vector<unsigned short>& buffer, EImageQuality imageQuality, const oiio::ParamValueList& metadata)
{
    writeImage(path, oiio::TypeDesc::UINT16, width, height, 1, buffer, imageQuality, metadata);
}

void writeImage(const std::string& path, int width, int height, const std::vector<rgb>& buffer, EImageQuality imageQuality, const oiio::ParamValueList& metadata)
{
    writeImage(path, oiio::TypeDesc::UCHAR, width, height, 3, buffer, imageQuality, metadata);
}

void writeImage(const std::string& path, int width, int height, const std::vector<float>& buffer, EImageQuality imageQuality, const oiio::ParamValueList& metadata)
{
    writeImage(path, oiio::TypeDesc::FLOAT, width, height, 1, buffer, imageQuality, metadata);
}

void writeImage(const std::string& path, int width, int height, const std::vector<Color>& buffer, EImageQuality imageQuality, const oiio::ParamValueList& metadata)
{
    writeImage(path, oiio::TypeDesc::FLOAT, width, height, 3, buffer, imageQuality, metadata);
}

template<typename T>
void transposeImage(oiio::TypeDesc typeDesc,
                    int width,
                    int height,
                    int nchannels,
                    std::vector<T>& buffer)
{
    oiio::ImageSpec imageSpec(width, height, nchannels, typeDesc);

    oiio::ImageBuf inBuf(imageSpec, buffer.data());
    oiio::ImageBuf transposeBuf;

    oiio::ImageBufAlgo::transpose(transposeBuf, inBuf, oiio::ROI::All());

    transposeBuf.get_pixels(oiio::ROI::All(), typeDesc, buffer.data());
}

void transposeImage(int width, int height, std::vector<unsigned char>& buffer)
{
    transposeImage(oiio::TypeDesc::UCHAR, width, height, 1, buffer);
}

void transposeImage(int width, int height, std::vector<rgb>& buffer)
{
    transposeImage(oiio::TypeDesc::UCHAR, width, height, 3, buffer);
}

void transposeImage(int width, int height, std::vector<float>& buffer)
{
    transposeImage(oiio::TypeDesc::FLOAT, width, height, 1, buffer);
}

void transposeImage(int width, int height, std::vector<Color>& buffer)
{
    transposeImage(oiio::TypeDesc::FLOAT, width, height, 3, buffer);
}

template<typename T>
void resizeImage(oiio::TypeDesc typeDesc,
                 int inWidth,
                 int inHeight,
                 int nchannels,
                 int downscale,
                 const std::vector<T>& inBuffer,
                 std::vector<T>& outBuffer,
                 const std::string& filter = "",
                 float filterSize = 0)
{
    const int outWidth = inWidth / downscale;
    const int outHeight = inHeight / downscale;

    outBuffer.resize(outWidth * outHeight);

    const oiio::ImageBuf inBuf(oiio::ImageSpec(inWidth, inHeight, nchannels, typeDesc), const_cast<T*>(inBuffer.data()));
    oiio::ImageBuf outBuf(oiio::ImageSpec(outWidth, outHeight, nchannels, typeDesc), outBuffer.data());

    oiio::ImageBufAlgo::resize(outBuf, inBuf, filter, filterSize, oiio::ROI::All());
}

void resizeImage(int inWidth, int inHeight, int downscale, const std::vector<unsigned char>& inBuffer, std::vector<unsigned char>& outBuffer, const std::string& filter, float filterSize)
{
    resizeImage(oiio::TypeDesc::UCHAR, inWidth, inHeight, 1, downscale, inBuffer, outBuffer, filter, filterSize);
}

void resizeImage(int inWidth, int inHeight, int downscale, const std::vector<rgb>& inBuffer, std::vector<rgb>& outBuffer, const std::string& filter, float filterSize)
{
    resizeImage(oiio::TypeDesc::UCHAR, inWidth, inHeight, 3, downscale, inBuffer, outBuffer, filter, filterSize);
}

void resizeImage(int inWidth, int inHeight, int downscale, const std::vector<float>& inBuffer, std::vector<float>& outBuffer, const std::string& filter, float filterSize)
{
    resizeImage(oiio::TypeDesc::FLOAT, inWidth, inHeight, 1, downscale, inBuffer, outBuffer, filter, filterSize);
}

void resizeImage(int inWidth, int inHeight, int downscale, const std::vector<Color>& inBuffer, std::vector<Color>& outBuffer, const std::string& filter, float filterSize)
{
    resizeImage(oiio::TypeDesc::FLOAT, inWidth, inHeight, 3, downscale, inBuffer, outBuffer, filter, filterSize);
}

template<typename T>
void convolveImage(oiio::TypeDesc typeDesc,
                   int inWidth,
                   int inHeight,
                   int nchannels,
                   const std::vector<T>& inBuffer,
                   std::vector<T>& outBuffer,
                   const std::string& kernel,
                   float kernelWidth,
                   float kernelHeight)
{
    outBuffer.resize(inBuffer.size());

    const oiio::ImageBuf inBuf(oiio::ImageSpec(inWidth, inHeight, nchannels, typeDesc), const_cast<T*>(inBuffer.data()));
    oiio::ImageBuf outBuf(oiio::ImageSpec(inWidth, inHeight, nchannels, typeDesc), outBuffer.data());

    oiio::ImageBuf K;
    oiio::ImageBufAlgo::make_kernel(K, kernel, kernelWidth, kernelHeight);

    oiio::ImageBufAlgo::convolve(outBuf, inBuf, K);
}


void convolveImage(int inWidth, int inHeight, const std::vector<unsigned char>& inBuffer, std::vector<unsigned char>& outBuffer, const std::string& kernel, float kernelWidth, float kernelHeight)
{
  convolveImage(oiio::TypeDesc::UCHAR, inWidth, inHeight, 1, inBuffer, outBuffer, kernel, kernelWidth, kernelHeight);
}

void convolveImage(int inWidth, int inHeight, const std::vector<rgb>& inBuffer, std::vector<rgb>& outBuffer, const std::string& kernel, float kernelWidth, float kernelHeight)
{
  convolveImage(oiio::TypeDesc::UCHAR, inWidth, inHeight, 3, inBuffer, outBuffer, kernel, kernelWidth, kernelHeight);
}

void convolveImage(int inWidth, int inHeight, const std::vector<float>& inBuffer, std::vector<float>& outBuffer, const std::string& kernel, float kernelWidth, float kernelHeight)
{
  convolveImage(oiio::TypeDesc::FLOAT, inWidth, inHeight, 1, inBuffer, outBuffer, kernel, kernelWidth, kernelHeight);
}

void convolveImage(int inWidth, int inHeight, const std::vector<Color>& inBuffer, std::vector<Color>& outBuffer, const std::string& kernel, float kernelWidth, float kernelHeight)
{
  convolveImage(oiio::TypeDesc::FLOAT, inWidth, inHeight, 3, inBuffer, outBuffer, kernel, kernelWidth, kernelHeight);
}

void fillHoles(int inWidth, int inHeight, std::vector<Color>& colorBuffer, const std::vector<float>& alphaBuffer)
{
    oiio::ImageBuf rgbBuf(oiio::ImageSpec(inWidth, inHeight, 3, oiio::TypeDesc::FLOAT), colorBuffer.data());
    const oiio::ImageBuf alphaBuf(oiio::ImageSpec(inWidth, inHeight, 1, oiio::TypeDesc::FLOAT), const_cast<float*>(alphaBuffer.data()));

    // Create RGBA ImageBuf from source buffers with correct channel names
    // (identified alpha channel is needed for fillholes_pushpull)
    oiio::ImageBuf rgbaBuf;
    oiio::ImageBufAlgo::channel_append(rgbaBuf, rgbBuf, alphaBuf);
    rgbaBuf.specmod().default_channel_names();

    // Temp RGBA buffer to store fillholes result
    oiio::ImageBuf filledBuf;
    oiio::ImageBufAlgo::fillholes_pushpull(filledBuf, rgbaBuf);
    rgbaBuf.clear();

    // Copy result to original RGB buffer
    oiio::ImageBufAlgo::copy(rgbBuf, filledBuf);
}

} // namespace imageIO
} // namespace aliceVision
