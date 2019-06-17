// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/image/all.hpp>

#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>

#include <OpenEXR/half.h>

#include <boost/filesystem.hpp>

#include <cstring>
#include <stdexcept>
#include <iostream>
#include <cmath>

namespace fs = boost::filesystem;

namespace aliceVision {
namespace image {

std::string EImageFileType_informations()
{
  return "Image file type :\n"
         "* jpg \n"
         "* png \n"
         "* tif \n"
         "* exr (half)";
}

EImageFileType EImageFileType_stringToEnum(const std::string& imageFileType)
{
  std::string type = imageFileType;
  std::transform(type.begin(), type.end(), type.begin(), ::tolower); //tolower

  if(type == "jpg" || type == "jpeg") return EImageFileType::JPEG;
  if(type == "png")                   return EImageFileType::PNG;
  if(type == "tif" || type == "tiff") return EImageFileType::TIFF;
  if(type == "exr")                   return EImageFileType::EXR;

  throw std::out_of_range("Invalid image file type : " + imageFileType);
}

std::string EImageFileType_enumToString(const EImageFileType imageFileType)
{
  switch(imageFileType)
  {
    case EImageFileType::JPEG:  return "jpg";
    case EImageFileType::PNG:   return "png";
    case EImageFileType::TIFF:  return "tif";
    case EImageFileType::EXR:   return "exr";
  }
  throw std::out_of_range("Invalid EImageType enum");
}

std::ostream& operator<<(std::ostream& os, EImageFileType imageFileType)
{
  return os << EImageFileType_enumToString(imageFileType);
}

std::istream& operator>>(std::istream& in, EImageFileType& imageFileType)
{
  std::string token;
  in >> token;
  imageFileType = EImageFileType_stringToEnum(token);
  return in;
}

oiio::ParamValueList getMetadataFromMap(const std::map<std::string, std::string>& metadataMap)
{
  oiio::ParamValueList metadata;
  for(const auto& metadataPair : metadataMap)
    metadata.push_back(oiio::ParamValue(metadataPair.first, metadataPair.second));
  return metadata;
}

oiio::ParamValueList readImageMetadata(const std::string& path)
{
  std::unique_ptr<oiio::ImageInput> in(oiio::ImageInput::open(path));

  if(!in)
    throw std::runtime_error("Can't find/open image file '" + path + "'.");

  oiio::ParamValueList metadata = in->spec().extra_attribs;

  in->close();

  return metadata;
}

void readImageMetadata(const std::string& path, int& width, int& height, std::map<std::string, std::string>& metadata)
{
  std::unique_ptr<oiio::ImageInput> in(oiio::ImageInput::open(path));

  if(!in)
    throw std::runtime_error("Can't find/open image file '" + path + "'.");

  const oiio::ImageSpec &spec = in->spec();

  width = spec.width;
  height = spec.height;

  for(const auto& param : spec.extra_attribs)
    metadata.emplace(param.name().string(), param.get_string());

  in->close();
}

template<typename T>
void getBufferFromImage(Image<T>& image,
                        oiio::TypeDesc format,
                        int nchannels,
                        oiio::ImageBuf& buffer)
{
  const oiio::ImageSpec imageSpec(image.Width(), image.Height(), nchannels, format);
  oiio::ImageBuf imageBuf(imageSpec, image.data());
  buffer.swap(imageBuf);
}

void getBufferFromImage(Image<float>& image, oiio::ImageBuf& buffer)
{
  getBufferFromImage(image, oiio::TypeDesc::FLOAT, 1, buffer);
}

void getBufferFromImage(Image<unsigned char>& image, oiio::ImageBuf& buffer)
{
  getBufferFromImage(image, oiio::TypeDesc::UINT8, 1, buffer);
}

void getBufferFromImage(Image<RGBAColor>& image, oiio::ImageBuf& buffer)
{
  getBufferFromImage(image, oiio::TypeDesc::UINT8, 4, buffer);
}

void getBufferFromImage(Image<RGBfColor>& image, oiio::ImageBuf& buffer)
{
  getBufferFromImage(image, oiio::TypeDesc::FLOAT, 3, buffer);
}

void getBufferFromImage(Image<RGBColor>& image, oiio::ImageBuf& buffer)
{
  getBufferFromImage(image, oiio::TypeDesc::UINT8, 3, buffer);
}

template<typename T>
void readImage(const std::string& path,
               oiio::TypeDesc format,
               int nchannels,
               Image<T>& image,
               EImageColorSpace imageColorSpace)
{
  // check requested channels number
  assert(nchannels == 1 || nchannels >= 3);

  oiio::ImageSpec configSpec;

  // libRAW configuration
  configSpec.attribute("raw:auto_bright", 0);       // don't want exposure correction
  configSpec.attribute("raw:use_camera_wb", 1);     // want white balance correction
  configSpec.attribute("raw:ColorSpace", "sRGB");   // want colorspace sRGB
  configSpec.attribute("raw:use_camera_matrix", 3); // want to use embeded color profile

  oiio::ImageBuf inBuf(path, 0, 0, NULL, &configSpec);

  inBuf.read(0, 0, true, oiio::TypeDesc::FLOAT); // force image convertion to float (for grayscale and color space convertion)

  if(!inBuf.initialized())
    throw std::runtime_error("Can't find/open image file '" + path + "'.");

  const oiio::ImageSpec& inSpec = inBuf.spec();

  // check picture channels number
  if(inSpec.nchannels != 1 && inSpec.nchannels < 3)
    throw std::runtime_error("Can't load channels of image file '" + path + "'.");

  // color conversion
  if(imageColorSpace == EImageColorSpace::AUTO)
    throw std::runtime_error("You must specify a requested color space for image file '" + path + "'.");

  if(imageColorSpace == EImageColorSpace::SRGB) // color conversion to sRGB
  {
    const std::string& colorSpace = inSpec.get_string_attribute("oiio:ColorSpace", "sRGB"); // default image color space is sRGB
    if(colorSpace != "sRGB")
    {
     oiio::ImageBufAlgo::colorconvert(inBuf, inBuf, colorSpace, "sRGB");
     ALICEVISION_LOG_INFO("Convert image " << path << " to sRGB colorspace");
    }
  }
  else if(imageColorSpace == EImageColorSpace::LINEAR) // color conversion to linear
  {
    const std::string& colorSpace = inSpec.get_string_attribute("oiio:ColorSpace", "sRGB");
    if(colorSpace != "Linear")
     oiio::ImageBufAlgo::colorconvert(inBuf, inBuf, colorSpace, "Linear");
  }

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
    oiio::ImageSpec requestedSpec(inSpec.width, inSpec.height, nchannels, format);
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

  // copy pixels from oiio to eigen
  image.resize(inSpec.width, inSpec.height, false);
  {
    oiio::ROI exportROI = inBuf.roi();
    exportROI.chbegin = 0;
    exportROI.chend = nchannels;

    inBuf.get_pixels(exportROI, format, image.data());
  }
}

template<typename T>
void writeImage(const std::string& path,
                oiio::TypeDesc typeDesc,
                int nchannels,
                const Image<T>& image,
                EImageColorSpace imageColorSpace,
                const oiio::ParamValueList& metadata = oiio::ParamValueList())
{
  const fs::path bPath = fs::path(path);
  const std::string extension = bPath.extension().string();
  const std::string tmpPath =  (bPath.parent_path() / bPath.stem()).string() + "." + fs::unique_path().string() + extension;
  const bool isEXR = (extension == ".exr");
  //const bool isTIF = (extension == ".tif");
  const bool isJPG = (extension == ".jpg");
  const bool isPNG = (extension == ".png");

  if(imageColorSpace == EImageColorSpace::AUTO)
  {
    if(isJPG || isPNG)
      imageColorSpace = EImageColorSpace::SRGB;
    else
      imageColorSpace = EImageColorSpace::LINEAR;
  }

  oiio::ImageSpec imageSpec(image.Width(), image.Height(), nchannels, typeDesc);
  imageSpec.extra_attribs = metadata; // add custom metadata

  imageSpec.attribute("jpeg:subsampling", "4:4:4");           // if possible, always subsampling 4:4:4 for jpeg
  imageSpec.attribute("CompressionQuality", 100);             // if possible, best compression quality
  imageSpec.attribute("compression", isEXR ? "piz" : "none"); // if possible, set compression (piz for EXR, none for the other)

  const oiio::ImageBuf imgBuf = oiio::ImageBuf(imageSpec, const_cast<T*>(image.data())); // original image buffer
  const oiio::ImageBuf* outBuf = &imgBuf;  // buffer to write

  oiio::ImageBuf colorspaceBuf; // buffer for image colorspace modification
  if(imageColorSpace == EImageColorSpace::SRGB)
  {
    oiio::ImageBufAlgo::colorconvert(colorspaceBuf, *outBuf, "Linear", "sRGB");
    outBuf = &colorspaceBuf;
  }

  oiio::ImageBuf formatBuf;  // buffer for image format modification
  if(isEXR)
  {
    formatBuf.copy(*outBuf, oiio::TypeDesc::HALF); // override format, use half instead of float
    outBuf = &formatBuf;
  }

  // write image
  if(!outBuf->write(tmpPath))
    throw std::runtime_error("Can't write output image file '" + path + "'.");

  // rename temporay filename
  fs::rename(tmpPath, path);
}

void readImage(const std::string& path, Image<float>& image, EImageColorSpace imageColorSpace)
{
  readImage(path, oiio::TypeDesc::FLOAT, 1, image, imageColorSpace);
}

void readImage(const std::string& path, Image<unsigned char>& image, EImageColorSpace imageColorSpace)
{
  readImage(path, oiio::TypeDesc::UINT8, 1, image, imageColorSpace);
}

void readImage(const std::string& path, Image<RGBAColor>& image, EImageColorSpace imageColorSpace)
{
  readImage(path, oiio::TypeDesc::UINT8, 4, image, imageColorSpace);
}

void readImage(const std::string& path, Image<RGBfColor>& image, EImageColorSpace imageColorSpace)
{
  readImage(path, oiio::TypeDesc::FLOAT, 3, image, imageColorSpace);
}

void readImage(const std::string& path, Image<RGBColor>& image, EImageColorSpace imageColorSpace)
{
  readImage(path, oiio::TypeDesc::UINT8, 3, image, imageColorSpace);
}

void writeImage(const std::string& path, const Image<unsigned char>& image, EImageColorSpace imageColorSpace, const oiio::ParamValueList& metadata)
{
  writeImage(path, oiio::TypeDesc::UINT8, 1, image, imageColorSpace, metadata);
}

void writeImage(const std::string& path, const Image<RGBAColor>& image, EImageColorSpace imageColorSpace, const oiio::ParamValueList& metadata)
{
  writeImage(path, oiio::TypeDesc::UINT8, 4, image, imageColorSpace, metadata);
}

void writeImage(const std::string& path, const Image<RGBfColor>& image, EImageColorSpace imageColorSpace, const oiio::ParamValueList& metadata)
{
  writeImage(path, oiio::TypeDesc::FLOAT, 3, image, imageColorSpace, metadata);
}

void writeImage(const std::string& path, const Image<RGBColor>& image, EImageColorSpace imageColorSpace, const oiio::ParamValueList& metadata)
{
  writeImage(path, oiio::TypeDesc::UINT8, 3, image, imageColorSpace, metadata);
}

}  // namespace image
}  // namespace aliceVision
