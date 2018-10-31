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
void readImage(const std::string& path, oiio::TypeDesc format, int nchannels, Image<T>& image)
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
                const oiio::ParamValueList& metadata = oiio::ParamValueList())
{
  const fs::path bPath = fs::path(path);
  const std::string extension = bPath.extension().string();
  const std::string tmpPath =  (bPath.parent_path() / bPath.stem()).string() + "." + fs::unique_path().string() + extension;
  const bool isEXR = (extension == ".exr");

  oiio::ImageSpec imageSpec(image.Width(), image.Height(), nchannels, typeDesc);
  imageSpec.extra_attribs = metadata; // add custom metadata

  if(isEXR)
  {
    oiio::ImageBuf buf(imageSpec, const_cast<T*>(image.data()));

    imageSpec.format = oiio::TypeDesc::HALF;     // override format
    imageSpec.attribute("compression", "piz");   // if possible, PIZ compression for openEXR

    // conversion to half
    oiio::ImageBuf outBuf(imageSpec);
    outBuf.copy_pixels(buf);

    // write image
    if(!outBuf.write(tmpPath))
      throw std::runtime_error("Can't write output image file '" + path + "'.");
  }
  else
  {
    imageSpec.attribute("jpeg:subsampling", "4:4:4"); // if possible, always subsampling 4:4:4 for jpeg
    imageSpec.attribute("CompressionQuality", 100);   // if possible, best compression quality
    imageSpec.attribute("compression", "none");       // if possible, no compression

    oiio::ImageBuf outBuf(imageSpec, const_cast<T*>(image.data()));

    // write image
    if(!outBuf.write(tmpPath))
      throw std::runtime_error("Can't write output image file '" + path + "'.");
  }

  // rename temporay filename
  fs::rename(tmpPath, path);
}

void readImage(const std::string& path, Image<float>& image)
{
  readImage(path, oiio::TypeDesc::FLOAT, 1, image);
}

void readImage(const std::string& path, Image<unsigned char>& image)
{
  readImage(path, oiio::TypeDesc::UINT8, 1, image);
}

void readImage(const std::string& path, Image<RGBAColor>& image)
{
  readImage(path, oiio::TypeDesc::UINT8, 4, image);
}

void readImage(const std::string& path, Image<RGBfColor>& image)
{
  readImage(path, oiio::TypeDesc::FLOAT, 3, image);
}

void readImage(const std::string& path, Image<RGBColor>& image)
{
  readImage(path, oiio::TypeDesc::UINT8, 3, image);
}

void writeImage(const std::string& path, const Image<unsigned char>& image, const oiio::ParamValueList& metadata)
{
  writeImage(path, oiio::TypeDesc::UINT8, 1, image, metadata);
}

void writeImage(const std::string& path, const Image<RGBAColor>& image, const oiio::ParamValueList& metadata)
{
  writeImage(path, oiio::TypeDesc::UINT8, 4, image, metadata);
}

void writeImage(const std::string& path, const Image<RGBfColor>& image, const oiio::ParamValueList& metadata)
{
  writeImage(path, oiio::TypeDesc::FLOAT, 3, image, metadata);
}

void writeImage(const std::string& path, const Image<RGBColor>& image, const oiio::ParamValueList& metadata)
{
  writeImage(path, oiio::TypeDesc::UINT8, 3, image, metadata);
}

}  // namespace image
}  // namespace aliceVision
