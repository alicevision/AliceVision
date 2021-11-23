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
#include <boost/algorithm/string.hpp>

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

  throw std::out_of_range("Invalid image file type: " + imageFileType);
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

std::vector<std::string> getSupportedExtensions()
{
  static const std::string extensionList = oiio::get_string_attribute("extension_list");
  std::vector<std::string> supportedExtensions;

  std::vector<std::string> supportedFormat;
  boost::split(supportedFormat, extensionList, boost::is_any_of(";"), boost::token_compress_on);
  for(const std::string& format: supportedFormat)
  {
    std::vector<std::string> extensions;
    const std::string str = format.substr(format.find(":")+1);
    boost::split(extensions, str, boost::is_any_of(","), boost::token_compress_on);
    for(std::string& extension: extensions)
      supportedExtensions.push_back(extension.insert(0, "."));
  }
  return supportedExtensions;
}

bool isSupported(const std::string& ext)
{
  static const std::vector<std::string> supportedExtensions = getSupportedExtensions();
  const auto start = supportedExtensions.begin();
  const auto end = supportedExtensions.end();
  return (std::find(start, end, boost::to_lower_copy(ext)) != end);
}


std::string EStorageDataType_informations()
{
    return EStorageDataType_enumToString(EStorageDataType::Float) + ", " +
        EStorageDataType_enumToString(EStorageDataType::Half) + ", " +
        EStorageDataType_enumToString(EStorageDataType::HalfFinite) + ", " +
        EStorageDataType_enumToString(EStorageDataType::Auto);
}

EStorageDataType EStorageDataType_stringToEnum(const std::string& dataType)
{
    std::string type = dataType;
    std::transform(type.begin(), type.end(), type.begin(), ::tolower); //tolower

    if (type == "float") return EStorageDataType::Float;
    if (type == "half") return EStorageDataType::Half;
    if (type == "halffinite") return EStorageDataType::HalfFinite;
    if (type == "auto") return EStorageDataType::Auto;

    throw std::out_of_range("Invalid EStorageDataType: " + dataType);
}

std::string EStorageDataType_enumToString(const EStorageDataType dataType)
{
    switch (dataType)
    {
    case EStorageDataType::Float:  return "float";
    case EStorageDataType::Half:   return "half";
    case EStorageDataType::HalfFinite:  return "halfFinite";
    case EStorageDataType::Auto:   return "auto";
    }
    throw std::out_of_range("Invalid EStorageDataType enum");
}

std::ostream& operator<<(std::ostream& os, EStorageDataType dataType)
{
    return os << EStorageDataType_enumToString(dataType);
}

std::istream& operator>>(std::istream& in, EStorageDataType& dataType)
{
    std::string token;
    in >> token;
    dataType = EStorageDataType_stringToEnum(token);
    return in;
}

// Warning: type conversion problems from string to param value, we may lose some metadata with string maps
oiio::ParamValueList getMetadataFromMap(const std::map<std::string, std::string>& metadataMap)
{
  oiio::ParamValueList metadata;
  for(const auto& metadataPair : metadataMap)
    metadata.push_back(oiio::ParamValue(metadataPair.first, metadataPair.second));
  return metadata;
}

oiio::ParamValueList readImageMetadata(const std::string& path, int& width, int& height)
{
  std::unique_ptr<oiio::ImageInput> in(oiio::ImageInput::open(path));
  oiio::ImageSpec spec = in->spec();

  if(!in)
    throw std::runtime_error("Can't find/open image file '" + path + "'.");

#if OIIO_VERSION <= (10000 * 2 + 100 * 0 + 8) // OIIO_VERSION <= 2.0.8
  const std::string formatStr = in->format_name();
  if(formatStr == "raw")
  {
    // For the RAW plugin: override colorspace as linear (as the content is linear with sRGB primaries but declared as sRGB)
    spec.attribute("oiio:ColorSpace", "Linear");
    ALICEVISION_LOG_TRACE("OIIO workaround: RAW input image " << path << " is in Linear.");
  }
#endif

  width = spec.width;
  height = spec.height;

  oiio::ParamValueList metadata = spec.extra_attribs;

  in->close();

  return metadata;
}

oiio::ParamValueList readImageMetadata(const std::string& path)
{
  int w, h;
  return readImageMetadata(path, w, h);
}

// Warning: type conversion problems from string to param value, we may lose some metadata with string maps
void readImageMetadata(const std::string& path, int& width, int& height, std::map<std::string, std::string>& metadata)
{
  oiio::ParamValueList oiioMetadadata = readImageMetadata(path, width, height);

  for(const auto& param : oiioMetadadata)
    metadata.emplace(param.name().string(), param.get_string());
}

void readImageSize(const std::string& path, int& width, int& height)
{
    std::map<std::string, std::string> metadata;
    readImageMetadata(path, width, height, metadata);
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

void getBufferFromImage(Image<RGBAfColor>& image, oiio::ImageBuf& buffer)
{
  getBufferFromImage(image, oiio::TypeDesc::FLOAT, 4, buffer);
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
               const ImageReadOptions & imageReadOptions)
{
  // check requested channels number
  assert(nchannels == 1 || nchannels >= 3);

  if(!fs::exists(path))
    ALICEVISION_THROW_ERROR("No such image file: '" << path << "'.");

  oiio::ImageSpec configSpec;

  // libRAW configuration
  // See https://openimageio.readthedocs.io/en/master/builtinplugins.html#raw-digital-camera-files
  configSpec.attribute("raw:auto_bright", 0); // disable exposure correction
  configSpec.attribute("raw:use_camera_wb", (imageReadOptions.applyWhiteBalance?1:0)); // white balance correction
  // use_camera_matrix: Whether to use the embedded color profile, if it is present: 0=never, 1 (default)=only for DNG files, 3=always
  configSpec.attribute("raw:use_camera_matrix", 3); // use embeded color profile
#if OIIO_VERSION <= (10000 * 2 + 100 * 0 + 8) // OIIO_VERSION <= 2.0.8
  // In old versions of oiio, there was no Linear option
  configSpec.attribute("raw:ColorSpace", "sRGB"); // use colorspace sRGB
#else
  configSpec.attribute("raw:ColorSpace", "Linear"); // use linear colorspace with sRGB primaries
#endif

  oiio::ImageBuf inBuf(path, 0, 0, NULL, &configSpec);

  inBuf.read(0, 0, true, oiio::TypeDesc::FLOAT); // force image convertion to float (for grayscale and color space convertion)

  if(!inBuf.initialized())
    ALICEVISION_THROW_ERROR("Failed to open the image file: '" << path << "'.");

  // check picture channels number
  if(inBuf.spec().nchannels != 1 && inBuf.spec().nchannels < 3)
    ALICEVISION_THROW_ERROR("Can't load channels of image file: '" << path << "', nchannels=" << inBuf.spec().nchannels);

  // color conversion
  if(imageReadOptions.outputColorSpace == EImageColorSpace::AUTO)
    throw std::runtime_error("You must specify a requested color space for image file '" + path + "'.");

  const std::string& colorSpace = inBuf.spec().get_string_attribute("oiio:ColorSpace", "sRGB"); // default image color space is sRGB
  ALICEVISION_LOG_TRACE("Read image " << path << " (encoded in " << colorSpace << " colorspace).");

  if(imageReadOptions.outputColorSpace == EImageColorSpace::SRGB) // color conversion to sRGB
  {
    if (colorSpace != "sRGB")
    {
      oiio::ImageBufAlgo::colorconvert(inBuf, inBuf, colorSpace, "sRGB");
      ALICEVISION_LOG_TRACE("Convert image " << path << " from " << colorSpace << " to sRGB colorspace");
    }
  }
  else if(imageReadOptions.outputColorSpace == EImageColorSpace::LINEAR) // color conversion to linear
  {
    if (colorSpace != "Linear")
    {
      oiio::ImageBufAlgo::colorconvert(inBuf, inBuf, colorSpace, "Linear");
      ALICEVISION_LOG_TRACE("Convert image " << path << " from " << colorSpace << " to Linear colorspace");
    }
  }

  // convert to grayscale if needed
  if(nchannels == 1 && inBuf.spec().nchannels >= 3)
  {
    // convertion region of interest (for inBuf.spec().nchannels > 3)
    oiio::ROI convertionROI = inBuf.roi();
    convertionROI.chbegin = 0;
    convertionROI.chend = 3;

    // compute luminance via a weighted sum of R,G,B
    // (assuming Rec709 primaries and a linear scale)
    const float weights[3] = {.2126f, .7152f, .0722f};
    oiio::ImageBuf grayscaleBuf;
    oiio::ImageBufAlgo::channel_sum(grayscaleBuf, inBuf, weights, convertionROI);
    inBuf.copy(grayscaleBuf);

    // TODO: if inBuf.spec().nchannels == 4: premult?
  }

  // duplicate first channel for RGB
  if (nchannels >= 3 && inBuf.spec().nchannels == 1)
  {
    oiio::ImageSpec requestedSpec(inBuf.spec().width, inBuf.spec().height, 3, format);
    oiio::ImageBuf requestedBuf(requestedSpec);
    int channelOrder[] = { 0, 0, 0 };
    float channelValues[] = { 0 /*ignore*/, 0 /*ignore*/, 0 /*ignore*/ };
    oiio::ImageBufAlgo::channels(requestedBuf, inBuf, 3, channelOrder, channelValues);
    inBuf.swap(requestedBuf);
  }

  // Add an alpha channel if needed
  if (nchannels == 4 && inBuf.spec().nchannels == 3)
  {
    oiio::ImageSpec requestedSpec(inBuf.spec().width, inBuf.spec().height, 3, format);
    oiio::ImageBuf requestedBuf(requestedSpec);
    int channelOrder[] = { 0, 1, 2, -1 /*constant value*/ };
    float channelValues[] = { 0 /*ignore*/, 0 /*ignore*/, 0 /*ignore*/, 1.0 };
    oiio::ImageBufAlgo::channels(requestedBuf, inBuf,
                                  4, // create an image with 4 channels
                                  channelOrder,
                                  channelValues); // only the 4th value is used
    inBuf.swap(requestedBuf);
  }

  // copy pixels from oiio to eigen
  image.resize(inBuf.spec().width, inBuf.spec().height, false);
  {
    oiio::ROI exportROI = inBuf.roi();
    exportROI.chbegin = 0;
    exportROI.chend = nchannels;

    inBuf.get_pixels(exportROI, format, image.data());
  }
}

template<typename T>
void readImageNoFloat(const std::string& path,
               oiio::TypeDesc format,
               Image<T>& image)
{
  oiio::ImageSpec configSpec;

  oiio::ImageBuf inBuf(path, 0, 0, NULL, &configSpec);

  inBuf.read(0, 0, true, format);

  if(!inBuf.initialized())
  {
    throw std::runtime_error("Cannot find/open image file '" + path + "'.");
  }

  // check picture channels number
  if(inBuf.spec().nchannels != 1)
  {
    throw std::runtime_error("Can't load channels of image file '" + path + "'.");
  }
    
  // copy pixels from oiio to eigen
  image.resize(inBuf.spec().width, inBuf.spec().height, false);
  {
    oiio::ROI exportROI = inBuf.roi();
    exportROI.chbegin = 0;
    exportROI.chend = 1;

    inBuf.get_pixels(exportROI, format, image.data());
  }
}

bool containsHalfFloatOverflow(const oiio::ImageBuf& image)
{
    oiio::ImageBufAlgo::PixelStats stats;
    oiio::ImageBufAlgo::computePixelStats(stats, image);

    for(auto maxValue: stats.max)
    {
        if(maxValue > HALF_MAX)
            return true;
    }
    for (auto minValue: stats.min)
    {
        if (minValue < -HALF_MAX)
            return true;
    }
    return false;
}

template<typename T>
void writeImage(const std::string& path,
                oiio::TypeDesc typeDesc,
                int nchannels,
                const Image<T>& image,
                EImageColorSpace imageColorSpace,
                const oiio::ParamValueList& metadata = oiio::ParamValueList(),
                const oiio::ROI& roi = oiio::ROI())
{
  const fs::path bPath = fs::path(path);
  const std::string extension = boost::to_lower_copy(bPath.extension().string());
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
  imageSpec.attribute("compression", isEXR ? "zips" : "none"); // if possible, set compression (zips for EXR, none for the other)
  if(roi.defined() && isEXR)
  {
      imageSpec.set_roi_full(roi);
  }

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
    const std::string storageDataTypeStr = imageSpec.get_string_attribute("AliceVision:storageDataType", EStorageDataType_enumToString(EStorageDataType::HalfFinite));
    EStorageDataType storageDataType  = EStorageDataType_stringToEnum(storageDataTypeStr);

    if (storageDataType == EStorageDataType::Auto)
    {
        if (containsHalfFloatOverflow(*outBuf))
        {
            storageDataType = EStorageDataType::Float;
        }
        else
        {
            storageDataType = EStorageDataType::Half;
        }
        ALICEVISION_LOG_DEBUG("writeImage storageDataTypeStr: " << storageDataType);
    }

    if (storageDataType == EStorageDataType::HalfFinite)
    {
        oiio::ImageBufAlgo::clamp(colorspaceBuf, *outBuf, -HALF_MAX, HALF_MAX);
        outBuf = &colorspaceBuf;
    }

    if (storageDataType == EStorageDataType::Half ||
        storageDataType == EStorageDataType::HalfFinite)
    {
        formatBuf.copy(*outBuf, oiio::TypeDesc::HALF); // override format, use half instead of float
        outBuf = &formatBuf;
    }
  }

  // write image
  if(!outBuf->write(tmpPath))
    throw std::runtime_error("Can't write output image file '" + path + "'.");

  // rename temporay filename
  fs::rename(tmpPath, path);
}

template<typename T>
void writeImageNoFloat(const std::string& path,
                oiio::TypeDesc typeDesc,
                const Image<T>& image,
                EImageColorSpace imageColorSpace,
                const oiio::ParamValueList& metadata = oiio::ParamValueList())
{
  const fs::path bPath = fs::path(path);
  const std::string extension = boost::to_lower_copy(bPath.extension().string());
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

  oiio::ImageSpec imageSpec(image.Width(), image.Height(), 1, typeDesc);
  imageSpec.extra_attribs = metadata; // add custom metadata

  imageSpec.attribute("jpeg:subsampling", "4:4:4");           // if possible, always subsampling 4:4:4 for jpeg
  imageSpec.attribute("compression", isEXR ? "zips" : "none"); // if possible, set compression (zips for EXR, none for the other)

  const oiio::ImageBuf imgBuf = oiio::ImageBuf(imageSpec, const_cast<T*>(image.data())); // original image buffer
  const oiio::ImageBuf* outBuf = &imgBuf;  // buffer to write

  oiio::ImageBuf formatBuf;  // buffer for image format modification
  if(isEXR)
  {
    
    formatBuf.copy(*outBuf, typeDesc); // override format, use half instead of float
    outBuf = &formatBuf;
  }

  // write image
  if(!outBuf->write(tmpPath))
    throw std::runtime_error("Can't write output image file '" + path + "'.");

  // rename temporay filename
  fs::rename(tmpPath, path);
}

void readImage(const std::string& path, Image<float>& image, const ImageReadOptions & imageReadOptions)
{
  readImage(path, oiio::TypeDesc::FLOAT, 1, image, imageReadOptions);
}

void readImage(const std::string& path, Image<unsigned char>& image, const ImageReadOptions & imageReadOptions)
{
  readImage(path, oiio::TypeDesc::UINT8, 1, image, imageReadOptions);
}

void readImageDirect(const std::string& path, Image<unsigned char>& image)
{
  readImageNoFloat(path, oiio::TypeDesc::UINT8, image);
}

void readImageDirect(const std::string& path, Image<IndexT>& image)
{
  readImageNoFloat(path, oiio::TypeDesc::UINT32, image);
}

void readImage(const std::string& path, Image<RGBAfColor>& image, const ImageReadOptions & imageReadOptions)
{
  readImage(path, oiio::TypeDesc::FLOAT, 4, image, imageReadOptions);
}

void readImage(const std::string& path, Image<RGBAColor>& image, const ImageReadOptions & imageReadOptions)
{
  readImage(path, oiio::TypeDesc::UINT8, 4, image, imageReadOptions);
}

void readImage(const std::string& path, Image<RGBfColor>& image, const ImageReadOptions & imageReadOptions)
{
  readImage(path, oiio::TypeDesc::FLOAT, 3, image, imageReadOptions);
}

void readImage(const std::string& path, Image<RGBColor>& image, const ImageReadOptions & imageReadOptions)
{
  readImage(path, oiio::TypeDesc::UINT8, 3, image, imageReadOptions);
}

void writeImage(const std::string& path, const Image<unsigned char>& image, EImageColorSpace imageColorSpace,const oiio::ParamValueList& metadata)
{
  writeImageNoFloat(path, oiio::TypeDesc::UINT8, image, imageColorSpace, metadata);
}

void writeImage(const std::string& path, const Image<int>& image, EImageColorSpace imageColorSpace, const oiio::ParamValueList& metadata)
{
  writeImageNoFloat(path, oiio::TypeDesc::INT32, image, imageColorSpace, metadata);
}

void writeImage(const std::string& path, const Image<IndexT>& image, EImageColorSpace imageColorSpace, const oiio::ParamValueList& metadata)
{
  writeImageNoFloat(path, oiio::TypeDesc::UINT32, image, imageColorSpace, metadata);
}

void writeImage(const std::string& path, const Image<RGBAfColor>& image, EImageColorSpace imageColorSpace, const oiio::ParamValueList& metadata, const oiio::ROI& roi)
{
  writeImage(path, oiio::TypeDesc::FLOAT, 4, image, imageColorSpace, metadata,roi);
}

void writeImage(const std::string& path, const Image<RGBAColor>& image, EImageColorSpace imageColorSpace,const oiio::ParamValueList& metadata)
{
  writeImage(path, oiio::TypeDesc::UINT8, 4, image, imageColorSpace, metadata);
}

void writeImage(const std::string& path, const Image<RGBfColor>& image, EImageColorSpace imageColorSpace, const oiio::ParamValueList& metadata, const oiio::ROI &roi)
{
  writeImage(path, oiio::TypeDesc::FLOAT, 3, image, imageColorSpace, metadata,roi);
}

void writeImage(const std::string& path, const Image<float>& image, EImageColorSpace imageColorSpace, const oiio::ParamValueList& metadata, const oiio::ROI& roi)
{
  writeImage(path, oiio::TypeDesc::FLOAT, 1, image, imageColorSpace, metadata,roi);
}

void writeImage(const std::string& path, const Image<RGBColor>& image, EImageColorSpace imageColorSpace,const oiio::ParamValueList& metadata)
{
  writeImage(path, oiio::TypeDesc::UINT8, 3, image, imageColorSpace, metadata);
}

}  // namespace image
}  // namespace aliceVision
