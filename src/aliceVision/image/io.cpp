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
#include <OpenImageIO/color.h>

#include <aliceVision/half.hpp>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <cstring>
#include <stdexcept>
#include <iostream>
#include <cmath>

namespace fs = boost::filesystem;

namespace
{
    std::string getDefaultColorConfigFilePath()
    {
        std::string configOCIOFilePath = "";

        char const* ALICEVISION_OCIO = std::getenv("ALICEVISION_OCIO");
        if (ALICEVISION_OCIO != NULL)
        {
            configOCIOFilePath = std::string(ALICEVISION_OCIO);
            if (fs::exists(configOCIOFilePath))
            {
                // Check if a sRGB linear color space named "scene-linear Rec.709-sRGB" is present and set as scene_linear role 
                oiio::ColorConfig colorConfig(configOCIOFilePath);
                const std::string linearColorSpace = colorConfig.getColorSpaceNameByRole("scene_linear");
                if (linearColorSpace == "scene-linear Rec.709-sRGB")
                {
                    ALICEVISION_LOG_TRACE("ALICEVISION_OCIO configuration file: '" << configOCIOFilePath << "' found.");
                    return configOCIOFilePath;
                }
                else
                {
                    ALICEVISION_LOG_WARNING("ALICEVISION_OCIO configuration file is not valid: '" << configOCIOFilePath << "'.\n"
                                            "The scene_linear role named \"scene-linear Rec.709-sRGB\" is required.\n"
                                            "Skip this OCIO configuration file and use the embedded one.");
                }
            }
            else if (configOCIOFilePath.empty())
            {
                ALICEVISION_LOG_TRACE("ALICEVISION_OCIO is empty.");
            }
            else
            {
                ALICEVISION_LOG_WARNING("ALICEVISION_OCIO is defined but does not point to an existing file: '" << configOCIOFilePath << "'");
            }
        }

        // To be enabled if we decide to take OCIO env var in consideration before using the enbedded config file
        if(false)
        {
            char const* OCIO = std::getenv("OCIO");
            if(OCIO != NULL)
            {
                configOCIOFilePath = std::string(OCIO);
                if(fs::exists(configOCIOFilePath))
                {
                    ALICEVISION_LOG_TRACE("OCIO configuration file: '" << configOCIOFilePath << "' found.");
                    return configOCIOFilePath;
                }
                else if(configOCIOFilePath == "")
                {
                    ALICEVISION_LOG_TRACE("OCIO is empty. Use embedded config...");
                }
                else
                {
                    ALICEVISION_LOG_TRACE("OCIO does not point to an existing file. Use embedded config...");
                }
            }
        }

        char const* ALICEVISION_ROOT = std::getenv("ALICEVISION_ROOT");
        if (ALICEVISION_ROOT == NULL)
        {
            // Output message with logging before throw as this function could be called before main.
            ALICEVISION_LOG_ERROR("ALICEVISION_ROOT is not defined, embedded OCIO config file cannot be accessed.");
            ALICEVISION_THROW_ERROR("ALICEVISION_ROOT is not defined, embedded OCIO config file cannot be accessed.");
        }
        configOCIOFilePath = std::string(ALICEVISION_ROOT);
        configOCIOFilePath.append("/share/aliceVision/config.ocio");

        if (!fs::exists(configOCIOFilePath))
        {
            ALICEVISION_LOG_ERROR("Embedded OCIO configuration file: '" << configOCIOFilePath << "' cannot be accessed.");
            ALICEVISION_THROW_ERROR("Embedded OCIO configuration file: '" << configOCIOFilePath << "' cannot be accessed.");
        }
        else
        {
            ALICEVISION_LOG_TRACE("Embedded OCIO configuration file: '" << configOCIOFilePath << "' found.");
        }

        return configOCIOFilePath;
    }
    oiio::ColorConfig colorConfigOCIO(getDefaultColorConfigFilePath());
}

namespace aliceVision {
namespace image {

void initColorConfigOCIO(const std::string& colorConfigFilePath)
{
    colorConfigOCIO.reset(colorConfigFilePath);
    if (!colorConfigOCIO.supportsOpenColorIO())
    {
        ALICEVISION_THROW_ERROR("OpenImageIO has not been compiled with OCIO.");
    }
    const std::string error = colorConfigOCIO.geterror();
    if (!error.empty())
    {
        ALICEVISION_THROW_ERROR("Erroneous OCIO config file " << colorConfigFilePath << ":" << std::endl << error);
    }
    int ocioVersion = colorConfigOCIO.OpenColorIO_version_hex();
    int ocioMajor = (ocioVersion & 0xFF000000) >> 24;
    int ocioMinor = (ocioVersion & 0x00FF0000) >> 16;
    int ocioPatch = (ocioVersion & 0x0000FF00) >> 8;
    ALICEVISION_LOG_INFO("OCIO color config initialized with OCIO version: " << ocioMajor << "." << ocioMinor << "." << ocioPatch);
}

std::string EImageColorSpace_informations()
{
    return EImageColorSpace_enumToString(EImageColorSpace::AUTO) + ", " +
           EImageColorSpace_enumToString(EImageColorSpace::LINEAR) + ", " +
           EImageColorSpace_enumToString(EImageColorSpace::SRGB) + ", " +
           EImageColorSpace_enumToString(EImageColorSpace::ACES2065_1) + ", " +
           EImageColorSpace_enumToString(EImageColorSpace::ACEScg) + ", " +
           EImageColorSpace_enumToString(EImageColorSpace::LAB) + ", " +
           EImageColorSpace_enumToString(EImageColorSpace::XYZ) + ", " +
           EImageColorSpace_enumToString(EImageColorSpace::NO_CONVERSION);
}

EImageColorSpace EImageColorSpace_stringToEnum(const std::string& dataType)
{
    const std::string type = boost::to_lower_copy(dataType);

    if(type == "auto")
        return EImageColorSpace::AUTO;
    if(type == "linear")
        return EImageColorSpace::LINEAR;
    if(type == "srgb")
        return EImageColorSpace::SRGB;
    if(type == "aces2065-1")
        return EImageColorSpace::ACES2065_1;
    if(type == "acescg")
        return EImageColorSpace::ACEScg;
    if(type == "lab")
        return EImageColorSpace::LAB;
    if(type == "xyz")
        return EImageColorSpace::XYZ;
    if(type == "no_conversion")
        return EImageColorSpace::NO_CONVERSION;

    throw std::out_of_range("Invalid EImageColorSpace: " + dataType);
}

std::string EImageColorSpace_enumToString(const EImageColorSpace dataType)
{
    switch(dataType)
    {
        case EImageColorSpace::AUTO:
            return "auto";
        case EImageColorSpace::LINEAR:
            return "linear";
        case EImageColorSpace::SRGB:
            return "srgb";
        case EImageColorSpace::ACES2065_1:
            return "aces2065-1";
        case EImageColorSpace::ACEScg:
            return "acescg";
        case EImageColorSpace::LAB:
            return "lab";
        case EImageColorSpace::XYZ:
            return "xyz";
        case EImageColorSpace::NO_CONVERSION:
            return "no_conversion";
    }
    throw std::out_of_range("Invalid EImageColorSpace enum");
}

std::string EImageColorSpace_enumToOIIOString(const EImageColorSpace colorSpace)
{
    switch(colorSpace)
    {
        case EImageColorSpace::SRGB: return "sRGB";
        case EImageColorSpace::LINEAR: return "Linear";
        case EImageColorSpace::ACES2065_1: return "aces2065-1";
        case EImageColorSpace::ACEScg: return "ACEScg";
        default: ;
    }
    throw std::out_of_range("No string defined for EImageColorSpace to OIIO conversion: " +
                            std::to_string(int(colorSpace)));
}

EImageColorSpace EImageColorSpace_OIIOstringToEnum(const std::string& colorspace)
{
    if (colorspace == "Linear") return EImageColorSpace::LINEAR;
    if (colorspace == "sRGB") return EImageColorSpace::SRGB;
    if (colorspace == "aces2065-1") return EImageColorSpace::ACES2065_1;
    if (colorspace == "ACEScg") return EImageColorSpace::ACEScg;

    throw std::out_of_range("No EImageColorSpace defined for string: " + colorspace);
}

bool EImageColorSpace_isSupportedOIIOEnum(const EImageColorSpace& colorspace)
{
    switch(colorspace)
    {
        case EImageColorSpace::SRGB: return true;
        case EImageColorSpace::LINEAR: return true;
        case EImageColorSpace::ACES2065_1: return true;
        case EImageColorSpace::ACEScg: return true;
        default: return false;
    }
}

bool EImageColorSpace_isSupportedOIIOstring(const std::string& colorspace)
{
    if (colorspace == "Linear") return true;
    if (colorspace == "sRGB") return true;
    if (colorspace == "aces2065-1") return true;
    if (colorspace == "ACEScg") return true;
    return false;
}

std::ostream& operator<<(std::ostream& os, EImageColorSpace dataType)
{
    return os << EImageColorSpace_enumToString(dataType);
}

std::istream& operator>>(std::istream& in, EImageColorSpace& dataType)
{
    std::string token;
    in >> token;
    dataType = EImageColorSpace_stringToEnum(token);
    return in;
}

EImageColorSpace getImageColorSpace(const std::string imagePath)
{
    oiio::ImageSpec metadataSpec;

    metadataSpec.extra_attribs = readImageMetadata(imagePath);

    std::string colorSpace = metadataSpec.get_string_attribute("AliceVision:ColorSpace", ""); // default image color space is empty
    if (!colorSpace.empty())
    {
        ALICEVISION_LOG_TRACE("Read image " << imagePath << " (encoded in " << colorSpace << " colorspace according to AliceVision:ColorSpace metadata).");
    }
    else
    {
        colorSpace = metadataSpec.get_string_attribute("oiio:ColorSpace", ""); // Check oiio metadata
        if ((colorSpace == "Linear") || (colorSpace == ""))
        {
            std::string colorSpaceFromFileName = colorConfigOCIO.getColorSpaceFromFilepath(imagePath);
            if (!colorSpaceFromFileName.empty())
            {
                ALICEVISION_LOG_TRACE("Read image " << imagePath << " (encoded in " << colorSpaceFromFileName << " colorspace according to file name).");
                colorSpace = colorSpaceFromFileName;
            }
            else if (!colorSpace.empty())
            {
                ALICEVISION_LOG_TRACE("Read image " << imagePath << " (encoded in " << colorSpace << " colorspace according to oiio:ColorSpace metadata).");
            }
            else
            {
                ALICEVISION_LOG_TRACE("Read image " << imagePath << " (no colorspace info, supposed to be encoded in sRGB).");
                colorSpace = "sRGB";
            }
        }
    }

    if (!EImageColorSpace_isSupportedOIIOstring(colorSpace))
    {
        size_t npos = imagePath.find_last_of(".");
        std::string ext = imagePath.substr(npos + 1);
        std::string forcedColorSpace = (ext == "exr" || ext == "EXR") ? "linear" : "sRGB";

        ALICEVISION_LOG_WARNING("The color space " << colorSpace << " detected for " << imagePath << " is not supported. Force Color space to " << forcedColorSpace << ".");
        colorSpace = forcedColorSpace;
    }

    return EImageColorSpace_stringToEnum(colorSpace);
}

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
  const std::string type = boost::to_lower_copy(imageFileType);

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
    case EImageFileType::NONE:  return "none";
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

    if (type == "optimized")
        return EImageQuality::OPTIMIZED;
    if (type == "lossless")
        return EImageQuality::LOSSLESS;

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
    static const std::array<std::string, 6> supportedExtensions = {
        ".jpg", ".jpeg", ".png",  ".tif", ".tiff", ".exr"
    };
    const auto start = supportedExtensions.begin();
    const auto end = supportedExtensions.end();
    return(std::find(start, end, boost::to_lower_copy(ext)) != end);
}

// Warning: type conversion problems from string to param value, we may lose some metadata with string maps
oiio::ParamValueList getMetadataFromMap(const std::map<std::string, std::string>& metadataMap)
{
  oiio::ParamValueList metadata;
  for(const auto& metadataPair : metadataMap)
    metadata.push_back(oiio::ParamValue(metadataPair.first, metadataPair.second));
  return metadata;
}

std::map<std::string, std::string> getMapFromMetadata(const oiio::ParamValueList& metadata)
{
    std::map<std::string, std::string> metadataMap;

    for (const auto& param : metadata)
        metadataMap.emplace(param.name().string(), param.get_string());

    return metadataMap;
}

oiio::ParamValueList readImageMetadata(const std::string& path, int& width, int& height)
{
    const auto spec = readImageSpec(path);
    width = spec.width;
    height = spec.height;
    return spec.extra_attribs;
}

oiio::ImageSpec readImageSpec(const std::string& path)
{
  std::unique_ptr<oiio::ImageInput> in(oiio::ImageInput::open(path));
  oiio::ImageSpec spec = in->spec();

  if(!in)
    throw std::runtime_error("Can't find/open image file '" + path + "'.");

  in->close();

  return spec;
}

oiio::ParamValueList readImageMetadata(const std::string& path)
{
    return readImageSpec(path).extra_attribs;
}

void readImageSize(const std::string& path, int& width, int& height)
{
    const auto spec = readImageSpec(path);
    width = spec.width;
    height = spec.height;
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
    ALICEVISION_LOG_DEBUG("[IO] Read Image: " << path);

    // check requested channels number
    if (nchannels == 0)
        throw std::runtime_error("Requested channels is 0. Image file: '" + path + "'.");
    if (nchannels == 2)
        throw std::runtime_error("Load of 2 channels is not supported. Image file: '" + path + "'.");

  if(!fs::exists(path))
    ALICEVISION_THROW_ERROR("No such image file: '" << path << "'.");

  oiio::ImageSpec configSpec;

  // libRAW configuration
  // See https://openimageio.readthedocs.io/en/master/builtinplugins.html#raw-digital-camera-files
  configSpec.attribute("raw:auto_bright", 0); // disable exposure correction
  configSpec.attribute("raw:use_camera_wb", (imageReadOptions.applyWhiteBalance?1:0)); // white balance correction
  // use_camera_matrix: Whether to use the embedded color profile, if it is present: 0=never, 1 (default)=only for DNG files, 3=always
  configSpec.attribute("raw:use_camera_matrix", 3); // use embeded color profile
  configSpec.attribute("raw:ColorSpace", "Linear"); // use linear colorspace with sRGB primaries

  oiio::ImageBuf inBuf(path, 0, 0, NULL, &configSpec);

  inBuf.read(0, 0, true, oiio::TypeDesc::FLOAT); // force image convertion to float (for grayscale and color space convertion)

  if(!inBuf.initialized())
    ALICEVISION_THROW_ERROR("Failed to open the image file: '" << path << "'.");

    // check picture channels number
    if (inBuf.spec().nchannels == 0)
        throw std::runtime_error("No channel in the input image file: '" + path + "'.");
    if (inBuf.spec().nchannels == 2)
        throw std::runtime_error("Load of 2 channels is not supported. Image file: '" + path + "'.");

  // color conversion
  if(imageReadOptions.workingColorSpace == EImageColorSpace::AUTO)
    throw std::runtime_error("You must specify a requested color space for image file '" + path + "'.");

  // Get color space name. Default image color space is sRGB
  const std::string& fromColorSpaceName = inBuf.spec().get_string_attribute("oiio:ColorSpace", "sRGB");

  ALICEVISION_LOG_TRACE("Read image " << path << " (encoded in " << fromColorSpaceName << " colorspace).");

  if (imageReadOptions.workingColorSpace != image::EImageColorSpace::NO_CONVERSION)
  {
      imageAlgo::colorconvert(inBuf, fromColorSpaceName, imageReadOptions.workingColorSpace);
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
    const float weights[3] = {.2126f, .7152f, .0722f}; // To be changed if not sRGB Rec 709 Linear.
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
    auto stats = oiio::ImageBufAlgo::computePixelStats(image);

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
                OutputFileColorSpace colorspace,
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

    if (colorspace.to == EImageColorSpace::AUTO)
    {
        if (isJPG || isPNG)
            colorspace.to = EImageColorSpace::SRGB;
        else
            colorspace.to = EImageColorSpace::LINEAR;
    }


    ALICEVISION_LOG_DEBUG("[IO] Write Image: " << path << "\n"
                       << "\t- width: " << image.Width() << "\n"
                       << "\t- height: " << image.Height() << "\n"
                       << "\t- channels: " << nchannels);

  oiio::ImageSpec imageSpec(image.Width(), image.Height(), nchannels, typeDesc);
  imageSpec.extra_attribs = metadata; // add custom metadata

  imageSpec.attribute("jpeg:subsampling", "4:4:4");           // if possible, always subsampling 4:4:4 for jpeg
  imageSpec.attribute("compression", isEXR ? "zips" : "none"); // if possible, set compression (zips for EXR, none for the other)
  if(roi.defined() && isEXR)
  {
      imageSpec.set_roi_full(roi);
  }
  std::string currentColorSpace = imageSpec.get_string_attribute("AliceVision:ColorSpace", "Linear");

  imageSpec.attribute("AliceVision:ColorSpace", (colorspace.to == EImageColorSpace::NO_CONVERSION) ? currentColorSpace : EImageColorSpace_enumToString(colorspace.to));
  
  const oiio::ImageBuf imgBuf = oiio::ImageBuf(imageSpec, const_cast<T*>(image.data())); // original image buffer
  const oiio::ImageBuf* outBuf = &imgBuf;  // buffer to write

  oiio::ImageBuf colorspaceBuf; // buffer for image colorspace modification
    if (colorspace.from == colorspace.to)
    {
        // Do nothing. Note that calling imageAlgo::colorconvert() will copy the source buffer
        // even if no conversion is needed.
    }
    else if((colorspace.to == EImageColorSpace::ACES2065_1) || (colorspace.to == EImageColorSpace::ACEScg))
    {
        const auto colorConfigPath = getAliceVisionOCIOConfig();
        if (colorConfigPath.empty())
        {
            throw std::runtime_error("ALICEVISION_ROOT is not defined, OCIO config file cannot be accessed.");
        }

        oiio::ColorConfig colorConfig(colorConfigPath);
        oiio::ImageBufAlgo::colorconvert(colorspaceBuf, *outBuf,
                                         EImageColorSpace_enumToOIIOString(colorspace.from),
                                         EImageColorSpace_enumToOIIOString(colorspace.to), true, "", "",
                                         &colorConfig);
        outBuf = &colorspaceBuf;
    }
    else
    {
        imageAlgo::colorconvert(colorspaceBuf, *outBuf, colorspace.from, colorspace.to);
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
      ALICEVISION_THROW_ERROR("Can't write output image file '" + path + "'.");

  // rename temporary filename
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

  // rename temporary filename
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

void writeImage(const std::string& path, const Image<unsigned char>& image, EImageColorSpace outputImageColorSpace,const oiio::ParamValueList& metadata)
{
  writeImageNoFloat(path, oiio::TypeDesc::UINT8, image, outputImageColorSpace, metadata);
}

void writeImage(const std::string& path, const Image<int>& image, EImageColorSpace outputImageColorSpace, const oiio::ParamValueList& metadata)
{
  writeImageNoFloat(path, oiio::TypeDesc::INT32, image, outputImageColorSpace, metadata);
}

void writeImage(const std::string& path, const Image<IndexT>& image, EImageColorSpace outputImageColorSpace, const oiio::ParamValueList& metadata)
{
  writeImageNoFloat(path, oiio::TypeDesc::UINT32, image, outputImageColorSpace, metadata);
}

void writeImage(const std::string& path, const Image<RGBAfColor>& image, EImageColorSpace outputImageColorSpace, const oiio::ParamValueList& metadata, const oiio::ROI& roi)
{
    writeImage(path, oiio::TypeDesc::FLOAT, 4, image,
               OutputFileColorSpace{EImageColorSpace::LINEAR, outputImageColorSpace }, metadata,roi);
}

void writeImage(const std::string& path, const Image<RGBAColor>& image, EImageColorSpace outputImageColorSpace,const oiio::ParamValueList& metadata)
{
    writeImage(path, oiio::TypeDesc::UINT8, 4, image,
               OutputFileColorSpace{EImageColorSpace::LINEAR, outputImageColorSpace }, metadata);
}

void writeImage(const std::string& path, const Image<RGBfColor>& image, EImageColorSpace outputImageColorSpace, const oiio::ParamValueList& metadata, const oiio::ROI &roi)
{
    writeImage(path, oiio::TypeDesc::FLOAT, 3, image,
               OutputFileColorSpace{EImageColorSpace::LINEAR, outputImageColorSpace }, metadata,roi);
}

void writeImage(const std::string& path, const Image<float>& image, EImageColorSpace outputImageColorSpace, const oiio::ParamValueList& metadata, const oiio::ROI& roi)
{
    writeImage(path, oiio::TypeDesc::FLOAT, 1, image,
               OutputFileColorSpace{EImageColorSpace::LINEAR, outputImageColorSpace }, metadata,roi);
}

void writeImage(const std::string& path, const Image<RGBColor>& image, EImageColorSpace outputImageColorSpace,const oiio::ParamValueList& metadata)
{
    writeImage(path, oiio::TypeDesc::UINT8, 3, image,
               OutputFileColorSpace{EImageColorSpace::LINEAR, outputImageColorSpace }, metadata);
}

void writeImage(const std::string& path, const Image<float>& image, OutputFileColorSpace colorspace,
                const oiio::ParamValueList& metadata, const oiio::ROI& roi)
{
    writeImage(path, oiio::TypeDesc::FLOAT, 1, image, colorspace, metadata,roi);
}

void writeImage(const std::string& path, const Image<RGBAfColor>& image,
                OutputFileColorSpace colorspace, const oiio::ParamValueList& metadata,
                const oiio::ROI& roi)
{
    writeImage(path, oiio::TypeDesc::FLOAT, 4, image, colorspace, metadata,roi);
}

void writeImage(const std::string& path, const Image<RGBAColor>& image,
                OutputFileColorSpace colorspace,
                const oiio::ParamValueList& metadata)
{
    writeImage(path, oiio::TypeDesc::UINT8, 4, image, colorspace, metadata);
}

void writeImage(const std::string& path, const Image<RGBfColor>& image,
                OutputFileColorSpace colorspace, const oiio::ParamValueList& metadata,
                const oiio::ROI& roi)
{
    writeImage(path, oiio::TypeDesc::FLOAT, 3, image, colorspace, metadata, roi);
}

void writeImage(const std::string& path, const Image<RGBColor>& image,
                OutputFileColorSpace colorspace,
                const oiio::ParamValueList& metadata)
{
    writeImage(path, oiio::TypeDesc::UINT8, 3, image, colorspace, metadata);
}

bool tryLoadMask(Image<unsigned char>* mask, const std::vector<std::string>& masksFolders,
                 const IndexT viewId, const std::string & srcImage)
{
    for (const auto & masksFolder_str : masksFolders)
    {
        if (!masksFolder_str.empty() && fs::exists(masksFolder_str))
        {
            const auto masksFolder = fs::path(masksFolder_str);
            const auto idMaskPath = masksFolder / fs::path(std::to_string(viewId)).replace_extension("png");
            const auto nameMaskPath = masksFolder / fs::path(srcImage).filename().replace_extension("png");

            if (fs::exists(idMaskPath))
            {
                readImage(idMaskPath.string(), *mask, EImageColorSpace::LINEAR);
                return true;
            }
            else if (fs::exists(nameMaskPath))
            {
                readImage(nameMaskPath.string(), *mask, EImageColorSpace::LINEAR);
                return true;
            }
        }
    }
    return false;
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

void readImage(const std::string& path, int& width, int& height, std::vector<RGBfColor>& buffer,
               image::EImageColorSpace toColorSpace)
{
    readImage(path, oiio::TypeDesc::FLOAT, 3, width, height, buffer, toColorSpace);
}

void readImage(const std::string& path, int& width, int& height, std::vector<RGBAfColor>& buffer,
               image::EImageColorSpace toColorSpace)
{
    readImage(path, oiio::TypeDesc::FLOAT, 4, width, height, buffer, toColorSpace);
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
        if (isJPG || isPNG)
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
    if (imageQuality == EImageQuality::OPTIMIZED && isEXR)
    {
        formatBuf.copy(*outBuf, oiio::TypeDesc::HALF); // override format, use half instead of float
        outBuf = &formatBuf;
    }

    // write image
    if (!outBuf->write(tmpPath))
        throw std::runtime_error("Can't write output image file '" + path + "'.");

    // rename temporary filename
    fs::rename(tmpPath, path);
}

void writeImage(const std::string& path, int width, int height, const std::vector<unsigned char>& buffer,
                EImageQuality imageQuality, const OutputFileColorSpace& colorspace,
                const oiio::ParamValueList& metadata)
{
    writeImage(path, oiio::TypeDesc::UCHAR, width, height, 1, buffer, imageQuality, colorspace, metadata);
}

void writeImage(const std::string& path, int width, int height, const std::vector<unsigned short>& buffer,
                EImageQuality imageQuality, const OutputFileColorSpace& colorspace,
                const oiio::ParamValueList& metadata)
{
    writeImage(path, oiio::TypeDesc::UINT16, width, height, 1, buffer, imageQuality, colorspace, metadata);
}

void writeImage(const std::string& path, int width, int height, const std::vector<rgb>& buffer,
                EImageQuality imageQuality, const OutputFileColorSpace& colorspace,
                const oiio::ParamValueList& metadata)
{
    writeImage(path, oiio::TypeDesc::UCHAR, width, height, 3, buffer, imageQuality, colorspace, metadata);
}

void writeImage(const std::string& path, int width, int height, const std::vector<float>& buffer,
                EImageQuality imageQuality, const OutputFileColorSpace& colorspace,
                const oiio::ParamValueList& metadata)
{
    writeImage(path, oiio::TypeDesc::FLOAT, width, height, 1, buffer, imageQuality, colorspace, metadata);
}

void writeImage(const std::string& path, int width, int height, const std::vector<RGBfColor>& buffer,
                EImageQuality imageQuality, const OutputFileColorSpace& colorspace,
                const oiio::ParamValueList& metadata)
{
    writeImage(path, oiio::TypeDesc::FLOAT, width, height, 3, buffer, imageQuality, colorspace, metadata);
}

static std::string aliceVisionRootOverride;

std::string getAliceVisionRoot()
{
    if (!aliceVisionRootOverride.empty())
        return aliceVisionRootOverride;
    const char* value = std::getenv("ALICEVISION_ROOT");
    return value ? value : "";
}

std::string getAliceVisionOCIOConfig()
{
    if (!getAliceVisionRoot().empty())
        return getAliceVisionRoot() + "/share/aliceVision/config.ocio";
    return {};
}

void setAliceVisionRootOverride(const std::string& value)
{
    aliceVisionRootOverride = value;
}

}  // namespace image
}  // namespace aliceVision
