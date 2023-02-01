// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/image/all.hpp>

#include <aliceVision/system/Logger.hpp>

#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>
#include <OpenImageIO/color.h>

#include <aliceVision/half.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <cstring>
#include <stdexcept>
#include <iostream>
#include <cmath>

namespace fs = boost::filesystem;

namespace aliceVision {
namespace image {


EImageColorSpace getImageColorSpace(const std::string& imagePath)
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
            const std::string colorSpaceFromFileName = getGlobalColorConfigOCIO().getColorSpaceFromFilepath(imagePath);
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
        const size_t npos = imagePath.find_last_of(".");
        const std::string ext = imagePath.substr(npos + 1);
        const std::string forcedColorSpace = (ext == "exr" || ext == "EXR") ? "linear" : "sRGB";

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
        EStorageDataType_enumToString(EStorageDataType::Auto) + ", " +
        EStorageDataType_enumToString(EStorageDataType::Undefined);
}

EStorageDataType EStorageDataType_stringToEnum(const std::string& dataType)
{
    std::string type = dataType;
    std::transform(type.begin(), type.end(), type.begin(), ::tolower); //tolower

    if (type == "float") return EStorageDataType::Float;
    if (type == "half") return EStorageDataType::Half;
    if (type == "halffinite") return EStorageDataType::HalfFinite;
    if (type == "auto") return EStorageDataType::Auto;
    if (type == "undefined") return EStorageDataType::Undefined;

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
    case EStorageDataType::Undefined: return "undefined";
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

std::string ERawColorInterpretation_informations()
{
    return "Raw color interpretation :\n"
           "* none : None \n"
           "* librawnowhitebalancing : libRaw whithout white balancing \n"
           "* librawwhitebalancing : libRaw whith white balancing \n"
           "* dcplinearprocessing : DCP linear processing \n"
           "* dcpmetadata : None but DCP info embedded in metadata";
}

ERawColorInterpretation ERawColorInterpretation_stringToEnum(const std::string& rawColorInterpretation)
{
    std::string type = rawColorInterpretation;
    std::transform(type.begin(), type.end(), type.begin(), ::tolower); //tolower

    if (type == "none")
        return ERawColorInterpretation::None;
    if (type == "librawnowhitebalancing")
        return ERawColorInterpretation::LibRawNoWhiteBalancing;
    if (type == "librawwhitebalancing")
        return ERawColorInterpretation::LibRawWhiteBalancing;
    if (type == "dcplinearprocessing")
        return ERawColorInterpretation::DcpLinearProcessing;
    if (type == "dcpmetadata")
        return ERawColorInterpretation::DcpMetadata;
    if (type == "auto")
        return ERawColorInterpretation::Auto;

    throw std::out_of_range("Invalid raw color interpretation : " + rawColorInterpretation);

}

std::string ERawColorInterpretation_enumToString(const ERawColorInterpretation rawColorInterpretation)
{
    switch (rawColorInterpretation)
    {
        case ERawColorInterpretation::None: return "none";
        case ERawColorInterpretation::LibRawNoWhiteBalancing: return "librawnowhitebalancing";
        case ERawColorInterpretation::LibRawWhiteBalancing: return "librawwhitebalancing";
        case ERawColorInterpretation::DcpLinearProcessing: return "dcpLinearprocessing";
        case ERawColorInterpretation::DcpMetadata: return "dcpmetadata";
        case ERawColorInterpretation::Auto: return "auto";
    }
    throw std::out_of_range("Invalid ERawColorInterpretation enum");
}

std::ostream& operator<<(std::ostream& os, ERawColorInterpretation rawColorInterpretation)
{
    return os << ERawColorInterpretation_enumToString(rawColorInterpretation);
}

std::istream& operator>>(std::istream& in, ERawColorInterpretation& rawColorInterpretation)
{
    std::string token;
    in >> token;
    rawColorInterpretation = ERawColorInterpretation_stringToEnum(token);
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

bool isRawFormat(const std::string& path)
{
    std::unique_ptr<oiio::ImageInput> in(oiio::ImageInput::open(path));
    if(!in)
        return false;
    std::string imgFormat = in->format_name();

    return (imgFormat.compare("raw") == 0);
}


template<typename T>
void readImage(const std::string& path,
               oiio::TypeDesc format,
               int nchannels,
               Image<T>& image,
               const ImageReadOptions& imageReadOptions)
{
    ALICEVISION_LOG_DEBUG("[IO] Read Image: " << path);

    // check requested channels number
    if (nchannels == 0)
        ALICEVISION_THROW_ERROR("Requested channels is 0. Image file: '" + path + "'.");
    if (nchannels == 2)
        ALICEVISION_THROW_ERROR("Load of 2 channels is not supported. Image file: '" + path + "'.");

    if(!fs::exists(path))
        ALICEVISION_THROW_ERROR("No such image file: '" << path << "'.");

    oiio::ImageSpec configSpec;

    const bool isRawImage = isRawFormat(path);
    image::DCPProfile::Triple neutral = {1.0,1.0,1.0};

    if (isRawImage)
    {
        if (imageReadOptions.rawColorInterpretation == ERawColorInterpretation::DcpLinearProcessing)
        {
            oiio::ParamValueList imgMetadata = readImageMetadata(path);
            std::string cam_mul = "";
            if (!imgMetadata.getattribute("raw:cam_mul", cam_mul))
            {
                cam_mul = "{1024, 1024, 1024, 1024}";
                ALICEVISION_LOG_WARNING("[readImage]: cam_mul metadata not available, the openImageIO version might be too old (>= 2.4.5.0 requested for dcp management).");
            }

            std::vector<float> v_mult;
            size_t last = 1;
            size_t next = 1;
            while ((next = cam_mul.find(",", last)) != std::string::npos)
            {
                v_mult.push_back(std::stof(cam_mul.substr(last, next - last).c_str()));
                last = next + 1;
            }
            v_mult.push_back(std::stof(cam_mul.substr(last, cam_mul.find("}", last) - last).c_str()));

            for (int i = 0; i < 3; i++)
            {
                //neutral[i] = v_mult[1] / v_mult[i];
                neutral[i] = v_mult[i] / v_mult[1];
            }
        }

        ALICEVISION_LOG_INFO("Neutral from camera = {" << neutral[0] << ", " << neutral[1] << ", " << neutral[2] << "}");

        // libRAW configuration
        // See https://openimageio.readthedocs.io/en/master/builtinplugins.html#raw-digital-camera-files

        if (imageReadOptions.rawColorInterpretation == ERawColorInterpretation::None)
        {
            if (imageReadOptions.workingColorSpace != EImageColorSpace::NO_CONVERSION)
            {
                ALICEVISION_THROW_ERROR("Working color space must be set to \"no_conversion\" if raw color interpretation is set to \"none\"");
            }

            float user_mul[4] = {1,1,1,1};

            configSpec.attribute("raw:auto_bright", 0); // disable exposure correction
            configSpec.attribute("raw:use_camera_wb", 0); // no white balance correction
            configSpec.attribute("raw:user_mul", oiio::TypeDesc(oiio::TypeDesc::FLOAT, 4), user_mul); // no neutralization
            configSpec.attribute("raw:use_camera_matrix", 0); // do not use embeded color profile if any
            configSpec.attribute("raw:ColorSpace", "raw"); // use raw data
            configSpec.attribute("raw:HighlightMode", 1); // unclip
            configSpec.attribute("raw:Demosaic", "DHT"); // DHT algorithm
        }
        else if (imageReadOptions.rawColorInterpretation == ERawColorInterpretation::LibRawNoWhiteBalancing)
        {
            configSpec.attribute("raw:auto_bright", 0); // disable exposure correction
            configSpec.attribute("raw:use_camera_wb", 0); // white balance correction
            configSpec.attribute("raw:use_camera_matrix", 0); // do not use embeded color profile if any
            configSpec.attribute("raw:ColorSpace", "Linear"); // use linear colorspace with sRGB primaries
        }
        else if (imageReadOptions.rawColorInterpretation == ERawColorInterpretation::LibRawWhiteBalancing)
        {
            configSpec.attribute("raw:auto_bright", 0); // disable exposure correction
            configSpec.attribute("raw:use_camera_wb", 1); // white balance correction
            configSpec.attribute("raw:use_camera_matrix", 0); // do not use embeded color profile if any
            configSpec.attribute("raw:ColorSpace", "Linear"); // use linear colorspace with sRGB primaries
        }
        else if (imageReadOptions.rawColorInterpretation == ERawColorInterpretation::DcpLinearProcessing)
        {
            if (imageReadOptions.colorProfileFileName.empty())
            {
                ALICEVISION_THROW_ERROR("A DCP color profile is required but cannot be found");
            }
            ALICEVISION_LOG_INFO("Raw demosaicing algo: " << imageReadOptions.demosaicingAlgo);
            ALICEVISION_LOG_INFO("WB after demosaicing: " << (imageReadOptions.doWBAfterDemosaicing ? "True" : "False"));
            ALICEVISION_LOG_INFO("Highlight mode: " << (imageReadOptions.highlightMode==0 ? "Clip" : (imageReadOptions.highlightMode == 1 ? "Unclip" : "Blend")));

            float user_mul[4] = { neutral[0],neutral[1],neutral[2],neutral[1] };
            if (imageReadOptions.doWBAfterDemosaicing)
            {
                for (int i = 0; i < 4; ++i)
                {
                    user_mul[i] = 1.0;
                }
            }
            configSpec.attribute("raw:auto_bright", 0); // disable exposure correction
            configSpec.attribute("raw:use_camera_wb", 0); // No White balance correction => user_mul is used
            configSpec.attribute("raw:user_mul", oiio::TypeDesc(oiio::TypeDesc::FLOAT, 4), user_mul);
            configSpec.attribute("raw:use_camera_matrix", 0); // do not use embeded color profile if any
            configSpec.attribute("raw:ColorSpace", "raw");
            configSpec.attribute("raw:HighlightMode", imageReadOptions.highlightMode);
            configSpec.attribute("raw:Demosaic", imageReadOptions.demosaicingAlgo);
        }
        else if (imageReadOptions.rawColorInterpretation == ERawColorInterpretation::DcpMetadata)
        {
            if (imageReadOptions.colorProfileFileName.empty())
            {
                ALICEVISION_THROW_ERROR("A DCP color profile is required but cannot be found");
            }
            ALICEVISION_LOG_INFO("Raw demosaicing algo: " << imageReadOptions.demosaicingAlgo);
            ALICEVISION_LOG_INFO("WB after demosaicing: " << (imageReadOptions.doWBAfterDemosaicing ? "True" : "False"));
            ALICEVISION_LOG_INFO("Highlight mode: " << (imageReadOptions.highlightMode == 0 ? "Clip" : (imageReadOptions.highlightMode == 1 ? "Unclip" : "Blend")));

            float user_mul[4] = { neutral[0],neutral[1],neutral[2],neutral[1] };
            if (imageReadOptions.doWBAfterDemosaicing)
            {
                for (int i = 0; i < 4; ++i)
                {
                    user_mul[i] = 1.0;
                }
            }
            configSpec.attribute("raw:auto_bright", 0); // disable exposure correction
            configSpec.attribute("raw:use_camera_wb", 0); // no white balance correction
            configSpec.attribute("raw:user_mul", oiio::TypeDesc(oiio::TypeDesc::FLOAT, 4), user_mul); // no neutralization
            configSpec.attribute("raw:use_camera_matrix", 0); // do not use embeded color profile if any
            configSpec.attribute("raw:ColorSpace", "raw"); // use raw data
            configSpec.attribute("raw:HighlightMode", imageReadOptions.highlightMode);
            configSpec.attribute("raw:Demosaic", imageReadOptions.demosaicingAlgo);
        }
        else
        {
            ALICEVISION_THROW_ERROR("[image] readImage: invalid rawColorInterpretation " << ERawColorInterpretation_enumToString(imageReadOptions.rawColorInterpretation) << ".");
        }
    }

    oiio::ImageBuf inBuf(path, 0, 0, NULL, &configSpec);

    inBuf.read(0, 0, true, oiio::TypeDesc::FLOAT); // force image convertion to float (for grayscale and color space convertion)

    if(!inBuf.initialized())
        ALICEVISION_THROW_ERROR("Failed to open the image file: '" << path << "'.");

    // check picture channels number
    if (inBuf.spec().nchannels == 0)
        ALICEVISION_THROW_ERROR("No channel in the input image file: '" + path + "'.");
    if (inBuf.spec().nchannels == 2)
        ALICEVISION_THROW_ERROR("Load of 2 channels is not supported. Image file: '" + path + "'.");

    // Apply DCP profile
    if (!imageReadOptions.colorProfileFileName.empty() &&
        imageReadOptions.rawColorInterpretation == ERawColorInterpretation::DcpLinearProcessing)
    {
        image::DCPProfile dcpProfile(imageReadOptions.colorProfileFileName);


        oiio::ParamValueList imgMetadata = readImageMetadata(path);
        std::string cam_mul = "";
        if (!imgMetadata.getattribute("raw:cam_mul", cam_mul))
        {
            cam_mul = "{1024, 1024, 1024, 1024}";
            ALICEVISION_LOG_WARNING("[readImage]: cam_mul metadata not availbale, the openImageIO version might be too old (>= 2.4.5.0 requested for dcp management).");
        }

        std::vector<float> v_mult;
        size_t last = 1;
        size_t next = 1;
        while ((next = cam_mul.find(",", last)) != std::string::npos)
        {
            v_mult.push_back(std::stof(cam_mul.substr(last, next - last)));
            last = next + 1;
        }
        v_mult.push_back(std::stof(cam_mul.substr(last, cam_mul.find("}", last) - last)));

        image::DCPProfile::Triple neutral;
        for (int i = 0; i < 3; i++)
        {
            neutral[i] = v_mult[1] / v_mult[i];
        }

        ALICEVISION_LOG_TRACE("Apply DCP Linear processing with neutral = {" << neutral[0] << ", " << neutral[1] << ", " << neutral[2] << "}");

        dcpProfile.applyLinear(inBuf, neutral, imageReadOptions.doWBAfterDemosaicing, imageReadOptions.useDCPColorMatrixOnly);
    }

    // color conversion
    if(imageReadOptions.workingColorSpace == EImageColorSpace::AUTO)
        ALICEVISION_THROW_ERROR("You must specify a requested color space for image file '" + path + "'.");

    // Get color space name. Default image color space is sRGB
    const std::string fromColorSpaceName = (isRawImage && imageReadOptions.rawColorInterpretation == ERawColorInterpretation::DcpLinearProcessing) ? "aces2065-1" :
                                            (isRawImage ? "linear" :
                                             inBuf.spec().get_string_attribute("aliceVision:ColorSpace", inBuf.spec().get_string_attribute("oiio:ColorSpace", "sRGB")));

    ALICEVISION_LOG_TRACE("Read image " << path << " (encoded in " << fromColorSpaceName << " colorspace).");
  
    if ((imageReadOptions.workingColorSpace == EImageColorSpace::NO_CONVERSION) ||
        (imageReadOptions.workingColorSpace == EImageColorSpace_stringToEnum(fromColorSpaceName)))
    {
        // Do nothing. Note that calling imageAlgo::colorconvert() will copy the source buffer
        // even if no conversion is needed.
    }
    else if ((imageReadOptions.workingColorSpace == EImageColorSpace::ACES2065_1) || (imageReadOptions.workingColorSpace == EImageColorSpace::ACEScg) ||
             (EImageColorSpace_stringToEnum(fromColorSpaceName) == EImageColorSpace::ACES2065_1) || (EImageColorSpace_stringToEnum(fromColorSpaceName) == EImageColorSpace::ACEScg))
    {
        const auto colorConfigPath = getAliceVisionOCIOConfig();
        if (colorConfigPath.empty())
        {
            throw std::runtime_error("ALICEVISION_ROOT is not defined, OCIO config file cannot be accessed.");
        }
        oiio::ImageBuf colorspaceBuf;
        oiio::ColorConfig colorConfig(colorConfigPath);
        oiio::ImageBufAlgo::colorconvert(colorspaceBuf, inBuf,
            fromColorSpaceName,
            EImageColorSpace_enumToOIIOString(imageReadOptions.workingColorSpace), true, "", "",
            &colorConfig);
        inBuf = colorspaceBuf;
    }
    else
    {
        oiio::ImageBuf colorspaceBuf;
        oiio::ImageBufAlgo::colorconvert(colorspaceBuf, inBuf, fromColorSpaceName, EImageColorSpace_enumToOIIOString(imageReadOptions.workingColorSpace));
        inBuf = colorspaceBuf;
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
                const ImageWriteOptions& options,
                const oiio::ParamValueList& metadata = oiio::ParamValueList(),
                const oiio::ROI& displayRoi = oiio::ROI(), 
                const oiio::ROI& pixelRoi = oiio::ROI())
{
    const fs::path bPath = fs::path(path);
    const std::string extension = boost::to_lower_copy(bPath.extension().string());
    const std::string tmpPath =  (bPath.parent_path() / bPath.stem()).string() + "." + fs::unique_path().string() + extension;
    const bool isEXR = (extension == ".exr");
    //const bool isTIF = (extension == ".tif");
    const bool isJPG = (extension == ".jpg");
    const bool isPNG = (extension == ".png");

    auto toColorSpace = options.getToColorSpace();
    auto fromColorSpace = options.getFromColorSpace();

    if (toColorSpace == EImageColorSpace::AUTO)
    {
        if (isJPG || isPNG)
            toColorSpace = EImageColorSpace::SRGB;
        else
            toColorSpace = EImageColorSpace::LINEAR;
    }

    ALICEVISION_LOG_DEBUG("[IO] Write Image: " << path << "\n"
                        << "\t- width: " << image.Width() << "\n"
                        << "\t- height: " << image.Height() << "\n"
                        << "\t- channels: " << nchannels);

    oiio::ImageSpec imageSpec(image.Width(), image.Height(), nchannels, typeDesc);
    imageSpec.extra_attribs = metadata; // add custom metadata

  imageSpec.attribute("jpeg:subsampling", "4:4:4");           // if possible, always subsampling 4:4:4 for jpeg
  imageSpec.attribute("compression", isEXR ? "zips" : "none"); // if possible, set compression (zips for EXR, none for the other)

  if(displayRoi.defined() && isEXR)
  {
      imageSpec.set_roi_full(displayRoi);
  }

  if(pixelRoi.defined() && isEXR)
  {
      imageSpec.set_roi(pixelRoi);
  }


    imageSpec.attribute("AliceVision:ColorSpace",
                        (toColorSpace == EImageColorSpace::NO_CONVERSION)
                            ? EImageColorSpace_enumToString(fromColorSpace) : EImageColorSpace_enumToString(toColorSpace));
  
    const oiio::ImageBuf imgBuf = oiio::ImageBuf(imageSpec, const_cast<T*>(image.data())); // original image buffer
    const oiio::ImageBuf* outBuf = &imgBuf;  // buffer to write
        
    oiio::ImageBuf colorspaceBuf = oiio::ImageBuf(imageSpec, const_cast<T*>(image.data())); // buffer for image colorspace modification
    if ((fromColorSpace == toColorSpace) || (toColorSpace == EImageColorSpace::NO_CONVERSION))
    {
        // Do nothing. Note that calling imageAlgo::colorconvert() will copy the source buffer
        // even if no conversion is needed.
    }
    else if ((toColorSpace == EImageColorSpace::ACES2065_1) || (toColorSpace == EImageColorSpace::ACEScg) ||
             (fromColorSpace == EImageColorSpace::ACES2065_1) || (fromColorSpace == EImageColorSpace::ACEScg))
    {
        const auto colorConfigPath = getAliceVisionOCIOConfig();
        if (colorConfigPath.empty())
        {
            throw std::runtime_error("ALICEVISION_ROOT is not defined, OCIO config file cannot be accessed.");
        }
        oiio::ColorConfig colorConfig(colorConfigPath);
        oiio::ImageBufAlgo::colorconvert(colorspaceBuf, *outBuf,
                                         EImageColorSpace_enumToOIIOString(fromColorSpace),
                                         EImageColorSpace_enumToOIIOString(toColorSpace), true, "", "",
                                         &colorConfig);
        outBuf = &colorspaceBuf;
    }
    else
    {
        oiio::ImageBufAlgo::colorconvert(colorspaceBuf, *outBuf, EImageColorSpace_enumToOIIOString(fromColorSpace), EImageColorSpace_enumToOIIOString(toColorSpace));
        outBuf = &colorspaceBuf;
    }

    oiio::ImageBuf formatBuf;  // buffer for image format modification
    if(isEXR)
    {
        // Storage data type may be saved as attributes to formats that support it and then come back
        // as metadata to this function. Therefore we store the storage data type to attributes if it
        // is set and load it from attributes if it isn't set.
        if (options.getStorageDataType() != EStorageDataType::Undefined)
        {
            imageSpec.attribute("AliceVision:storageDataType",
                                EStorageDataType_enumToString(options.getStorageDataType()));
        }

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
                const ImageWriteOptions& options,
                const oiio::ParamValueList& metadata = oiio::ParamValueList())
{
  const fs::path bPath = fs::path(path);
  const std::string extension = boost::to_lower_copy(bPath.extension().string());
  const std::string tmpPath =  (bPath.parent_path() / bPath.stem()).string() + "." + fs::unique_path().string() + extension;
  const bool isEXR = (extension == ".exr");
  //const bool isTIF = (extension == ".tif");
  const bool isJPG = (extension == ".jpg");
  const bool isPNG = (extension == ".png");

  auto imageColorSpace = options.getToColorSpace();
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

void writeImage(const std::string& path, const Image<unsigned char>& image,
                const ImageWriteOptions& options, const oiio::ParamValueList& metadata)
{
    writeImageNoFloat(path, oiio::TypeDesc::UINT8, image, options, metadata);
}

void writeImage(const std::string& path, const Image<int>& image,
                const ImageWriteOptions& options, const oiio::ParamValueList& metadata)
{
    writeImageNoFloat(path, oiio::TypeDesc::INT32, image, options, metadata);
}

void writeImage(const std::string& path, const Image<IndexT>& image,
                const ImageWriteOptions& options, const oiio::ParamValueList& metadata)
{
    writeImageNoFloat(path, oiio::TypeDesc::UINT32, image, options, metadata);
}

void writeImage(const std::string& path, 
                const Image<float>& image,
                const ImageWriteOptions& options,
                const oiio::ParamValueList& metadata,
                const oiio::ROI& displayRoi, 
                const oiio::ROI& pixelRoi)
{
    writeImage(path, oiio::TypeDesc::FLOAT, 1, image, options, metadata, displayRoi, pixelRoi);
}

void writeImage(const std::string& path, 
                const Image<RGBAfColor>& image,
                const ImageWriteOptions& options, 
                const oiio::ParamValueList& metadata,
                const oiio::ROI& displayRoi, 
                const oiio::ROI& pixelRoi)
{
    writeImage(path, oiio::TypeDesc::FLOAT, 4, image, options, metadata, displayRoi, pixelRoi);
}

void writeImage(const std::string& path, const Image<RGBAColor>& image,
                const ImageWriteOptions& options, const oiio::ParamValueList& metadata)
{
    writeImage(path, oiio::TypeDesc::UINT8, 4, image, options, metadata);
}

void writeImage(const std::string& path, 
                const Image<RGBfColor>& image,
                const ImageWriteOptions& options, 
                const oiio::ParamValueList& metadata,
                const oiio::ROI& displayRoi,
                const oiio::ROI& pixelRoi)
{
    writeImage(path, oiio::TypeDesc::FLOAT, 3, image, options, metadata, displayRoi, pixelRoi);
}

void writeImage(const std::string& path, const Image<RGBColor>& image,
                const ImageWriteOptions& options, const oiio::ParamValueList& metadata)
{
    writeImage(path, oiio::TypeDesc::UINT8, 3, image, options, metadata);
}

void writeImageWithFloat(const std::string& path, const Image<unsigned char>& image,
                         const ImageWriteOptions& options, const oiio::ParamValueList& metadata)
{
    writeImage(path, oiio::TypeDesc::UINT8, 1, image, options, metadata);
}

void writeImageWithFloat(const std::string& path, const Image<int>& image,
                         const ImageWriteOptions& options, const oiio::ParamValueList& metadata)
{
    writeImage(path, oiio::TypeDesc::INT32, 1, image, options, metadata);
}

void writeImageWithFloat(const std::string& path, const Image<IndexT>& image,
                         const ImageWriteOptions& options, const oiio::ParamValueList& metadata)
{
    writeImage(path, oiio::TypeDesc::UINT32, 1, image, options, metadata);
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
