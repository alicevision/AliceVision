// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#ifdef __CUDA_FP16_HPP__
#define __CUDA_FP16_HPP_BAK__ __CUDA_FP16_HPP__
#undef __CUDA_FP16_HPP__
#endif

#include "Image.hpp"
#include "pixelTypes.hpp"
#include "colorspace.hpp"

#ifdef __CUDA_FP16_HPP_BAK__
#define __CUDA_FP16_HPP__ __CUDA_FP16_HPP_BAK__ 
#undef __CUDA_FP16_HPP_BAK__
#endif

#include <aliceVision/types.hpp>

#include <OpenImageIO/paramlist.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/color.h>

#include <string>

namespace aliceVision {

class rgb;

namespace image {

/**
 * @brief Available raw processing methods
 */
enum class ERawColorInterpretation
{
    /// Debayering without any color processing
    None,
    /// Simple neutralization
    LibRawNoWhiteBalancing,
    /// Use internal white balancing from libraw
    LibRawWhiteBalancing,
    /// If DCP file is not available throw an exception
    DcpLinearProcessing,
    /// If DCP file is not available throw an exception
    DcpMetadata,
    /// Access aliceVision:rawColorInterpretation metadata to set the method
    Auto
};

std::string ERawColorInterpretation_informations();
ERawColorInterpretation ERawColorInterpretation_stringToEnum(const std::string& dataType);
std::string ERawColorInterpretation_enumToString(const ERawColorInterpretation dataType);
std::ostream& operator<<(std::ostream& os, ERawColorInterpretation dataType);
std::istream& operator>>(std::istream& in, ERawColorInterpretation& dataType);

/**
 * @brief Available image file type for pipeline output
 */
enum class EImageFileType
{
    JPEG,
    PNG,
    TIFF,
    EXR,
    NONE
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
 * @brief Return a list of extensions supported by OpenImageIO (ie. extensions existing in extension_list from imageio.h).
 *        The list of supported extensions also includes video formats.
 * @return a vector containing all the extensions supported by OpenImageIO.
 */
std::vector<std::string> getSupportedExtensions();

/**
 * @brief Check if an input image extension is supported by OpenImageIO (ie. it exists in extension_list from imageio.h).
 *        This function might return true if the input is a video rather than an image, as long as the video format is
 *        supported by OpenImageIO.
 * @param[in] extension the input image extension.
 * @return true if the extension is valid and supported by OpenImageIO, false otherwise.
 */
bool isSupported(const std::string& extension);

/**
 * @brief Check if the extension is a video format supported by OpenImageIO, based on the list provided by OpenImageIO
 *        (https://openimageio.readthedocs.io/en/latest/builtinplugins.html#movie-formats-using-ffmpeg).
 * @param[in] extension the input file's extension.
 * @return true if the extension is a valid video extension supported by OpenImageIO, false otherwise.
 */
bool isVideoExtension(const std::string& extension);

/**
 * @brief Data type use to write the image
 */
enum class EStorageDataType
{
    Float,       //< Use full floating point precision to store
    Half,        //< Use half (values our of range could become inf or nan)
    HalfFinite,  //< Use half, but ensures out-of-range pixels are clamps to keep finite pixel values
    Auto,        //< Use half if all pixels can be stored in half without clamp, else use full float
    Undefined    //< Storage data type is not defined and should be inferred from other sources
};

std::string EStorageDataType_informations();
EStorageDataType EStorageDataType_stringToEnum(const std::string& dataType);
std::string EStorageDataType_enumToString(const EStorageDataType dataType);
std::ostream& operator<<(std::ostream& os, EStorageDataType dataType);
std::istream& operator>>(std::istream& in, EStorageDataType& dataType);

/**
 * @brief Compression method used to write an exr image
 */
enum class EImageExrCompression
{
    None,
    Auto,
    RLE,
    ZIP,
    ZIPS,
    PIZ,
    PXR24,
    B44,
    B44A,
    DWAA,
    DWAB
};

std::string EImageExrCompression_informations();
EImageExrCompression EImageExrCompression_stringToEnum(const std::string& dataType);
std::string EImageExrCompression_enumToString(const EImageExrCompression dataType);
std::ostream& operator<<(std::ostream& os, EImageExrCompression dataType);
std::istream& operator>>(std::istream& in, EImageExrCompression& dataType);

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
 * @brief aggregate for multiple image reading options
 */
struct ImageReadOptions
{
    ImageReadOptions(EImageColorSpace workingColorSpace = EImageColorSpace::AUTO,
                     EImageColorSpace inputColorSpace = EImageColorSpace::AUTO,
                     ERawColorInterpretation rawColorInterpretation = ERawColorInterpretation::LibRawWhiteBalancing,
                     const std::string& colorProfile = "",
                     const bool useDCPColorMatrixOnly = true,
                     const oiio::ROI& roi = oiio::ROI())
      : workingColorSpace(workingColorSpace),
        inputColorSpace(inputColorSpace),
        rawColorInterpretation(rawColorInterpretation),
        colorProfileFileName(colorProfile),
        useDCPColorMatrixOnly(useDCPColorMatrixOnly),
        doWBAfterDemosaicing(false),
        demosaicingAlgo("AHD"),
        highlightMode(0),
        rawAutoBright(false),
        rawExposureAdjustment(1.0),
        correlatedColorTemperature(-1.0),
        subROI(roi)
    {}

    EImageColorSpace workingColorSpace;
    EImageColorSpace inputColorSpace;
    ERawColorInterpretation rawColorInterpretation;
    std::string colorProfileFileName;
    bool useDCPColorMatrixOnly;
    bool doWBAfterDemosaicing;
    std::string demosaicingAlgo;
    int highlightMode;
    bool rawAutoBright;
    float rawExposureAdjustment;
    double correlatedColorTemperature;
    // ROI for this image.
    // If the image contains an roi, this is the roi INSIDE the roi.
    oiio::ROI subROI;
};

/**
 * @brief aggregate for multiple image writing options
 */
class ImageWriteOptions
{
  public:
    ImageWriteOptions() = default;

    EImageColorSpace getFromColorSpace() const { return _fromColorSpace; }
    EImageColorSpace getToColorSpace() const { return _toColorSpace; }
    EStorageDataType getStorageDataType() const { return _storageDataType; }
    EImageExrCompression getExrCompressionMethod() const { return _exrCompressionMethod; }
    int getExrCompressionLevel() const { return _exrCompressionLevel; }
    bool getJpegCompress() const { return _jpegCompress; }
    int getJpegQuality() const { return _jpegQuality; }

    ImageWriteOptions& fromColorSpace(EImageColorSpace colorSpace)
    {
        _fromColorSpace = colorSpace;
        return *this;
    }

    ImageWriteOptions& toColorSpace(EImageColorSpace colorSpace)
    {
        _toColorSpace = colorSpace;
        return *this;
    }

    ImageWriteOptions& storageDataType(EStorageDataType storageDataType)
    {
        _storageDataType = storageDataType;
        return *this;
    }

    ImageWriteOptions& exrCompressionMethod(EImageExrCompression compressionMethod)
    {
        _exrCompressionMethod = compressionMethod;
        return *this;
    }

    ImageWriteOptions& exrCompressionLevel(int compressionLevel)
    {
        _exrCompressionLevel = compressionLevel;
        return *this;
    }

    ImageWriteOptions& jpegCompress(bool compress)
    {
        _jpegCompress = compress;
        return *this;
    }

    ImageWriteOptions& jpegQuality(int quality)
    {
        _jpegQuality = quality;
        return *this;
    }

  private:
    EImageColorSpace _fromColorSpace{EImageColorSpace::LINEAR};
    EImageColorSpace _toColorSpace{EImageColorSpace::AUTO};
    EStorageDataType _storageDataType{EStorageDataType::Undefined};
    EImageExrCompression _exrCompressionMethod{EImageExrCompression::Auto};
    int _exrCompressionLevel{0};
    bool _jpegCompress{true};
    int _jpegQuality{90};
};

/**
 * @brief Test if the extension is supported for undistorted images.
 * @param[in] ext The extension with the dot (eg ".png")
 * @return \p true if the extension is supported.
 */
bool isSupportedUndistortFormat(const std::string& ext);

/**
 * @brief convert a metadata string map into an oiio::ParamValueList
 * @param[in] metadataMap string map
 * @return oiio::ParamValueList
 */
oiio::ParamValueList getMetadataFromMap(const std::map<std::string, std::string>& metadataMap);

/**
 * @brief convert an oiio::ParamValueList into metadata string map
 * @param[in] metadata An instance of oiio::ParamValueList
 * @return std::map Metadata string map
 */
// Warning: type conversion problems from string to param value, we may lose some metadata with string maps
std::map<std::string, std::string> getMapFromMetadata(const oiio::ParamValueList& metadata);

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
 * @brief extract entire image specification from an image for a given path
 * @param[in] path The given path to the image
 * @return imageSpec Specification describing the image
 */
oiio::ImageSpec readImageSpec(const std::string& path);

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
void readImage(const std::string& path, Image<float>& image, const ImageReadOptions& imageReadOptions);
void readImage(const std::string& path, Image<unsigned char>& image, const ImageReadOptions& imageReadOptions);
void readImage(const std::string& path, Image<IndexT>& image, const ImageReadOptions& imageReadOptions);
void readImage(const std::string& path, Image<RGBAfColor>& image, const ImageReadOptions& imageReadOptions);
void readImage(const std::string& path, Image<RGBAColor>& image, const ImageReadOptions& imageReadOptions);
void readImage(const std::string& path, Image<RGBfColor>& image, const ImageReadOptions& imageReadOptions);
void readImage(const std::string& path, Image<RGBColor>& image, const ImageReadOptions& imageReadOptions);

/**
 * @brief read an image with a given path and buffer without any processing such as color conversion
 * @param[in] path The given path to the image
 * @param[out] image The output image buffer
 */
void readImageDirect(const std::string& path, Image<IndexT>& image);
void readImageDirect(const std::string& path, Image<unsigned char>& image);

/**
 * @brief log information about the memory usage of the OIIO default shared image cache
 */
void logOIIOImageCacheInfo();

/**
 * @brief write an image with a given path and buffer
 * @param[in] path The given path to the image
 * @param[in] image The output image buffer
 */
void writeImage(const std::string& path,
                const Image<unsigned char>& image,
                const ImageWriteOptions& options,
                const oiio::ParamValueList& metadata = oiio::ParamValueList());

void writeImage(const std::string& path,
                const Image<int>& image,
                const ImageWriteOptions& options,
                const oiio::ParamValueList& metadata = oiio::ParamValueList());

void writeImage(const std::string& path,
                const Image<IndexT>& image,
                const ImageWriteOptions& options,
                const oiio::ParamValueList& metadata = oiio::ParamValueList());

void writeImage(const std::string& path,
                const Image<float>& image,
                const ImageWriteOptions& options,
                const oiio::ParamValueList& metadata = oiio::ParamValueList(),
                const oiio::ROI& displayRoi = oiio::ROI(),
                const oiio::ROI& pixelRoi = oiio::ROI());

void writeImage(const std::string& path,
                const Image<RGBAfColor>& image,
                const ImageWriteOptions& options,
                const oiio::ParamValueList& metadata = oiio::ParamValueList(),
                const oiio::ROI& displayRoi = oiio::ROI(),
                const oiio::ROI& pixelRoi = oiio::ROI());

void writeImage(const std::string& path,
                const Image<RGBAColor>& image,
                const ImageWriteOptions& options,
                const oiio::ParamValueList& metadata = oiio::ParamValueList());

void writeImage(const std::string& path,
                const Image<RGBfColor>& image,
                const ImageWriteOptions& options,
                const oiio::ParamValueList& metadata = oiio::ParamValueList(),
                const oiio::ROI& displayRoi = oiio::ROI(),
                const oiio::ROI& pixelRoi = oiio::ROI());

void writeImage(const std::string& path,
                const Image<RGBColor>& image,
                const ImageWriteOptions& options,
                const oiio::ParamValueList& metadata = oiio::ParamValueList());

/**
 * @brief write an image with a given path and buffer, converting to float as necessary to perform
 * intermediate calculations.
 * @param[in] path The given path to the image
 * @param[in] image The output image buffer
 */
void writeImageWithFloat(const std::string& path,
                         const Image<unsigned char>& image,
                         const ImageWriteOptions& options,
                         const oiio::ParamValueList& metadata = oiio::ParamValueList());

void writeImageWithFloat(const std::string& path,
                         const Image<int>& image,
                         const ImageWriteOptions& options,
                         const oiio::ParamValueList& metadata = oiio::ParamValueList());

void writeImageWithFloat(const std::string& path,
                         const Image<IndexT>& image,
                         const ImageWriteOptions& options,
                         const oiio::ParamValueList& metadata = oiio::ParamValueList());

template<typename T>
struct ColorTypeInfo
{
    // no size parameter, so no default value.
    // An error will be raise at compile time if this type traits is not defined.
};

template<>
struct ColorTypeInfo<unsigned char>
{
    static const int size = 1;
    static const oiio::TypeDesc::BASETYPE typeDesc = oiio::TypeDesc::UINT8;
};
template<>
struct ColorTypeInfo<float>
{
    static const int size = 1;
    static const oiio::TypeDesc::BASETYPE typeDesc = oiio::TypeDesc::FLOAT;
};
template<>
struct ColorTypeInfo<RGBColor>
{
    static const int size = 3;
    static const oiio::TypeDesc::BASETYPE typeDesc = oiio::TypeDesc::UINT8;
};
template<>
struct ColorTypeInfo<RGBfColor>
{
    static const int size = 3;
    static const oiio::TypeDesc::BASETYPE typeDesc = oiio::TypeDesc::FLOAT;
};
template<>
struct ColorTypeInfo<RGBAColor>
{
    static const int size = 4;
    static const oiio::TypeDesc::BASETYPE typeDesc = oiio::TypeDesc::UINT8;
};
template<>
struct ColorTypeInfo<RGBAfColor>
{
    static const int size = 4;
    static const oiio::TypeDesc::BASETYPE typeDesc = oiio::TypeDesc::FLOAT;
};

bool isRawFormat(const std::string& path);

bool tryLoadMask(Image<unsigned char>* mask,
                 const std::vector<std::string>& masksFolders,
                 const IndexT viewId,
                 const std::string& srcImage,
                 const std::string& fileExtension);

/**
 * Returns the value of ALICEVISION_ROOT environmental variable, or empty string if it is not
 * defined. The returned value can be overridden by `setAliceVisionRootOverride` if needed, for
 * example in tests.
 */
// TODO: use std::optional when the C++ standard version is upgraded to C++17
std::string getAliceVisionRoot();


}  // namespace image
}  // namespace aliceVision
