// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "image.hpp"

#include <aliceVision/structures/mv_structures.hpp>
#include <aliceVision/structures/mv_color.hpp>

#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>

#include <OpenEXR/half.h>

#include <stdexcept>

namespace oiio = OIIO;

namespace imageIO {

void readImageSpec(const std::string& path, int& width, int& height, int& nchannels)
{
  std::unique_ptr<oiio::ImageInput> in(oiio::ImageInput::open(path));

  if(!in)
    throw std::runtime_error("Can't find/open image file '" + path + "'.");

  const oiio::ImageSpec &spec = in->spec();

  width = spec.width;
  height = spec.height;
  nchannels = spec.nchannels;

  in->close();
}

template<typename T>
void readImage(const std::string& path, oiio::TypeDesc typeDesc, int nchannels, std::vector<T>& buffer)
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

    buffer.resize(inSpec.width * inSpec.height * nchannels);

    {
        oiio::ROI exportROI = inBuf.roi();
        exportROI.chbegin = 0;
        exportROI.chend = nchannels;

        inBuf.get_pixels(exportROI, typeDesc, buffer.data());
    }
}

void readImage(const std::string& path, std::vector<unsigned char>& buffer)
{
    readImage(path, oiio::TypeDesc::UCHAR, 1, buffer);
}

void readImage(const std::string& path, std::vector<rgb>& buffer)
{
    readImage(path, oiio::TypeDesc::UCHAR, 3, buffer);
}

void readImage(const std::string& path, std::vector<float>& buffer)
{
    readImage(path, oiio::TypeDesc::FLOAT, 1, buffer);
}

void readImage(const std::string& path, std::vector<Color>& buffer)
{
    readImage(path, oiio::TypeDesc::FLOAT, 3, buffer);
}

template<typename T>
void writeImage(const std::string& path, oiio::TypeDesc typeDesc, int width, int height, int nchannels, std::vector<T>& buffer)
{
    bool isEXR = (path.size() > 4 && path.compare(path.size() - 4, 4, ".exr") == 0);

    oiio::ImageSpec imageSpec(width, height, nchannels, typeDesc);

    if(isEXR)
    {
        oiio::ImageBuf buf(imageSpec, buffer.data());

        imageSpec.format = oiio::TypeDesc::HALF;     // override format
        imageSpec.attribute("compression", "piz");   // if possible, PIZ compression for openEXR

        // conversion to half
        oiio::ImageBuf outBuf(imageSpec);
        outBuf.copy_pixels(buf);

        // write image
        if(!outBuf.write(path))
          throw std::runtime_error("Can't write output image file '" + path + "'.");
    }
    else
    {
        imageSpec.attribute("jpeg:subsampling", "4:4:4"); // if possible, always subsampling 4:4:4 for jpeg
        imageSpec.attribute("CompressionQuality", 100);   // if possible, best compression quality
        imageSpec.attribute("compression", "none");       // if possible, no compression

        oiio::ImageBuf outBuf(imageSpec, buffer.data());

        // write image
        if(!outBuf.write(path))
          throw std::runtime_error("Can't write output image file '" + path + "'.");
    }
}

void writeImage(const std::string& path, int width, int height, std::vector<unsigned char>& buffer)
{
    writeImage(path, oiio::TypeDesc::UCHAR, width, height, 1, buffer);
}

void writeImage(const std::string& path, int width, int height, std::vector<rgb>& buffer)
{
    writeImage(path, oiio::TypeDesc::UCHAR, width, height, 3, buffer);
}

void writeImage(const std::string& path, int width, int height, std::vector<float>& buffer)
{
    writeImage(path, oiio::TypeDesc::FLOAT, width, height, 1, buffer);
}

void writeImage(const std::string& path, int width, int height, std::vector<Color>& buffer)
{
    writeImage(path, oiio::TypeDesc::FLOAT, width, height, 3, buffer);
}

template<typename T>
void transposeImage(oiio::TypeDesc typeDesc, int width, int height, int nchannels, std::vector<T>& buffer)
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
void resizeImage(oiio::TypeDesc typeDesc, int inWidth, int inHeight, int nchannels, float scale, std::vector<T>& inBuffer, std::vector<T>& outBuffer, const std::string& filter = "", float filterSize = 0)
{
    const int outWidth = inWidth * scale;
    const int outHeight = inHeight * scale;

    outBuffer.resize(outWidth * outHeight);

    oiio::ImageBuf inBuf(oiio::ImageSpec(inWidth, inHeight, nchannels, typeDesc), inBuffer.data());
    oiio::ImageBuf outBuf(oiio::ImageSpec(outWidth, outHeight, nchannels, typeDesc), outBuffer.data());

    oiio::ImageBufAlgo::resize(outBuf, inBuf, filter, filterSize, oiio::ROI::All());
}

void resizeImage(int inWidth, int inHeight, float scale, std::vector<unsigned char>& inBuffer, std::vector<unsigned char>& outBuffer, const std::string& filter, float filterSize)
{
    resizeImage(oiio::TypeDesc::UCHAR, inWidth, inHeight, 1, scale, inBuffer, outBuffer, filter, filterSize);
}

void resizeImage(int inWidth, int inHeight, float scale, std::vector<rgb>& inBuffer, std::vector<rgb>& outBuffer, const std::string& filter, float filterSize)
{
    resizeImage(oiio::TypeDesc::UCHAR, inWidth, inHeight, 3, scale, inBuffer, outBuffer, filter, filterSize);
}

void resizeImage(int inWidth, int inHeight, float scale, std::vector<float>& inBuffer, std::vector<float>& outBuffer, const std::string& filter, float filterSize)
{
    resizeImage(oiio::TypeDesc::FLOAT, inWidth, inHeight, 1, scale, inBuffer, outBuffer, filter, filterSize);
}

void resizeImage(int inWidth, int inHeight, float scale, std::vector<Color>& inBuffer, std::vector<Color>& outBuffer, const std::string& filter, float filterSize)
{
    resizeImage(oiio::TypeDesc::FLOAT, inWidth, inHeight, 3, scale, inBuffer, outBuffer, filter, filterSize);
}

} // namespace imageIO
