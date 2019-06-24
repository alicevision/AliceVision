#include "imageAlgo.hpp"


#include <aliceVision/mvsData/Color.hpp>
#include <aliceVision/mvsData/Rgb.hpp>
#include <aliceVision/mvsData/Image.hpp>


#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>

namespace aliceVision
{
namespace imageAlgo
{
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

void transposeImage(Image &image)
{
    transposeImage(oiio::TypeDesc::FLOAT, image.width(), image.height(), 3, image.data());
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

void resizeImage(int downscale, const Image &inImage, Image &outImage, const std::string &filter, float filterSize)
{
    resizeImage(oiio::TypeDesc::FLOAT, inImage.width(), inImage.height(), 3, downscale, inImage.data(), outImage.data(), filter, filterSize);
    outImage.setHeight(inImage.height() / downscale);
    outImage.setWidth(inImage.width() / downscale);
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

void convolveImage(const Image &inImage, Image &outImage, const std::string &kernel, float kernelWidth, float kernelHeight)
{
    convolveImage(oiio::TypeDesc::FLOAT, inImage.width(), inImage.height(), 3, inImage.data(), outImage.data(), kernel, kernelWidth, kernelHeight);
    outImage.setHeight(inImage.height());
    outImage.setWidth(inImage.width());
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

void fillHoles(Image& image, const std::vector<float>& alphaBuffer)
{
    fillHoles(image.width(), image.height(), image.data(), alphaBuffer);
}


} // namespace imageAlgo
} // namspace aliceVision
