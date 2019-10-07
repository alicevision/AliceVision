#include "imageAlgo.hpp"


#include <aliceVision/mvsData/Color.hpp>
#include <aliceVision/mvsData/Rgb.hpp>
#include <aliceVision/mvsData/Image.hpp>


#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>

#include <Eigen/Eigen>

namespace aliceVision
{
namespace imageAlgo
{

float func_XYZtoLAB(float t)
{
    if(t > 0.008856f)
        return std::pow(t, 1.0f/3.0f);
    else
        return t / 0.1284f + 0.1379f;
}

float func_LABtoXYZ(float t)
{
    if(t > 0.2069f)
        return std::pow(t, 3.0f);
    else
        return 0.1284f * (t - 0.1379f);
}

void RGBtoXYZ(oiio::ImageBuf::Iterator<float>& pixel)
{
    const Eigen::Vector3f rgb(pixel[0], pixel[1], pixel[2]);
    Eigen::Matrix3f M;
    M << 0.4124f, 0.3576f, 0.1805f,
            0.2126f, 0.7152f, 0.0722f,
            0.0193f, 0.1192f, 0.9504f;
    const Eigen::Vector3f xyz_vec = M * rgb;

    pixel[0] = xyz_vec[0] * 0.9505f;
    pixel[1] = xyz_vec[1];
    pixel[2] = xyz_vec[2] * 1.0890f;
}

void XYZtoRGB(oiio::ImageBuf::Iterator<float>& pixel)
{
    const Eigen::Vector3f xyz(pixel[0] / 0.9505f, pixel[1], pixel[2] / 1.0890f);
    Eigen::Matrix3f M;
    M << 3.2406f, -1.5372f, -0.4986f,
            -0.9689f, 1.8758f, 0.0415f,
            0.0557f, -0.2040f, 1.0570f;
    const Eigen::Vector3f rgb_vec = M * xyz;

    pixel[0] = rgb_vec[0];
    pixel[1] = rgb_vec[1];
    pixel[2] = rgb_vec[2];
}

void XYZtoLAB(oiio::ImageBuf::Iterator<float>& pixel)
{
    float L = 116.0f * func_XYZtoLAB(pixel[1]) - 16.0f;
    float A = 500.0f * (func_XYZtoLAB(pixel[0]) - func_XYZtoLAB(pixel[1]));
    float B = 200.0f * (func_XYZtoLAB(pixel[1]) - func_XYZtoLAB(pixel[2]));

    pixel[0] = L / 100.0f;
    pixel[1] = A / 100.0f;
    pixel[2] = B / 100.0f;
}

void LABtoXYZ(oiio::ImageBuf::Iterator<float>& pixel)
{
    float L_offset = (pixel[0] * 100.0f + 16.0f) / 116.0f;

    pixel[0] = func_LABtoXYZ(L_offset + pixel[1] * 100.0f / 500.0f);
    pixel[1] = func_LABtoXYZ(L_offset);
    pixel[2] = func_LABtoXYZ(L_offset - pixel[2] * 100.0f / 200.0f);
}

void RGBtoLAB(oiio::ImageBuf::Iterator<float>& pixel)
{
    RGBtoXYZ(pixel);
    XYZtoLAB(pixel);
}

void LABtoRGB(oiio::ImageBuf::Iterator<float>& pixel)
{
    LABtoXYZ(pixel);
    XYZtoRGB(pixel);
}

void processImage(oiio::ImageBuf& image, std::function<void(oiio::ImageBuf::Iterator<float>&)> pixelFunc)
{
    oiio::ImageBufAlgo::parallel_image(image.roi(), [&image, &pixelFunc](oiio::ROI roi) {
        for(oiio::ImageBuf::Iterator<float> pixel(image, roi); !pixel.done(); ++pixel)
        {
            pixelFunc(pixel);
        }
    });
}

void processImage(oiio::ImageBuf& dst, const oiio::ImageBuf& src, std::function<void(oiio::ImageBuf::Iterator<float>&)> pixelFunc)
{
    dst.copy(src);
    processImage(dst, pixelFunc);
}

void colorconvert(oiio::ImageBuf& imgBuf, imageIO::EImageColorSpace fromColorSpace, imageIO::EImageColorSpace toColorSpace)
{
    using namespace imageIO;

    if(fromColorSpace == toColorSpace)
        return;

    else if(toColorSpace == EImageColorSpace::LINEAR)
    {
        if(fromColorSpace == EImageColorSpace::SRGB)
            oiio::ImageBufAlgo::colorconvert(imgBuf, imgBuf,
                                             EImageColorSpace_enumToString(EImageColorSpace::SRGB), EImageColorSpace_enumToString(EImageColorSpace::LINEAR));
        else if(fromColorSpace == EImageColorSpace::XYZ)
            processImage(imgBuf, &XYZtoRGB);
        else if(fromColorSpace == EImageColorSpace::LAB)
            processImage(imgBuf, &LABtoRGB);
    }
    else if(toColorSpace == EImageColorSpace::SRGB)
    {
        if(fromColorSpace == EImageColorSpace::XYZ)
            processImage(imgBuf, &XYZtoRGB);
        else if(fromColorSpace == EImageColorSpace::LAB)
            processImage(imgBuf, &LABtoRGB);
        oiio::ImageBufAlgo::colorconvert(imgBuf, imgBuf,
                                         EImageColorSpace_enumToString(EImageColorSpace::LINEAR), EImageColorSpace_enumToString(EImageColorSpace::SRGB));
    }
    else if(toColorSpace == EImageColorSpace::XYZ)
    {
        if(fromColorSpace == EImageColorSpace::LINEAR)
            processImage(imgBuf, &RGBtoXYZ);
        else if(fromColorSpace == EImageColorSpace::SRGB)
        {
            oiio::ImageBufAlgo::colorconvert(imgBuf, imgBuf,
                                             EImageColorSpace_enumToString(EImageColorSpace::SRGB), EImageColorSpace_enumToString(EImageColorSpace::LINEAR));
            processImage(imgBuf, &RGBtoXYZ);
        }
        else if(fromColorSpace == EImageColorSpace::LAB)
            processImage(imgBuf, &LABtoXYZ);
    }
    else if(toColorSpace == EImageColorSpace::LAB)
    {
        if(fromColorSpace == EImageColorSpace::LINEAR)
            processImage(imgBuf, &RGBtoLAB);
        else if(fromColorSpace == EImageColorSpace::SRGB)
        {
            oiio::ImageBufAlgo::colorconvert(imgBuf, imgBuf,
                                             EImageColorSpace_enumToString(EImageColorSpace::SRGB), EImageColorSpace_enumToString(EImageColorSpace::LINEAR));
            processImage(imgBuf, &RGBtoLAB);
        }
        else if(fromColorSpace == EImageColorSpace::XYZ)
            processImage(imgBuf, &XYZtoLAB);
    }
    ALICEVISION_LOG_TRACE("Convert image from " << EImageColorSpace_enumToString(fromColorSpace) << " to " << EImageColorSpace_enumToString(toColorSpace));
}

void colorconvert(ImageRGBf& image, imageIO::EImageColorSpace fromColorSpace, imageIO::EImageColorSpace toColorSpace)
{
    oiio::ImageSpec imageSpec(image.width(), image.height(), 3, oiio::TypeDesc::FLOAT);
    std::vector<ColorRGBf>& buffer = image.data();
    oiio::ImageBuf imageBuf(imageSpec, buffer.data());

    colorconvert(imageBuf, fromColorSpace, toColorSpace);
}

void colorconvert(ImageRGBAf& image, imageIO::EImageColorSpace fromColorSpace, imageIO::EImageColorSpace toColorSpace)
{
    oiio::ImageSpec imageSpec(image.width(), image.height(), 4, oiio::TypeDesc::FLOAT);
    std::vector<ColorRGBAf>& buffer = image.data();
    oiio::ImageBuf imageBuf(imageSpec, buffer.data());

    colorconvert(imageBuf, fromColorSpace, toColorSpace);
}

void colorconvert(oiio::ImageBuf& dst, const oiio::ImageBuf& src, imageIO::EImageColorSpace fromColorSpace, imageIO::EImageColorSpace toColorSpace)
{
    dst.copy(src);
    colorconvert(dst, fromColorSpace, toColorSpace);
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

void transposeImage(int width, int height, std::vector<ColorRGBf>& buffer)
{
    transposeImage(oiio::TypeDesc::FLOAT, width, height, 3, buffer);
}

void transposeImage(ImageRGBf &image)
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

void resizeImage(int inWidth, int inHeight, int downscale, const std::vector<ColorRGBf>& inBuffer, std::vector<ColorRGBf>& outBuffer, const std::string& filter, float filterSize)
{
    resizeImage(oiio::TypeDesc::FLOAT, inWidth, inHeight, 3, downscale, inBuffer, outBuffer, filter, filterSize);
}

void resizeImage(int downscale, const ImageRGBf &inImage, ImageRGBf &outImage, const std::string &filter, float filterSize)
{
    resizeImage(oiio::TypeDesc::FLOAT, inImage.width(), inImage.height(), 3, downscale, inImage.data(), outImage.data(), filter, filterSize);
    outImage.setHeight(inImage.height() / downscale);
    outImage.setWidth(inImage.width() / downscale);
}

void resizeImage(int downscale, const ImageRGBAf &inImage, ImageRGBAf &outImage, const std::string &filter, float filterSize)
{
    resizeImage(oiio::TypeDesc::FLOAT, inImage.width(), inImage.height(), 4, downscale, inImage.data(), outImage.data(), filter, filterSize);
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

void convolveImage(int inWidth, int inHeight, const std::vector<ColorRGBf>& inBuffer, std::vector<ColorRGBf>& outBuffer, const std::string& kernel, float kernelWidth, float kernelHeight)
{
  convolveImage(oiio::TypeDesc::FLOAT, inWidth, inHeight, 3, inBuffer, outBuffer, kernel, kernelWidth, kernelHeight);
}

void convolveImage(const ImageRGBf &inImage, ImageRGBf &outImage, const std::string &kernel, float kernelWidth, float kernelHeight)
{
    convolveImage(oiio::TypeDesc::FLOAT, inImage.width(), inImage.height(), 3, inImage.data(), outImage.data(), kernel, kernelWidth, kernelHeight);
    outImage.setHeight(inImage.height());
    outImage.setWidth(inImage.width());
}

void fillHoles(int inWidth, int inHeight, std::vector<ColorRGBf>& colorBuffer, const std::vector<float>& alphaBuffer)
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

void fillHoles(ImageRGBf& image, const std::vector<float>& alphaBuffer)
{
    fillHoles(image.width(), image.height(), image.data(), alphaBuffer);
}


} // namespace imageAlgo
} // namspace aliceVision
