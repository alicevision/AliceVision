#include "imageAlgo.hpp"

#include <aliceVision/image/Image.hpp>
#include <aliceVision/image/Rgb.hpp>
#include <aliceVision/system/Logger.hpp>


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

void colorconvert(oiio::ImageBuf& imgBuf, const std::string& fromColorSpaceOIIOName,
                  image::EImageColorSpace toColorSpace)
{
    using image::EImageColorSpace;

    if (fromColorSpaceOIIOName == image::EImageColorSpace_enumToOIIOString(toColorSpace))
        return;

    if (!image::EImageColorSpace_isSupportedOIIOstring(fromColorSpaceOIIOName))
    {
        if (!EImageColorSpace_isSupportedOIIOEnum(toColorSpace))
        {
            // We don't know about OIIO format and OIIO does not know about the destination format.
            // Convert to LINEAR and then do conversion as usual (colorconvert will handle
            // formats unknown to OIIO)
            oiio::ImageBufAlgo::colorconvert(imgBuf, imgBuf, fromColorSpaceOIIOName,
                                             image::EImageColorSpace_enumToOIIOString(EImageColorSpace::LINEAR));
            colorconvert(imgBuf, EImageColorSpace::LINEAR, toColorSpace);
            return;
        }
        // We don't know about OIIO format, but OIIO knows about the destination format
        oiio::ImageBufAlgo::colorconvert(imgBuf, imgBuf, fromColorSpaceOIIOName,
                                         image::EImageColorSpace_enumToOIIOString(toColorSpace));
    }
    else
    {
        auto fromColorSpace = image::EImageColorSpace_OIIOstringToEnum(fromColorSpaceOIIOName);
        colorconvert(imgBuf, fromColorSpace, toColorSpace);
    }
}

void colorconvert(oiio::ImageBuf& imgBuf, image::EImageColorSpace fromColorSpace,
                  image::EImageColorSpace toColorSpace)
{
    using image::EImageColorSpace;

    if(fromColorSpace == toColorSpace)
        return;

    else if(toColorSpace == EImageColorSpace::LINEAR)
    {
        if(fromColorSpace == EImageColorSpace::SRGB)
            oiio::ImageBufAlgo::colorconvert(imgBuf, imgBuf,
                                             EImageColorSpace_enumToOIIOString(EImageColorSpace::SRGB),
                                             EImageColorSpace_enumToOIIOString(EImageColorSpace::LINEAR));
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
                                         EImageColorSpace_enumToOIIOString(EImageColorSpace::LINEAR),
                                         EImageColorSpace_enumToOIIOString(EImageColorSpace::SRGB));
    }
    else if(toColorSpace == EImageColorSpace::XYZ)
    {
        if(fromColorSpace == EImageColorSpace::LINEAR)
            processImage(imgBuf, &RGBtoXYZ);
        else if(fromColorSpace == EImageColorSpace::SRGB)
        {
            oiio::ImageBufAlgo::colorconvert(imgBuf, imgBuf,
                                             EImageColorSpace_enumToOIIOString(EImageColorSpace::SRGB),
                                             EImageColorSpace_enumToOIIOString(EImageColorSpace::LINEAR));
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
                                             EImageColorSpace_enumToOIIOString(EImageColorSpace::SRGB),
                                             EImageColorSpace_enumToOIIOString(EImageColorSpace::LINEAR));
            processImage(imgBuf, &RGBtoLAB);
        }
        else if(fromColorSpace == EImageColorSpace::XYZ)
            processImage(imgBuf, &XYZtoLAB);
    }
    ALICEVISION_LOG_TRACE("Convert image from " << EImageColorSpace_enumToString(fromColorSpace) << " to " << EImageColorSpace_enumToString(toColorSpace));
}

void colorconvert(image::Image<image::RGBfColor>& image, image::EImageColorSpace fromColorSpace,
                  image::EImageColorSpace toColorSpace)
{
    oiio::ImageSpec imageSpec(image.Width(), image.Height(), 3, oiio::TypeDesc::FLOAT);
    auto* buffer = image.data();
    oiio::ImageBuf imageBuf(imageSpec, buffer->data());

    colorconvert(imageBuf, fromColorSpace, toColorSpace);
}

void colorconvert(image::Image<image::RGBAfColor>& image, image::EImageColorSpace fromColorSpace,
                  image::EImageColorSpace toColorSpace)
{
    oiio::ImageSpec imageSpec(image.Width(), image.Height(), 4, oiio::TypeDesc::FLOAT);
    auto* buffer = image.data();
    oiio::ImageBuf imageBuf(imageSpec, buffer->data());

    colorconvert(imageBuf, fromColorSpace, toColorSpace);
}

void colorconvert(oiio::ImageBuf& dst, const oiio::ImageBuf& src,
                  image::EImageColorSpace fromColorSpace, image::EImageColorSpace toColorSpace)
{
    dst.copy(src);
    colorconvert(dst, fromColorSpace, toColorSpace);
}

template<typename T>
void resizeImage(oiio::TypeDesc typeDesc,
                 int inWidth,
                 int inHeight,
                 int outWidth,
                 int outHeight,
                 int nchannels,
                 const T* inBuffer,
                 T* outBuffer,
                 const std::string& filter,
                 float filterSize)
{
    const oiio::ImageBuf inBuf(oiio::ImageSpec(inWidth, inHeight, nchannels, typeDesc),
                               const_cast<T*>(inBuffer));
    oiio::ImageBuf outBuf(oiio::ImageSpec(outWidth, outHeight, nchannels, typeDesc), outBuffer);

    oiio::ImageBufAlgo::resize(outBuf, inBuf, filter, filterSize, oiio::ROI::All());
}

void resizeImage(int downscale, const image::Image<unsigned char>& inImage,
                 image::Image<unsigned char>& outImage, const std::string& filter, float filterSize)
{
    const int outWidth = inImage.Width() / downscale;
    const int outHeight = inImage.Height() / downscale;
    outImage.resize(outWidth, outHeight);
    resizeImage(oiio::TypeDesc::UINT8, inImage.Width(), inImage.Height(), outWidth, outHeight, 1,
                inImage.data(), outImage.data(), filter, filterSize);
}

void resizeImage(int downscale, image::Image<unsigned char>& inoutImage,
                 const std::string& filter, float filterSize)
{
    if (downscale <= 1)
    {
        return;
    }

    const int outWidth = inoutImage.Width() / downscale;
    const int outHeight = inoutImage.Height() / downscale;

    image::Image<unsigned char> rescaled(outWidth, outHeight);

    resizeImage(oiio::TypeDesc::UINT8, inoutImage.Width(), inoutImage.Height(), outWidth, outHeight, 1,
                inoutImage.data(), rescaled.data(), filter, filterSize);
    
    inoutImage.swap(rescaled);
}

void resizeImage(int downscale, const image::Image<float>& inImage,
                 image::Image<float>& outImage, const std::string& filter, float filterSize)
{
    const int outWidth = inImage.Width() / downscale;
    const int outHeight = inImage.Height() / downscale;
    outImage.resize(outWidth, outHeight);
    resizeImage(oiio::TypeDesc::FLOAT, inImage.Width(), inImage.Height(), outWidth, outHeight, 1,
                inImage.data(), outImage.data(), filter, filterSize);
}

void resizeImage(int downscale, image::Image<float>& inoutImage,
                 const std::string& filter, float filterSize)
{
    if (downscale <= 1)
    {
        return;
    }

    const int outWidth = inoutImage.Width() / downscale;
    const int outHeight = inoutImage.Height() / downscale;

    image::Image<float> rescaled(outWidth, outHeight);

    resizeImage(oiio::TypeDesc::FLOAT, inoutImage.Width(), inoutImage.Height(), outWidth, outHeight, 1,
                inoutImage.data(), rescaled.data(), filter, filterSize);
    
    inoutImage.swap(rescaled);
}

void resizeImage(int downscale, const image::Image<image::RGBColor> &inImage,
                 image::Image<image::RGBColor> &outImage, const std::string &filter,
                 float filterSize)
{
    const int outWidth = inImage.Width() / downscale;
    const int outHeight = inImage.Height() / downscale;
    outImage.resize(outWidth, outHeight);
    resizeImage(oiio::TypeDesc::UINT8, inImage.Width(), inImage.Height(), outWidth, outHeight, 3,
                inImage.data(), outImage.data(), filter, filterSize);
}

void resizeImage(int downscale, image::Image<image::RGBColor>& inoutImage,
                 const std::string& filter, float filterSize)
{
    if (downscale <= 1)
    {
        return;
    }

    const int outWidth = inoutImage.Width() / downscale;
    const int outHeight = inoutImage.Height() / downscale;

    image::Image<image::RGBColor> rescaled(outWidth, outHeight);

    resizeImage(oiio::TypeDesc::UINT8, inoutImage.Width(), inoutImage.Height(), outWidth, outHeight, 3,
                inoutImage.data(), rescaled.data(), filter, filterSize);
    
    inoutImage.swap(rescaled);
}

void resizeImage(int downscale, const image::Image<image::RGBfColor> &inImage,
                 image::Image<image::RGBfColor> &outImage, const std::string &filter,
                 float filterSize)
{
    const int outWidth = inImage.Width() / downscale;
    const int outHeight = inImage.Height() / downscale;
    outImage.resize(outWidth, outHeight);
    resizeImage(oiio::TypeDesc::FLOAT, inImage.Width(), inImage.Height(), outWidth, outHeight, 3,
                inImage.data(), outImage.data(), filter, filterSize);
}

void resizeImage(int downscale, image::Image<image::RGBfColor>& inoutImage,
                 const std::string& filter, float filterSize)
{
    if (downscale <= 1)
    {
        return;
    }

    const int outWidth = inoutImage.Width() / downscale;
    const int outHeight = inoutImage.Height() / downscale;

    image::Image<image::RGBfColor> rescaled(outWidth, outHeight);

    resizeImage(oiio::TypeDesc::FLOAT, inoutImage.Width(), inoutImage.Height(), outWidth, outHeight, 3,
                inoutImage.data(), rescaled.data(), filter, filterSize);
    
    inoutImage.swap(rescaled);
}

void resizeImage(int downscale, const image::Image<image::RGBAColor> &inImage,
                 image::Image<image::RGBAColor> &outImage, const std::string &filter,
                 float filterSize)
{
    const int outWidth = inImage.Width() / downscale;
    const int outHeight = inImage.Height() / downscale;
    outImage.resize(outWidth, outHeight);
    resizeImage(oiio::TypeDesc::UINT8, inImage.Width(), inImage.Height(), outWidth, outHeight, 4,
                inImage.data(), outImage.data(), filter, filterSize);
}

void resizeImage(int downscale, image::Image<image::RGBAColor>& inoutImage,
                 const std::string& filter, float filterSize)
{
    if (downscale <= 1)
    {
        return;
    }

    const int outWidth = inoutImage.Width() / downscale;
    const int outHeight = inoutImage.Height() / downscale;

    image::Image<image::RGBAColor> rescaled(outWidth, outHeight);

    resizeImage(oiio::TypeDesc::UINT8, inoutImage.Width(), inoutImage.Height(), outWidth, outHeight, 4,
                inoutImage.data(), rescaled.data(), filter, filterSize);
    
    inoutImage.swap(rescaled);
}

void resizeImage(int downscale, const image::Image<image::RGBAfColor> &inImage,
                 image::Image<image::RGBAfColor> &outImage, const std::string &filter,
                 float filterSize)
{
    const int outWidth = inImage.Width() / downscale;
    const int outHeight = inImage.Height() / downscale;
    outImage.resize(outWidth, outHeight);
    resizeImage(oiio::TypeDesc::FLOAT, inImage.Width(), inImage.Height(), outWidth, outHeight, 4,
                inImage.data(), outImage.data(), filter, filterSize);
}

void resizeImage(int downscale, image::Image<image::RGBAfColor>& inoutImage,
                 const std::string& filter, float filterSize)
{
    if (downscale <= 1)
    {
        return;
    }

    const int outWidth = inoutImage.Width() / downscale;
    const int outHeight = inoutImage.Height() / downscale;

    image::Image<image::RGBAfColor> rescaled(outWidth, outHeight);

    resizeImage(oiio::TypeDesc::FLOAT, inoutImage.Width(), inoutImage.Height(), outWidth, outHeight, 4,
                inoutImage.data(), rescaled.data(), filter, filterSize);
    
    inoutImage.swap(rescaled);
}

template<typename T>
void convolveImage(oiio::TypeDesc typeDesc,
                   int inWidth,
                   int inHeight,
                   int nchannels,
                   const T* inBuffer,
                   T* outBuffer,
                   const std::string& kernel,
                   float kernelWidth,
                   float kernelHeight)
{
    const oiio::ImageBuf inBuf(oiio::ImageSpec(inWidth, inHeight, nchannels, typeDesc), const_cast<T*>(inBuffer));
    oiio::ImageBuf outBuf(oiio::ImageSpec(inWidth, inHeight, nchannels, typeDesc), outBuffer);

    oiio::ImageBuf K = oiio::ImageBufAlgo::make_kernel(kernel, kernelWidth, kernelHeight);

    oiio::ImageBufAlgo::convolve(outBuf, inBuf, K);
}

void convolveImage(const image::Image<unsigned char>& inBuffer,
                   image::Image<unsigned char>& outBuffer,
                   const std::string& kernel, float kernelWidth, float kernelHeight)
{
    outBuffer.resize(inBuffer.Width(), inBuffer.Height());
    convolveImage(oiio::TypeDesc::UCHAR, inBuffer.Width(), inBuffer.Height(), 1,
                  inBuffer.data(), outBuffer.data(),
                  kernel, kernelWidth, kernelHeight);
}

void convolveImage(const image::Image<rgb>& inBuffer, image::Image<rgb>& outBuffer,
                   const std::string& kernel, float kernelWidth, float kernelHeight)
{
    outBuffer.resize(inBuffer.Width(), inBuffer.Height());
    convolveImage(oiio::TypeDesc::UCHAR, inBuffer.Width(), inBuffer.Height(), 3,
                  inBuffer.data(), outBuffer.data(),
                  kernel, kernelWidth, kernelHeight);
}

void convolveImage(const image::Image<float>& inBuffer, image::Image<float>& outBuffer,
                   const std::string& kernel, float kernelWidth, float kernelHeight)
{
    outBuffer.resize(inBuffer.Width(), inBuffer.Height());
    convolveImage(oiio::TypeDesc::FLOAT, inBuffer.Width(), inBuffer.Height(), 1,
                  inBuffer.data(), outBuffer.data(),
                  kernel, kernelWidth, kernelHeight);
}

void convolveImage(const image::Image<image::RGBfColor>& inBuffer,
                   image::Image<image::RGBfColor>& outBuffer,
                   const std::string& kernel, float kernelWidth, float kernelHeight)
{
    outBuffer.resize(inBuffer.Width(), inBuffer.Height());
    convolveImage(oiio::TypeDesc::FLOAT, inBuffer.Width(), inBuffer.Height(), 3,
                  inBuffer.data(), outBuffer.data(),
                  kernel, kernelWidth, kernelHeight);
}

void fillHoles(int inWidth, int inHeight, image::RGBfColor* colorBuffer,
               const std::vector<float>& alphaBuffer)
{
    oiio::ImageBuf rgbBuf(oiio::ImageSpec(inWidth, inHeight, 3, oiio::TypeDesc::FLOAT), colorBuffer);
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

void fillHoles(image::Image<image::RGBfColor>& image, const std::vector<float>& alphaBuffer)
{
    fillHoles(image.Width(), image.Height(), image.data(), alphaBuffer);
}

void imageDiff(const image::Image<image::RGBfColor>& inImg,
               const image::Image<image::RGBfColor>& inImgDownscaled,
               image::Image<image::RGBfColor>& outImg, unsigned int downscale)
{
    outImg.resize(inImg.Width(), inImg.Height());

    for (int iy = 0; iy < inImg.Height(); iy++)
    {
        for (int ix = 0; ix < inImg.Width(); ix++)
        {
            outImg(iy, ix) = inImg(iy, ix) - getInterpolateColor(inImgDownscaled,
                                                                 iy / downscale, ix / downscale);
        }
    }
}

void laplacianPyramid(std::vector<image::Image<image::RGBfColor>>& out_pyramidL,
                      const image::Image<image::RGBfColor>& image, int nbBand, unsigned int downscale)
{
    assert(nbBand >= 1);

    image::Image<image::RGBfColor> img(image);
    int outW = static_cast<int>(img.Width()/downscale);
    int outH = static_cast<int>(img.Height()/downscale);

    image::Image<image::RGBfColor> imgDownscaled(outW, outH);
    out_pyramidL.resize(nbBand);

    //Create Laplacian pyramid
    for(int b = 0; b < nbBand-1; ++b)
    {
        imageAlgo::resizeImage(static_cast<int>(downscale), img, imgDownscaled, "gaussian");
        imageDiff(img, imgDownscaled, out_pyramidL[b], downscale);
        img.swap(imgDownscaled);
/*
        outW = static_cast<int>(outW/downscale);
        outH = static_cast<int>(outH/downscale);
        imgDownscaled.resize(outW, outH);
*/
    }
    out_pyramidL[nbBand-1] = img;

    for(std::size_t i = 0; i < out_pyramidL.size(); ++i)
        ALICEVISION_LOG_DEBUG("laplacianDownscalePyramid: Size level " << i << " : "
                              << out_pyramidL[i].Width() << "x" << out_pyramidL[i].Height());
}

} // namespace imageAlgo
} // namspace aliceVision
