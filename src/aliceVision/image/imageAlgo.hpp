#pragma once

#include <string>

#include <aliceVision/image/io.hpp>

#include <OpenImageIO/paramlist.h>
#include <OpenImageIO/imagebufalgo_util.h>


namespace oiio = OIIO;

namespace aliceVision
{

class rgb;

namespace imageAlgo
{

void RGBtoXYZ(oiio::ImageBuf::Iterator<float>& pixel);
void XYZtoRGB(oiio::ImageBuf::Iterator<float>& pixel);

void XYZtoLAB(oiio::ImageBuf::Iterator<float>& pixel);
void LABtoXYZ(oiio::ImageBuf::Iterator<float>& pixel);

void RGBtoLAB(oiio::ImageBuf::Iterator<float>& pixel);
void LABtoRGB(oiio::ImageBuf::Iterator<float>& pixel);

/**
 * @brief split an image in chunks and proces them in parallel
 * @param [in] image to process (in place or not)
 * @param [in] pixelFunc the function to apply
 */
void processImage(oiio::ImageBuf& image, std::function<void(oiio::ImageBuf::Iterator<float>&)> pixelFunc);
void processImage(oiio::ImageBuf& dst, const oiio::ImageBuf& src, std::function<void(oiio::ImageBuf::Iterator<float>&)> pixelFunc);

void colorconvert(oiio::ImageBuf& image, const std::string& fromColorSpaceOIIOName,
                  image::EImageColorSpace toColorSpace);
void colorconvert(oiio::ImageBuf& image, image::EImageColorSpace fromColorSpace,
                  image::EImageColorSpace toColorSpace);
void colorconvert(image::Image<image::RGBfColor>& image, image::EImageColorSpace fromColorSpace,
                  image::EImageColorSpace toColorSpace);
void colorconvert(image::Image<image::RGBAfColor>& image, image::EImageColorSpace fromColorSpace,
                  image::EImageColorSpace toColorSpace);
void colorconvert(oiio::ImageBuf& dst, const oiio::ImageBuf& src,
                  image::EImageColorSpace fromColorSpace, image::EImageColorSpace toColorSpace);

/**
 * @brief Resize a given image buffer.
 * @param[in] downscale The resize downscale
 * @param[in] inImage The input image buffer
 * @param[out] outImage The output image buffer
 * @param[in] filter The name of a high-quality filter to use when resampling
 *            Default is bilinear resampling
 *            See openImageIO documentation "ImageBufAlgo filtername"
 * @param[in] filterSize The resize filter size
 */
void resizeImage(int downscale, const image::Image<unsigned char>& inImage,
                 image::Image<unsigned char>& outImage,
                 const std::string& filter = "", float filterSize = 0);
void resizeImage(int downscale, const image::Image<float>& inImage,
                 image::Image<float>& outImage,
                 const std::string& filter = "", float filterSize = 0);
void resizeImage(int downscale, const image::Image<image::RGBColor>& inImage,
                 image::Image<image::RGBColor>& outImage,
                 const std::string& filter = "", float filterSize = 0);
void resizeImage(int downscale, const image::Image<image::RGBfColor>& inImage,
                 image::Image<image::RGBfColor>& outImage,
                 const std::string& filter = "", float filterSize = 0);
void resizeImage(int downscale, const image::Image<image::RGBAColor>& inImage,
                 image::Image<image::RGBAColor>& outImage,
                 const std::string& filter = "", float filterSize = 0);
void resizeImage(int downscale, const image::Image<image::RGBAfColor>& inImage,
                 image::Image<image::RGBAfColor>& outImage,
                 const std::string& filter = "", float filterSize = 0);

/**
 * @brief Resize a given image buffer in place.
 * @param[in] downscale The resize downscale
 * @param[in,out] inoutImage The input/output image buffer
 * @param[in] filter The name of a high-quality filter to use when resampling
 *            Default is bilinear resampling
 *            See openImageIO documentation "ImageBufAlgo filtername"
 * @param[in] filterSize The resize filter size
 */
void resizeImage(int downscale, image::Image<unsigned char>& inoutImage,
                 const std::string& filter = "", float filterSize = 0);
void resizeImage(int downscale, image::Image<float>& inoutImage,
                 const std::string& filter = "", float filterSize = 0);
void resizeImage(int downscale, image::Image<image::RGBColor>& inoutImage,
                 const std::string& filter = "", float filterSize = 0);
void resizeImage(int downscale, image::Image<image::RGBfColor>& inoutImage,
                 const std::string& filter = "", float filterSize = 0);
void resizeImage(int downscale, image::Image<image::RGBAColor>& inoutImage,
                 const std::string& filter = "", float filterSize = 0);
void resizeImage(int downscale, image::Image<image::RGBAfColor>& inoutImage,
                 const std::string& filter = "", float filterSize = 0);

/**
 * @brief convolve a given image buffer
 * @param[in] inBuffer The input image buffer
 * @param[out] outBuffer outBuffer The output image buffer
 * @param[in] kernel The kernel name, can be "gaussian", "sharp-gaussian", "box", ...
 *            Default is gaussian kernel
 *            See openImageIO documentation "ImageBufAlgo.make_kernel"
 * @param[in] kernelWidth The kernel width
 * @param[in] kernelHeight The kernal height
 */
void convolveImage(const image::Image<unsigned char>& inBuffer,
                   image::Image<unsigned char>& outBuffer,
                   const std::string& kernel = "gaussian",
                   float kernelWidth = 5.0f, float kernelHeight = 5.0f);

void convolveImage(const image::Image<rgb>& inBuffer, image::Image<rgb>& outBuffer,
                   const std::string& kernel = "gaussian",
                   float kernelWidth = 5.0f, float kernelHeight = 5.0f);

void convolveImage(const image::Image<float>& inBuffer, image::Image<float>& outBuffer,
                   const std::string& kernel = "gaussian",
                   float kernelWidth = 5.0f, float kernelHeight = 5.0f);

void convolveImage(const image::Image<image::RGBfColor>& inBuffer,
                   image::Image<image::RGBfColor>& outBuffer,
                   const std::string& kernel = "gaussian",
                   float kernelWidth = 5.0f, float kernelHeight = 5.0f);


/**
 * @brief fill holes in a given image buffer with plausible values
 * @param[in,out] colorBuffer The image buffer to fill
 * @param[in] alphaBuffer The input alpha buffer containing 0.0/1.0 for empty/valid pixels
 */
void fillHoles(image::Image<image::RGBfColor>& image, const std::vector<float>& alphaBuffer);

/**
* @brief Calculate the difference between images of different sizes
* @param [inImgDownscaled] the smaller image
* @param [outImg] the difference
* @param [downscale] the downscale coefficient between image sizes
*/
void imageDiff(const image::Image<image::RGBfColor>& inImg,
               const image::Image<image::RGBfColor>& inImgDownscaled,
               image::Image<image::RGBfColor>& outImg, unsigned int downscale);

/**
* @brief Calculate the laplacian pyramid of a given image,
*        ie. its decomposition in frequency bands
* @param [out_pyramidL] the laplacian pyramid
* @param [nbBand] the number of frequency bands
* @param [downscale] the downscale coefficient between floors of the pyramid
*/
void laplacianPyramid(std::vector<image::Image<image::RGBfColor>>& out_pyramidL,
                      const image::Image<image::RGBfColor>& image, int nbBand, unsigned int downscale);


}
}
