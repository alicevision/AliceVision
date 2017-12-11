// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <string>
#include <vector>

class rgb;
class Color;

namespace imageIO {

/**
 * @brief read image dimension from a given path
 * @param[in] path The given path to the image
 * @param[out] width The image width
 * @param[out] height The image height
 * @param[out] nchannels The image channel number
 */
void readImageSpec(const std::string& path, int& width, int& height, int& nchannels);

/**
 * @brief read an image with a given path and buffer
 * @param[in] path The given path to the image
 * @param[out] width The output image width
 * @param[out] height The output image height
 * @param[out] buffer The output image buffer
 */
void readImage(const std::string& path, int& width, int& height, std::vector<unsigned char>& buffer);
void readImage(const std::string& path, int& width, int& height, std::vector<unsigned short>& buffer);
void readImage(const std::string& path, int& width, int& height, std::vector<rgb>& buffer);
void readImage(const std::string& path, int& width, int& height, std::vector<float>& buffer);
void readImage(const std::string& path, int& width, int& height, std::vector<Color>& buffer);

/**
 * @brief write an image with a given path and buffer
 * @param[in] path The given path to the image
 * @param[in] width The input image width
 * @param[in] height The input image height
 * @param[in] buffer The input image buffer
 */
void writeImage(const std::string& path, int width, int height, std::vector<unsigned char>& buffer);
void writeImage(const std::string& path, int width, int height, std::vector<unsigned short>& buffer);
void writeImage(const std::string& path, int width, int height, std::vector<rgb>& buffer);
void writeImage(const std::string& path, int width, int height, std::vector<float>& buffer);
void writeImage(const std::string& path, int width, int height, std::vector<Color>& buffer);

/**
 * @brief transpose a given image buffer
 * @param[in] width The image buffer width
 * @param[in] height The image buffer height
 * @param[in,out] buffer The image buffer
 */
void transposeImage(int width, int height, std::vector<unsigned char>& inBuffer);
void transposeImage(int width, int height, std::vector<rgb>& inBuffer);
void transposeImage(int width, int height, std::vector<float>& inBuffer);
void transposeImage(int width, int height, std::vector<Color>& inBuffer);

/**
 * @brief resize a given image buffer
 * @param[in] inWidth The input image buffer width
 * @param[in] inHeight The input image buffer height
 * @param[in] downscale The resize downscale
 * @param[in] inBuffer The input image buffer
 * @param[out] outBuffer The output image buffer
 * @param[in] filter The name of a high-quality filter to use when resampling
 *            Default is bilinear resampling
 *            See openImageIO documentation "ImageBufAlgo filtername"
 * @param[in] filterSize The resize filter size
 */
void resizeImage(int inWidth, int inHeight, int downscale, std::vector<unsigned char>& inBuffer, std::vector<unsigned char>& outBuffer, const std::string& filter = "", float filterSize = 0);
void resizeImage(int inWidth, int inHeight, int downscale, std::vector<rgb>& inBuffer, std::vector<rgb>& outBuffer, const std::string& filter = "", float filterSize = 0);
void resizeImage(int inWidth, int inHeight, int downscale, std::vector<float>& inBuffer, std::vector<float>& outBuffer, const std::string& filter = "", float filterSize = 0);
void resizeImage(int inWidth, int inHeight, int downscale, std::vector<Color>& inBuffer, std::vector<Color>& outBuffer, const std::string& filter = "", float filterSize = 0);

/**
 * @brief convolve a given image buffer
 * @param[in] inWidth The input image buffer width
 * @param[in] inHeight The input image buffer heightt
 * @param[in] inBuffer The input image buffer
 * @param[out] outBuffer outBuffer The output image buffer
 * @param[in] kernel The kernel name, can be "gaussian", "sharp-gaussian", "box", ...
 *            Default is gaussian kernel
 *            See openImageIO documentation "ImageBufAlgo.make_kernel"
 * @param[in] kernelWidth The kernel width
 * @param[in] kernelHeight The kernal height
 */
void convolveImage(int inWidth, int inHeight, std::vector<unsigned char>& inBuffer, std::vector<unsigned char>& outBuffer, const std::string& kernel = "gaussian", float kernelWidth = 5.0f, float kernelHeight = 5.0f);
void convolveImage(int inWidth, int inHeight, std::vector<rgb>& inBuffer, std::vector<rgb>& outBuffer, const std::string& kernel = "gaussian", float kernelWidth = 5.0f, float kernelHeight = 5.0f);
void convolveImage(int inWidth, int inHeight, std::vector<float>& inBuffer, std::vector<float>& outBuffer, const std::string& kernel = "gaussian", float kernelWidth = 5.0f, float kernelHeight = 5.0f);
void convolveImage(int inWidth, int inHeight, std::vector<Color>& inBuffer, std::vector<Color>& outBuffer, const std::string& kernel = "gaussian", float kernelWidth = 5.0f, float kernelHeight = 5.0f);


} // namespace imageIO
