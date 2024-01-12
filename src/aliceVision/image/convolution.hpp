// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/numeric/Accumulator.hpp>
#include <aliceVision/image/convolutionBase.hpp>
#include <aliceVision/image/Image.hpp>
#include <aliceVision/config.hpp>

#include <vector>
#include <cassert>

/**
 ** @file Standard 2D image convolution functions :
 ** - vertical
 ** - horizontal
 ** - 2D (using standard 2d kernel of with separable kernels)
 **/

namespace aliceVision {
namespace image {

/**
 ** General image convolution by a kernel
 ** assume kernel has odd size in both dimensions and (border pixel are copied)
 ** @param img source image
 ** @param kernel convolution kernel
 ** @param out resulting image
 **/
template<typename Image>
void imageConvolution(const Image& img, const Mat& kernel, Image& out)
{
    const int kernelWidth = kernel.cols();
    const int kernelHeight = kernel.rows();

    assert(kernelWidth % 2 != 0 && kernelHeight % 2 != 0);

    typedef typename Image::Tpixel pix_t;
    typedef typename Accumulator<pix_t>::Type acc_pix_t;

    out.resize(img.width(), img.height());

    for (int row = 0; row < img.rows(); ++row)
    {
        for (int col = 0; col < img.cols(); ++col)
        {
            acc_pix_t sum = acc_pix_t();

            for (int i = 0; i < kernelHeight; ++i)
            {
                for (int j = 0; j < kernelWidth; ++j)
                {
                    int idy = row + i - kernelHeight / 2;
                    int idx = col + j - kernelWidth / 2;

                    // Get correct value
                    idx = idx < 0 ? 0 : (idx >= img.cols() ? img.cols() - 1 : idx);
                    idy = idy < 0 ? 0 : (idy >= img.rows() ? img.rows() - 1 : idy);

                    sum += kernel(i, j) * img(idy, idx);
                }
            }
            out(row, col) = sum;
        }
    }
}

/**
 ** Horizontal (1d) convolution
 ** assume kernel has odd size
 ** @param img Input image
 ** @param kernel convolution kernel
 ** @param out Output image
 **/
template<typename ImageTypeIn, typename ImageTypeOut, typename Kernel>
void imageHorizontalConvolution(const ImageTypeIn& img, const Kernel& kernel, ImageTypeOut& out)
{
    typedef typename ImageTypeIn::Tpixel pix_t;

    const int rows(img.rows());
    const int cols(img.cols());

    out.resize(cols, rows);

    const int kernelWidth = kernel.size();
    const int halfKernelWidth = kernelWidth / 2;

    std::vector<pix_t> line(cols + kernelWidth);

    for (int row = 0; row < rows; ++row)
    {
        // Copy line
        const pix_t startPix = img.coeffRef(row, 0);
        for (int k = 0; k < halfKernelWidth; ++k)  // pad before
        {
            line[k] = startPix;
        }
        memcpy(&line[0] + halfKernelWidth, img.data() + row * cols, sizeof(pix_t) * cols);
        const pix_t endPix = img.coeffRef(row, cols - 1);
        for (int k = 0; k < halfKernelWidth; ++k)  // pad after
        {
            line[k + halfKernelWidth + cols] = endPix;
        }

        // Apply convolution
        convBuffer(&line[0], kernel.data(), cols, kernelWidth);

        memcpy(out.data() + row * cols, &line[0], sizeof(pix_t) * cols);
    }
}

/**
 ** Vertical (1d) convolution
 ** assume kernel has odd size
 ** @param img Input image
 ** @param kernel convolution kernel
 ** @param out Output image
 **/
template<typename ImageTypeIn, typename ImageTypeOut, typename Kernel>
void imageVerticalConvolution(const ImageTypeIn& img, const Kernel& kernel, ImageTypeOut& out)
{
    typedef typename ImageTypeIn::Tpixel pix_t;

    const int kernelWidth = kernel.size();
    const int halfKernelWidth = kernelWidth / 2;

    const int rows = img.rows();
    const int cols = img.cols();

    out.resize(cols, rows);

    std::vector<pix_t> line(rows + kernelWidth);

    for (int col = 0; col < cols; ++col)
    {
        // Copy column
        for (int k = 0; k < halfKernelWidth; ++k)
        {
            line[k] = img.coeffRef(0, col);
        }
        for (int k = 0; k < rows; ++k)
        {
            line[k + halfKernelWidth] = img.coeffRef(k, col);
        }
        for (int k = 0; k < halfKernelWidth; ++k)
        {
            line[k + halfKernelWidth + rows] = img.coeffRef(rows - 1, col);
        }

        // Apply convolution
        convBuffer(&line[0], kernel.data(), rows, kernelWidth);

        for (int row = 0; row < rows; ++row)
        {
            out.coeffRef(row, col) = line[row];
        }
    }
}

/**
 ** Separable 2D convolution
 ** (nxm kernel is replaced by two 1D convolution of (size n then size m) )
 ** @param img source image
 ** @param horizK horizontal kernel
 ** @param vertK vertical kernel
 ** @param out output image
 **/
template<typename ImageType, typename Kernel>
void imageSeparableConvolution(const ImageType& img, const Kernel& horizK, const Kernel& vertK, ImageType& out)
{
    // Cast the Kernel to the appropriate type
    typedef typename ImageType::Tpixel pix_t;
    typedef Eigen::Matrix<typename Accumulator<pix_t>::Type, Eigen::Dynamic, 1> VecKernel;
    const VecKernel horizKCast = horizK.template cast<typename Accumulator<pix_t>::Type>();
    const VecKernel vertKCast = vertK.template cast<typename Accumulator<pix_t>::Type>();

    ImageType tmp;
    imageHorizontalConvolution(img, horizKCast, tmp);
    imageVerticalConvolution(tmp, vertKCast, out);
}

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMatrixXf;

/// Specialization for Float based image (for arbitrary sized kernel)
void separableConvolution2d(const RowMatrixXf& image,
                            const Eigen::Matrix<float, 1, Eigen::Dynamic>& kernelX,
                            const Eigen::Matrix<float, 1, Eigen::Dynamic>& kernelY,
                            RowMatrixXf* out);

// Specialization for Image<float> in order to use separableConvolution2d
template<typename Kernel>
void imageSeparableConvolution(const Image<float>& img, const Kernel& horizK, const Kernel& vertK, Image<float>& out)
{
    // Cast the Kernel to the appropriate type
    typedef Image<float>::Tpixel pix_t;
    typedef Eigen::Matrix<typename aliceVision::Accumulator<pix_t>::Type, Eigen::Dynamic, 1> VecKernel;
    const VecKernel horizKCast = horizK.template cast<typename aliceVision::Accumulator<pix_t>::Type>();
    const VecKernel vertKCast = vertK.template cast<typename aliceVision::Accumulator<pix_t>::Type>();

    out.resize(img.width(), img.height());
    separableConvolution2d(img.getMat(), horizKCast, vertKCast, &((Image<float>::Base&)out));
}

}  // namespace image
}  // namespace aliceVision
