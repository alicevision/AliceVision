// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "convolution.hpp"

namespace aliceVision {
namespace image {

void separableConvolution2d(const RowMatrixXf& image,
                            const Eigen::Matrix<float, 1, Eigen::Dynamic>& kernelX,
                            const Eigen::Matrix<float, 1, Eigen::Dynamic>& kernelY,
                            RowMatrixXf* out)
{
    const int sigmaY = static_cast<int>(kernelY.cols());
    const int halfSigmaY = static_cast<int>(kernelY.cols()) / 2;

    // Convolving a vertical filter across rows is the same thing as transpose
    // multiply i.e. kernel_y^t * rows. This will give us the convoled value for
    // each row. However, care must be taken at the top and bottom borders.
    const Eigen::Matrix<float, 1, Eigen::Dynamic> reverseKernelY = kernelY.reverse();

#pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < halfSigmaY; i++)
    {
        const int forwardSize = i + halfSigmaY + 1;
        const int reverseSize = sigmaY - forwardSize;
        out->row(i) = kernelY.tail(forwardSize) * image.block(0, 0, forwardSize, image.cols()) +
                      reverseKernelY.tail(reverseSize) * image.block(1, 0, reverseSize, image.cols());

        // Apply the same technique for the end rows.
        out->row(image.rows() - i - 1) = kernelY.head(forwardSize) * image.block(image.rows() - forwardSize, 0, forwardSize, image.cols()) +
                                         reverseKernelY.head(reverseSize) * image.block(image.rows() - reverseSize - 1, 0, reverseSize, image.cols());
    }

// Applying the rest of the y filter.
#pragma omp parallel for schedule(dynamic)
    for (int row = halfSigmaY; row < image.rows() - halfSigmaY; row++)
    {
        out->row(row) = kernelY * image.block(row - halfSigmaY, 0, sigmaY, out->cols());
    }

    const int sigmaX = static_cast<int>(kernelX.cols());
    const int halfSigmaX = static_cast<int>(kernelX.cols() / 2);

    // Convolving with the horizontal filter is easy. Rather than using the kernel
    // as a sliding window, we use the row pixels as a sliding window around the
    // filter. We prepend and append the proper border values so that we are sure
    // to end up with the correct convolved values.
    Eigen::RowVectorXf tempRow(image.cols() + sigmaX - 1);

#pragma omp parallel for firstprivate(tempRow), schedule(dynamic)
    for (int row = 0; row < out->rows(); row++)
    {
        tempRow.head(halfSigmaX) = out->row(row).segment(1, halfSigmaX).reverse();
        tempRow.segment(halfSigmaX, image.cols()) = out->row(row);
        tempRow.tail(halfSigmaX) = out->row(row).segment(image.cols() - 2 - halfSigmaX, halfSigmaX).reverse();

        // Convolve the row. We perform the first step here explicitly so that we
        // avoid setting the row equal to zero.
        out->row(row) = kernelX(0) * tempRow.head(image.cols());
        for (int i = 1; i < sigmaX; i++)
        {
            out->row(row) += kernelX(i) * tempRow.segment(i, image.cols());
        }
    }
}

}  // namespace image
}  // namespace aliceVision
