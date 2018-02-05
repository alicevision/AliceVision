// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "convolution.hpp"

namespace aliceVision {
namespace image {

void SeparableConvolution2d(const RowMatrixXf& image,
                            const Eigen::Matrix<float, 1, Eigen::Dynamic>& kernel_x,
                            const Eigen::Matrix<float, 1, Eigen::Dynamic>& kernel_y,
                            RowMatrixXf* out) {
  const int sigma_y = static_cast<int>( kernel_y.cols() );
  const int half_sigma_y = static_cast<int>( kernel_y.cols() ) / 2;

  // Convolving a vertical filter across rows is the same thing as transpose
  // multiply i.e. kernel_y^t * rows. This will give us the convoled value for
  // each row. However, care must be taken at the top and bottom borders.
  const Eigen::Matrix<float, 1, Eigen::Dynamic> reverse_kernel_y = kernel_y.reverse();

  #pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < half_sigma_y; i++)
  {
    const int forward_size = i + half_sigma_y + 1;
    const int reverse_size = sigma_y - forward_size;
    out->row(i) = kernel_y.tail(forward_size) *
                  image.block(0, 0, forward_size, image.cols()) +
                  reverse_kernel_y.tail(reverse_size) *
                  image.block(1, 0, reverse_size, image.cols());

    // Apply the same technique for the end rows.
    out->row(image.rows() - i - 1) =
      kernel_y.head(forward_size) *
      image.block(image.rows() - forward_size, 0, forward_size, image.cols())
      +
      reverse_kernel_y.head(reverse_size) *
      image.block(image.rows() - reverse_size - 1, 0, reverse_size, image.cols());
  }

  // Applying the rest of the y filter.
  #pragma omp parallel for schedule(dynamic)
  for (int row = half_sigma_y; row < image.rows() - half_sigma_y; row++)
  {
    out->row(row) =  kernel_y * image.block(row - half_sigma_y, 0, sigma_y, out->cols());
  }

  const int sigma_x = static_cast<int>(kernel_x.cols());
  const int half_sigma_x = static_cast<int>(kernel_x.cols() / 2);

  // Convolving with the horizontal filter is easy. Rather than using the kernel
  // as a sliding window, we use the row pixels as a sliding window around the
  // filter. We prepend and append the proper border values so that we are sure
  // to end up with the correct convolved values.
  Eigen::RowVectorXf temp_row(image.cols() + sigma_x - 1);

  #pragma omp parallel for firstprivate(temp_row), schedule(dynamic)
  for (int row = 0; row < out->rows(); row++)
  {
    temp_row.head(half_sigma_x) =
      out->row(row).segment(1, half_sigma_x).reverse();
    temp_row.segment(half_sigma_x, image.cols()) = out->row(row);
    temp_row.tail(half_sigma_x) =
      out->row(row)
      .segment(image.cols() - 2 - half_sigma_x, half_sigma_x)
      .reverse();

    // Convolve the row. We perform the first step here explicitly so that we
    // avoid setting the row equal to zero.
    out->row(row) = kernel_x(0) * temp_row.head(image.cols());
    for (int i = 1; i < sigma_x; i++) {
      out->row(row) += kernel_x(i) * temp_row.segment(i, image.cols());
    }
  }
}

} // namespace image
} // namespace aliceVision
