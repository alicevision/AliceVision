// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "conditioning.hpp"

namespace aliceVision {
namespace robustEstimation {

// HZ 4.4.4 pag.109
void preconditionerFromPoints(const Mat &points, Mat3 *T) {
  Vec mean, variance;
  MeanAndVarianceAlongRows(points, &mean, &variance);

  double xfactor = sqrt(2.0 / variance(0));
  double yfactor = sqrt(2.0 / variance(1));

  // If variance is equal to 0.0 set scaling factor to identity.
  // -> Else it will provide nan value (because division by 0).
  if (variance(0) < 1e-8)
    xfactor = mean(0) = 1.0;
  if (variance(1) < 1e-8)
    yfactor = mean(1) = 1.0;

  *T << xfactor, 0,       -xfactor * mean(0),
        0,       yfactor, -yfactor * mean(1),
        0,       0,        1;
}

void preconditionerFromImageSize(int width, int height, Mat3 *T) {
  // Build the normalization matrix
  double dNorm = 1.0 / sqrt( static_cast<double>(width*height) );

  (*T) = Mat3::Identity();
  (*T)(0,0) = (*T)(1,1) = dNorm;
  (*T)(0,2) = -.5f*width*dNorm;
  (*T)(1,2) = -.5*height*dNorm;
}

void applyTransformationToPoints(const Mat &points,
                                 const Mat3 &T,
                                 Mat *transformed_points)
{
  const Mat::Index n = points.cols();
  transformed_points->resize(2,n);
  for (Mat::Index i = 0; i < n; ++i) {
    Vec3 in, out;
    in << points(0, i), points(1, i), 1;
    out = T * in;
    (*transformed_points)(0, i) = out(0)/out(2);
    (*transformed_points)(1, i) = out(1)/out(2);
  }
}

void normalizePointsFromImageSize(const Mat &points,
                      Mat *normalized_points,
                      Mat3 *T,
                      int width,
                      int height)
{
  preconditionerFromImageSize(width, height, T);
  applyTransformationToPoints(points, *T, normalized_points);
}


void normalizePoints(const Mat &points,
                     Mat *normalized_points,
                     Mat3 *T)
{
  preconditionerFromPoints(points, T);
  applyTransformationToPoints(points, *T, normalized_points);
}

} // namespace robustEstimation
} // namespace aliceVision
