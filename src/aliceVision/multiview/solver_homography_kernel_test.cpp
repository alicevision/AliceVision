// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include <vector>
#include "aliceVision/multiview/solver_homography_kernel.hpp"
#include "testing/testing.h"

using namespace std;
using namespace aliceVision;

TEST(HomographyKernelTest, Fitting_Unnormalized) {
  // Define 3 knows homographies (Use as GT).
  vector<Mat3> H_gt(3);

  H_gt[0] = Mat3::Identity();
  H_gt[1] << 1,  0, -4,
             0,  1,  5,
             0,  0,  1;
  H_gt[2] << 1, -2,  3,
             4,  5, -6,
            -7,  8,  1;

  // Define a set of points.
  Mat x(2, 9), xh;
  x << 0, 0, 0, 1, 1, 1, 2, 2, 2,
       0, 1, 2, 0, 1, 2, 0, 1, 2;
  EuclideanToHomogeneous(x, &xh);

  for (int i = 0; i < H_gt.size(); ++i) {
    // Transform points by the ground truth homography.
    Mat y, yh = H_gt[i] * xh;
    HomogeneousToEuclidean(yh, &y);

    homography::kernel::UnnormalizedKernel kernel(x, y);

    size_t samples_[5]={0,1,2,3,4};
    vector<size_t> samples(samples_,samples_+5);
    for (size_t j = 4; samples.size() < x.cols(); samples.push_back(j++)) {
      vector<Mat3> Hs;
      kernel.Fit(samples, &Hs);
      CHECK_EQUAL(1, Hs.size());
      // Check that found matrix is equal to the GT
      EXPECT_MATRIX_PROP(H_gt[i], Hs[0], 1e-6);
    }
  }
}

TEST(HomographyKernelTest, Fitting_Normalized) {
  // Define 3 knows homographies (Use as GT).
  vector<Mat3> H_gt(3);

  H_gt[0] = Mat3::Identity();
  H_gt[1] << 1,  0, -4,
             0,  1,  5,
             0,  0,  1;
  H_gt[2] << 1, -2,  3,
             4,  5, -6,
            -7,  8,  1;

  // Define a set of points.
  Mat x(2, 9), xh;
  x << 0, 0, 0, 1, 1, 1, 2, 2, 2,
       0, 1, 2, 0, 1, 2, 0, 1, 2;
  EuclideanToHomogeneous(x, &xh);

  for (int i = 0; i < H_gt.size(); ++i) {
    // Transform points by the ground truth homography.
    Mat y, yh = H_gt[i] * xh;
    HomogeneousToEuclidean(yh, &y);

    homography::kernel::Kernel kernel(x, y);

    size_t samples_[5]={0,1,2,3,4};
    vector<size_t> samples(samples_,samples_+5);
    for (size_t j = 4; samples.size() < x.cols(); samples.push_back(j++)) {
      vector<Mat3> Hs;
      kernel.Fit(samples, &Hs);
      CHECK_EQUAL(1, Hs.size());
      // Check that found matrix is equal to the GT
      EXPECT_MATRIX_PROP(H_gt[i], Hs[0], 1e-6);
    }
  }
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
