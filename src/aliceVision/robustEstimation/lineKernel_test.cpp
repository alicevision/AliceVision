// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/robustEstimation/LineKernel.hpp"
#include "testing/testing.h"
#include <vector>
using namespace aliceVision;
using namespace aliceVision::robustEstimation;

// Since the line fitter isn't so simple, test it in isolation.
TEST(LineFitter, ItWorks) {

  Mat2X xy(2, 5);
  // y = 2x + 1
  xy << 1, 2, 3, 4,  5,
        3, 5, 7, 9, 11;
  std::vector<Vec2> models;
  LineKernel kernel(xy);
  std::vector<size_t> samples;
  for (size_t i = 0; i < xy.cols(); ++i) {
    samples.push_back(i);
  }
  kernel.Fit(samples, &models);
  CHECK_EQUAL(1, models.size());
  EXPECT_NEAR(2.0, models[0][1], 1e-9);
  EXPECT_NEAR(1.0, models[0][0], 1e-9);
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
