#include <openMVG/multiview/solver_triangulate_nviews_LORansac.hpp>
#include <openMVG/robust_estimation/score_evaluator.hpp>
#include <testing/testing.h>

#include <vector>

using namespace openMVG;

// Test triangulation as algebric problem, it generates some random projection
// matrices, a random 3D points and its corresponding 2d image points. Some of these
// points are considered as outliers. Inliers are assigned a max weight, outliers
// a zero weight. Note: this is just an algebric test, ie points and projection
// matrices have no physical meaning (eg no notion of point in front of the camera
// is considered).
TEST(LORansacTriangulate, Algebric)
{
  const std::size_t numTrials = 100;
  for(std::size_t trial = 0; trial < numTrials; ++ trial)
  {
    const std::size_t numviews = 20;
    const std::size_t outliers = 8;
    const std::size_t inliers = numviews - outliers;

    // Collect random P matrices together.
    std::vector<Mat34> Ps(numviews);
    for(std::size_t j = 0; j < numviews; ++j)
    {
      Ps[j] = Mat34::Random();
    }

    // generate a random 3D point
    Vec4 pt3d(Vec3::Random().homogeneous());

    // project the 3D point and prepare weights
    const double w = 1e8;
    std::vector<double> weights(numviews, w);
    Mat2X pt2d(2, numviews);
    for(std::size_t j = 0; j < numviews; ++j)
    {
      if(j < numviews - outliers)
      {
        // project the 3D point
        pt2d.col(j) = (Ps[j] * pt3d).hnormalized();
      }
      else
      {
        // for the outliers just set them to some random value
        pt2d.col(j) = Vec2::Random();
        // set the weight to 0 for the outliers
        weights[j] = 0.0;
      }
    }

    const double threshold = 1e-5;
    std::vector<std::size_t> vec_inliers;
    using TriangulationKernel = multiview::kernel::LORansacTriangulationKernel<>;
    TriangulationKernel kernel(pt2d, Ps);
    robust::ScorerEvaluator<TriangulationKernel> scorer(threshold);
    Vec4 X = robust::LO_RANSAC(kernel, scorer, &vec_inliers);

    // check inliers are correct
    EXPECT_EQ(vec_inliers.size(), inliers);

    // Check the reprojection error is nearly zero for inliers.
    for (std::size_t j = 0; j < inliers; ++j)
    {
      const Vec2 x_reprojected = (Ps[j] * X).hnormalized();
      const double error = (x_reprojected - pt2d.col(j)).norm();
//      EXPECT_NEAR(error, 0.0, 1e-4);
      assert(error < 1e-5);
    }
  }
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
