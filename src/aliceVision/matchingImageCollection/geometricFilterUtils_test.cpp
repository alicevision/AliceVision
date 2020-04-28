// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/matchingImageCollection/geometricFilterUtils.hpp>

#define BOOST_TEST_MODULE matchingImageCollectionPairBuilder

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace std;
using namespace aliceVision;

void meanAndStd(const Eigen::Matrix2Xf& points2d, Vec2f& mean, Vec2f& stdDev)
{
  mean = points2d.rowwise().mean();
  const auto nbPoints = points2d.cols();
  stdDev = ((points2d.colwise() - mean).cwiseAbs2().rowwise().sum() / (nbPoints - 1)).cwiseSqrt();
}

BOOST_AUTO_TEST_CASE(matchingImageCollection_centerMatrix)
{
  const std::size_t numPoints{100};
  const std::size_t numTrials{100};
  const float threshold{0.0001f};
  for(std::size_t i = 0; i < numTrials; ++i)
  {
    Eigen::Matrix2Xf points(2, numPoints);
    const Vec2f coeff = 100*Vec2f::Random();
    points = ((coeff(0) * points.setRandom()).array() + coeff(1)).matrix();

    Mat3 transformation;
    matchingImageCollection::centerMatrix(points, transformation);

    Eigen::Matrix2Xf points2d = (transformation.cast<float>() * points.colwise().homogeneous()).colwise().hnormalized();
    Vec2f mean;
    Vec2f stdDev;
    meanAndStd(points2d, mean, stdDev);
    std::cout << "points2d\n" << points2d << std::endl;
//    std::cout << "Mean " << mean << "\nStd dev " << stdDev << std::endl;
//    std::cout << "Mean " << mean.isZero() << "\nStd dev " << stdDev.isOnes() << std::endl;
    BOOST_CHECK(mean.isZero(threshold));
    BOOST_CHECK(stdDev.isOnes());
  }
}

BOOST_AUTO_TEST_CASE(matchingImageCollection_similarityEstimation)
{
  // same point should give the identity matrix
  const feature::PointFeature feat1 {1.0f, 5.0f, 1.0f, 0.1f};
  Mat3 S;
  matchingImageCollection::computeSimilarity(feat1, feat1, S);

  // up to eigen's default tolerance
  BOOST_CHECK(S.isIdentity());

  const std::size_t numTrials{100};
  for(std::size_t i = 0; i < numTrials; ++i)
  {
    // generate a random point with a random scale and orientation
    const Vec2f point = Vec2f::Random();
    const Vec2f param = Vec2f::Random();
    const feature::PointFeature feat2 {point(0), point(1), param(0), param(1)};

    // estimate the similarity
    matchingImageCollection::computeSimilarity(feat1, feat2, S);

    const Vec3 ptIp_hom = S * feat1.coords().cast<double>().homogeneous();
    const Vec2 result = point.cast<double>() - ptIp_hom.hnormalized();

    // up to eigen's default tolerance
    BOOST_CHECK(result.isZero());
//    std::cout << result.isZero() << std::endl;
//    std::cout <<  point.cast<double>() - ptIp_hom.hnormalized() << std::endl;
  }
}
