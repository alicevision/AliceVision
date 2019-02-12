// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/lightingEstimation/lightingEstimation.hpp>

#include <cmath>
#include <cstdio>
#include <iostream>

#define BOOST_TEST_MODULE LIGHTING_ESTIMATION

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

#include <aliceVision/unitTest.hpp>

using namespace Eigen;
using namespace aliceVision;
using namespace aliceVision::lightingEstimation;
using namespace aliceVision::image;

inline float zeroOneRand()
{
  return float(rand()) / float(RAND_MAX);
}


BOOST_AUTO_TEST_CASE(LIGHTING_ESTIMATION_Lambertian_GT)
{
  const std::size_t sx = 4;
  const std::size_t sy = 3;

  // Random initialization of lighting, albedo and normals
  LightingVector lightingSynt = MatrixXf::Random(9, 3).cwiseAbs();

  Image<RGBfColor> albedoSynt(sy, sx);
  Image<RGBfColor> normalsSynt(sy, sx);

  for(std::size_t y = 0; y < sy; ++y)
  {
    for(std::size_t x = 0; x < sx; ++x)
    {
      albedoSynt(x, y) = RGBfColor(zeroOneRand(), zeroOneRand(), zeroOneRand());
      RGBfColor n(zeroOneRand(), zeroOneRand(), std::abs(zeroOneRand()));
      n.normalize();
      normalsSynt(x, y) = n;
    }
  }

  Image<AugmentedNormal> agmNormalsSynt(normalsSynt.cast<AugmentedNormal>());

  // Create simulated image
  Image<RGBfColor> pictureGenerated(sy, sx);

  for(std::size_t y = 0; y < sy; ++y)
  {
    for(std::size_t x = 0; x < sx; ++x)
    {
      for(std::size_t ch = 0; ch < 3; ++ch)
      {
        pictureGenerated(x,y)(ch) = albedoSynt(x,y)(ch) * agmNormalsSynt(x,y).dot(lightingSynt.col(ch));
      }
    }
  }

  // Retrieve unknown lighting
  LighthingEstimator estimator;
  LightingVector lightingEst;

  estimator.addImage(albedoSynt, pictureGenerated, normalsSynt);
  estimator.estimateLigthing(lightingEst);

  const float epsilon = 1e-3f;
  EXPECT_MATRIX_NEAR(lightingEst, lightingSynt, epsilon);
}


BOOST_AUTO_TEST_CASE(LIGHTING_ESTIMATION_Lambertian_noise)
{
  const std::size_t sx = 500;
  const std::size_t sy = 300;

  // Random initialization of lighting, albedo and normals
  LightingVector lightingSynt = MatrixXf::Random(9, 3).cwiseAbs();

  Image<RGBfColor> albedoSynt(sy, sx);
  Image<RGBfColor> normalsSynt(sy, sx);

  for(std::size_t y = 0; y < sy; ++y)
  {
    for(std::size_t x = 0; x < sx; ++x)
    {
      albedoSynt(x, y) = RGBfColor(zeroOneRand(), zeroOneRand(), zeroOneRand());
      RGBfColor n(zeroOneRand(), zeroOneRand(), std::abs(zeroOneRand()));
      n.normalize();
      normalsSynt(x, y) = n;
    }
  }

  Image<AugmentedNormal> agmNormalsSynt(normalsSynt.cast<AugmentedNormal>());

  const float noiseLevel = 0.01f;

  // Create simulated image
  Image<RGBfColor> pictureGenerated(sy, sx);

  for(std::size_t y = 0; y < sy; ++y)
  {
    for(std::size_t x = 0; x < sx; ++x)
    {
      for(std::size_t ch = 0; ch < 3; ++ch)
      {
        pictureGenerated(x,y)(ch) = albedoSynt(x,y)(ch) * agmNormalsSynt(x,y).dot(lightingSynt.col(ch));
        pictureGenerated(x,y)(ch) += zeroOneRand() * noiseLevel;
      }
    }
  }

  // Retrieve unknown lighting
  LighthingEstimator estimator;
  LightingVector lightingEst;

  estimator.addImage(albedoSynt, pictureGenerated, normalsSynt);
  estimator.estimateLigthing(lightingEst);

  const float epsilon = 1e-2f;
  EXPECT_MATRIX_NEAR(lightingEst, lightingSynt, epsilon);
}




