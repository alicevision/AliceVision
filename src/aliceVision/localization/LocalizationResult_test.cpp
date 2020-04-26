// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "LocalizationResult.hpp"
#include <aliceVision/camera/PinholeRadial.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/sfm/pipeline/localization/SfMLocalizer.hpp>

#include <boost/filesystem.hpp>

#include <vector>
#include <chrono>
#include <random>

#define BOOST_TEST_MODULE LocalizationResult

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

namespace fs = boost::filesystem;
using namespace aliceVision;

sfm::ImageLocalizerMatchData generateRandomMatch_Data(std::size_t numPts)
{
  sfm::ImageLocalizerMatchData data;
  data.projection_matrix = Mat34::Random();
  data.pt3D = Mat::Random(3, numPts);
  data.pt2D = Mat::Random(2, numPts);
  const std::size_t numInliers = (std::size_t) numPts*0.7;
  for(std::size_t i = 0; i < numInliers; ++i)
  {
    data.vec_inliers.push_back(i);
  }
  return data;
}

localization::LocalizationResult generateRandomResult(std::size_t numPts)
{
  // random matchData
  const sfm::ImageLocalizerMatchData &data = generateRandomMatch_Data(numPts);
  
  // random indMatch3D2D
  std::vector<localization::IndMatch3D2D> indMatch3D2D;
  indMatch3D2D.reserve(numPts);
  for(std::size_t i = 0; i < numPts; ++i)
  {
    indMatch3D2D.emplace_back(i, feature::EImageDescriberType::UNKNOWN, i);
  }
  
  // random pose
  geometry::Pose3 pose = geometry::Pose3(Mat3::Random(), Vec3::Random());
  
  // random intrinsics
  camera::PinholeRadialK3 intrinsics = camera::PinholeRadialK3(640, 480, 1400, 320.5, 240.5, 0.001, -0.05, 0.00003);
  
  // random valid
  const bool valid = (numPts % 2 == 0);

  std::vector<voctree::DocMatch> matchedImages;
  matchedImages.push_back(voctree::DocMatch(2, 0.5));
  matchedImages.push_back(voctree::DocMatch(3, 0.8));

  return localization::LocalizationResult(data, indMatch3D2D, pose, intrinsics, matchedImages, valid);
}

BOOST_AUTO_TEST_CASE(LocalizationResult_LoadSaveVector)
{
  const double threshold = 1e-10;
  const std::size_t numResults = 10;
  const std::string filename = "test_localizationResults.json";
  const unsigned seed1 = std::chrono::system_clock::now().time_since_epoch().count();
  std::random_device rd;
  std::mt19937 gen(seed1);
  std::uniform_int_distribution<> numpts(1, 20);

  std::vector<localization::LocalizationResult> resGT;
  std::vector<localization::LocalizationResult> resCheck;
  resGT.reserve(numResults);
  resCheck.reserve(numResults);

  for(std::size_t i = 0; i < numResults; ++i)
  {
    resGT.push_back(generateRandomResult(numpts(gen)));
  }

  BOOST_CHECK_NO_THROW(localization::LocalizationResult::save(resGT, filename));
  BOOST_CHECK_NO_THROW(localization::LocalizationResult::load(resCheck, filename));
  BOOST_CHECK(resCheck.size() == resGT.size());

  // check each element
  for(std::size_t i = 0; i < numResults; ++i)
  {
    const auto res = resGT[i];
    const auto check = resCheck[i];

    // same validity
    BOOST_CHECK(res.isValid() == check.isValid());

    // same pose
    const Mat3 rotGT = res.getPose().rotation();
    const Mat3 rot = check.getPose().rotation();
    for(std::size_t i = 0; i < 3; ++i)
    {
      for(std::size_t j = 0; j < 3; ++j)
      {
        BOOST_CHECK_SMALL(rotGT(i, j)- rot(i, j), threshold);
      }
    }
    const Vec3 centerGT = res.getPose().center();
    const Vec3 center = check.getPose().center();
    BOOST_CHECK_SMALL(centerGT(0)- center(0), threshold);
    BOOST_CHECK_SMALL(centerGT(1)- center(1), threshold);
    BOOST_CHECK_SMALL(centerGT(2)- center(2), threshold);

    // same _indMatch3D2D
    const auto idxGT = res.getIndMatch3D2D();
    const auto idx = check.getIndMatch3D2D();
    BOOST_CHECK(idxGT.size() == idx.size());
    const std::size_t numpts = idxGT.size();
    for(std::size_t j = 0; j < numpts; ++j)
    {
      BOOST_CHECK(idxGT[j].landmarkId == idx[j].landmarkId);
      BOOST_CHECK(idxGT[j].featId == idx[j].featId);
    }

    // same _matchData
    BOOST_CHECK(res.getInliers().size() == check.getInliers().size());
    const auto inliersGT = res.getInliers();
    const auto inliers = check.getInliers();
    for(std::size_t j = 0; j < res.getInliers().size(); ++j)
    {
      BOOST_CHECK(inliersGT[j] == inliers[j]);
    }

    EXPECT_MATRIX_NEAR(res.getPt3D(), check.getPt3D(), threshold);
    EXPECT_MATRIX_NEAR(res.getPt2D(), check.getPt2D(), threshold);
    EXPECT_MATRIX_NEAR(res.getProjection(), check.getProjection(), threshold);
    
    // same matchedImages
    BOOST_CHECK(res.getMatchedImages().size() == check.getMatchedImages().size());
    const auto matchedImagesGT = res.getMatchedImages();
    const auto matchedImages = check.getMatchedImages();
    for(std::size_t j = 0; j < res.getMatchedImages().size(); ++j)
    {
      BOOST_CHECK(matchedImagesGT[j] == matchedImages[j]);
    }

    fs::remove(filename);
  }
}
