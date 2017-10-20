// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include <aliceVision/sfm/sfm.hpp>

#include <dependencies/stlplus3/filesystemSimplified/file_system.hpp>

#include <sstream>

#define BOOST_TEST_MODULE sfmDataUtils
#include <boost/test/included/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::sfm;

// Test a border case (empty scene)
BOOST_AUTO_TEST_CASE(SfMData_IntrinsicGrouping_Empty)
{
  SfMData sfmData;
  GroupSharedIntrinsics(sfmData);
}

// Two similar intrinsics object must be grouped as one
BOOST_AUTO_TEST_CASE(SfMData_IntrinsicGrouping_Grouping_One)
{
  SfMData sfmData;
  // One view, one intrinsic
  sfmData.views[0] = std::make_shared<View>("", 0, 0, 0);
  sfmData.intrinsics[0] = std::make_shared<Pinhole>(0);
  // One view, one intrinsic
  sfmData.views[1] = std::make_shared<View>("", 1, 1, 1);
  sfmData.intrinsics[1] = std::make_shared<Pinhole>(0);

  BOOST_CHECK_EQUAL(2, sfmData.intrinsics.size());
  GroupSharedIntrinsics(sfmData);
  BOOST_CHECK_EQUAL(1, sfmData.intrinsics.size());
  BOOST_CHECK_EQUAL(0, sfmData.views[0]->getIntrinsicId());
  BOOST_CHECK_EQUAL(0, sfmData.views[1]->getIntrinsicId());
}

// Unique intrinsics object must not be grouped
BOOST_AUTO_TEST_CASE(SfMData_IntrinsicGrouping_No_Grouping)
{
  SfMData sfmData;
  const int nbView = 10;
  for (int i = 0; i < nbView; ++i)
  {
    // Add one view, one intrinsic
    sfmData.views[i] = std::make_shared<View>("", i, i, i);
    sfmData.intrinsics[i] = std::make_shared<Pinhole>(i);
  }

  BOOST_CHECK_EQUAL(nbView, sfmData.intrinsics.size());
  GroupSharedIntrinsics(sfmData);
  BOOST_CHECK_EQUAL(nbView, sfmData.intrinsics.size());
}

// Similar intrinsics object must be grouped as one
BOOST_AUTO_TEST_CASE(SfMData_IntrinsicGrouping_Grouping_Two)
{
  SfMData sfmData;
  // Define separate intrinsics that share common properties
  // first block of intrinsics
  {
    sfmData.views[0] = std::make_shared<View>("", 0, 0, 0);
    sfmData.intrinsics[0] = std::make_shared<Pinhole>(0);
    sfmData.views[1] = std::make_shared<View>("", 1, 1, 1);
    sfmData.intrinsics[1] = std::make_shared<Pinhole>(0);
  }
  // second block of intrinsics
  {
    sfmData.views[2] = std::make_shared<View>("", 2, 2, 2);
    sfmData.intrinsics[2] = std::make_shared<Pinhole>(1);
    sfmData.views[3] = std::make_shared<View>("", 3, 3, 3);
    sfmData.intrinsics[3] = std::make_shared<Pinhole>(1);
  }

  BOOST_CHECK_EQUAL(4, sfmData.intrinsics.size());
  GroupSharedIntrinsics(sfmData);
  // Sort view Id from their intrinsic id to check that two view are in each intrinsic group
  std::map<IndexT, std::set<IndexT> > map_viewCount_per_intrinsic_id;
  for (const auto & val : sfmData.GetViews())
  {
    map_viewCount_per_intrinsic_id[val.second->getIntrinsicId()].insert(val.second->getViewId());
  }
  // Check that two view Id are linked to 0 and 1
  BOOST_CHECK_EQUAL(2, map_viewCount_per_intrinsic_id.size());
  BOOST_CHECK_EQUAL(1, map_viewCount_per_intrinsic_id.count(0));
  BOOST_CHECK_EQUAL(2, map_viewCount_per_intrinsic_id[0].size());
  BOOST_CHECK_EQUAL(1, map_viewCount_per_intrinsic_id.count(1));
  BOOST_CHECK_EQUAL(2, map_viewCount_per_intrinsic_id[1].size());
}

// Similar intrinsics object must be grouped as one (test with various camera type)
BOOST_AUTO_TEST_CASE(SfMData_IntrinsicGrouping_Grouping_Two_Different_Camera_Type)
{
  SfMData sfmData;
  // Define separate intrinsics that share common properties
  // first block of intrinsics
  {
    sfmData.views[0] = std::make_shared<View>("", 0, 0, 0);
    sfmData.intrinsics[0] = std::make_shared<Pinhole>(0);
    sfmData.views[1] = std::make_shared<View>("", 1, 1, 1);
    sfmData.intrinsics[1] = std::make_shared<Pinhole>(0);
  }
  // second block of intrinsics (different type)
  {
    sfmData.views[2] = std::make_shared<View>("", 2, 2, 2);
    sfmData.intrinsics[2] = std::make_shared<PinholeRadialK1>(0);
    sfmData.views[3] = std::make_shared<View>("", 3, 3, 3);
    sfmData.intrinsics[3] = std::make_shared<PinholeRadialK1>(0);
  }

  BOOST_CHECK_EQUAL(4, sfmData.intrinsics.size());
  GroupSharedIntrinsics(sfmData);
  BOOST_CHECK_EQUAL(2, sfmData.intrinsics.size());
  // Sort view Id from their intrinsic id to check that two view are in each intrinsic group
  std::map<IndexT, std::set<IndexT> > map_viewCount_per_intrinsic_id;
  for (const auto & val : sfmData.GetViews())
  {
    map_viewCount_per_intrinsic_id[val.second->getIntrinsicId()].insert(val.second->getViewId());
  }
  // Check that two view Id are linked to 0 and 1
  BOOST_CHECK_EQUAL(2, map_viewCount_per_intrinsic_id.size());
  BOOST_CHECK_EQUAL(1, map_viewCount_per_intrinsic_id.count(0));
  BOOST_CHECK_EQUAL(2, map_viewCount_per_intrinsic_id[0].size());
  BOOST_CHECK_EQUAL(1, map_viewCount_per_intrinsic_id.count(1));
  BOOST_CHECK_EQUAL(2, map_viewCount_per_intrinsic_id[1].size());
}
