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
  SfMData sfm_data;
  GroupSharedIntrinsics(sfm_data);
}

// Two similar intrinsics object must be grouped as one
BOOST_AUTO_TEST_CASE(SfMData_IntrinsicGrouping_Grouping_One)
{
  SfMData sfm_data;
  // One view, one intrinsic
  sfm_data.views[0] = std::make_shared<View>("", 0, 0, 0);
  sfm_data.intrinsics[0] = std::make_shared<Pinhole>(0);
  // One view, one intrinsic
  sfm_data.views[1] = std::make_shared<View>("", 1, 1, 1);
  sfm_data.intrinsics[1] = std::make_shared<Pinhole>(0);

  BOOST_CHECK_EQUAL(2, sfm_data.intrinsics.size());
  GroupSharedIntrinsics(sfm_data);
  BOOST_CHECK_EQUAL(1, sfm_data.intrinsics.size());
  BOOST_CHECK_EQUAL(0, sfm_data.views[0]->getIntrinsicId());
  BOOST_CHECK_EQUAL(0, sfm_data.views[1]->getIntrinsicId());
}

// Unique intrinsics object must not be grouped
BOOST_AUTO_TEST_CASE(SfMData_IntrinsicGrouping_No_Grouping)
{
  SfMData sfm_data;
  const int nbView = 10;
  for (int i = 0; i < nbView; ++i)
  {
    // Add one view, one intrinsic
    sfm_data.views[i] = std::make_shared<View>("", i, i, i);
    sfm_data.intrinsics[i] = std::make_shared<Pinhole>(i);
  }

  BOOST_CHECK_EQUAL(nbView, sfm_data.intrinsics.size());
  GroupSharedIntrinsics(sfm_data);
  BOOST_CHECK_EQUAL(nbView, sfm_data.intrinsics.size());
}

// Similar intrinsics object must be grouped as one
BOOST_AUTO_TEST_CASE(SfMData_IntrinsicGrouping_Grouping_Two)
{
  SfMData sfm_data;
  // Define separate intrinsics that share common properties
  // first block of intrinsics
  {
    sfm_data.views[0] = std::make_shared<View>("", 0, 0, 0);
    sfm_data.intrinsics[0] = std::make_shared<Pinhole>(0);
    sfm_data.views[1] = std::make_shared<View>("", 1, 1, 1);
    sfm_data.intrinsics[1] = std::make_shared<Pinhole>(0);
  }
  // second block of intrinsics
  {
    sfm_data.views[2] = std::make_shared<View>("", 2, 2, 2);
    sfm_data.intrinsics[2] = std::make_shared<Pinhole>(1);
    sfm_data.views[3] = std::make_shared<View>("", 3, 3, 3);
    sfm_data.intrinsics[3] = std::make_shared<Pinhole>(1);
  }

  BOOST_CHECK_EQUAL(4, sfm_data.intrinsics.size());
  GroupSharedIntrinsics(sfm_data);
  // Sort view Id from their intrinsic id to check that two view are in each intrinsic group
  std::map<IndexT, std::set<IndexT> > map_viewCount_per_intrinsic_id;
  for (const auto & val : sfm_data.GetViews())
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
  SfMData sfm_data;
  // Define separate intrinsics that share common properties
  // first block of intrinsics
  {
    sfm_data.views[0] = std::make_shared<View>("", 0, 0, 0);
    sfm_data.intrinsics[0] = std::make_shared<Pinhole>(0);
    sfm_data.views[1] = std::make_shared<View>("", 1, 1, 1);
    sfm_data.intrinsics[1] = std::make_shared<Pinhole>(0);
  }
  // second block of intrinsics (different type)
  {
    sfm_data.views[2] = std::make_shared<View>("", 2, 2, 2);
    sfm_data.intrinsics[2] = std::make_shared<PinholeRadialK1>(0);
    sfm_data.views[3] = std::make_shared<View>("", 3, 3, 3);
    sfm_data.intrinsics[3] = std::make_shared<PinholeRadialK1>(0);
  }

  BOOST_CHECK_EQUAL(4, sfm_data.intrinsics.size());
  GroupSharedIntrinsics(sfm_data);
  BOOST_CHECK_EQUAL(2, sfm_data.intrinsics.size());
  // Sort view Id from their intrinsic id to check that two view are in each intrinsic group
  std::map<IndexT, std::set<IndexT> > map_viewCount_per_intrinsic_id;
  for (const auto & val : sfm_data.GetViews())
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
