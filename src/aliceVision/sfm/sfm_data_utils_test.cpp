// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/sfm/sfm.hpp"
#include "testing/testing.h"
#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"

#include <sstream>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::sfm;

// Test a border case (empty scene)
TEST(SfM_Data_IntrinsicGrouping, Empty)
{
  SfM_Data sfm_data;
  GroupSharedIntrinsics(sfm_data);
}

// Two similar intrinsics object must be grouped as one
TEST(SfM_Data_IntrinsicGrouping, Grouping_One)
{
  SfM_Data sfm_data;
  // One view, one intrinsic
  sfm_data.views[0] = std::make_shared<View>("", 0, 0, 0);
  sfm_data.intrinsics[0] = std::make_shared<Pinhole>(0);
  // One view, one intrinsic
  sfm_data.views[1] = std::make_shared<View>("", 1, 1, 1);
  sfm_data.intrinsics[1] = std::make_shared<Pinhole>(0);

  CHECK_EQUAL(2, sfm_data.intrinsics.size());
  GroupSharedIntrinsics(sfm_data);
  CHECK_EQUAL(1, sfm_data.intrinsics.size());
  CHECK_EQUAL(0, sfm_data.views[0]->getIntrinsicId());
  CHECK_EQUAL(0, sfm_data.views[1]->getIntrinsicId());
}

// Unique intrinsics object must not be grouped
TEST(SfM_Data_IntrinsicGrouping, No_Grouping)
{
  SfM_Data sfm_data;
  const int nbView = 10;
  for (int i = 0; i < nbView; ++i)
  {
    // Add one view, one intrinsic
    sfm_data.views[i] = std::make_shared<View>("", i, i, i);
    sfm_data.intrinsics[i] = std::make_shared<Pinhole>(i);
  }

  CHECK_EQUAL(nbView, sfm_data.intrinsics.size());
  GroupSharedIntrinsics(sfm_data);
  CHECK_EQUAL(nbView, sfm_data.intrinsics.size());
}

// Similar intrinsics object must be grouped as one
TEST(SfM_Data_IntrinsicGrouping, Grouping_Two)
{
  SfM_Data sfm_data;
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

  CHECK_EQUAL(4, sfm_data.intrinsics.size());
  GroupSharedIntrinsics(sfm_data);
  // Sort view Id from their intrinsic id to check that two view are in each intrinsic group
  std::map<IndexT, std::set<IndexT> > map_viewCount_per_intrinsic_id;
  for (const auto & val : sfm_data.GetViews())
  {
    map_viewCount_per_intrinsic_id[val.second->getIntrinsicId()].insert(val.second->getViewId());
  }
  // Check that two view Id are linked to 0 and 1
  CHECK_EQUAL(2, map_viewCount_per_intrinsic_id.size());
  CHECK_EQUAL(1, map_viewCount_per_intrinsic_id.count(0));
  CHECK_EQUAL(2, map_viewCount_per_intrinsic_id[0].size());
  CHECK_EQUAL(1, map_viewCount_per_intrinsic_id.count(1));
  CHECK_EQUAL(2, map_viewCount_per_intrinsic_id[1].size());
}

// Similar intrinsics object must be grouped as one (test with various camera type)
TEST(SfM_Data_IntrinsicGrouping, Grouping_Two_Different_Camera_Type)
{
  SfM_Data sfm_data;
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

  CHECK_EQUAL(4, sfm_data.intrinsics.size());
  GroupSharedIntrinsics(sfm_data);
  CHECK_EQUAL(2, sfm_data.intrinsics.size());
  // Sort view Id from their intrinsic id to check that two view are in each intrinsic group
  std::map<IndexT, std::set<IndexT> > map_viewCount_per_intrinsic_id;
  for (const auto & val : sfm_data.GetViews())
  {
    map_viewCount_per_intrinsic_id[val.second->getIntrinsicId()].insert(val.second->getViewId());
  }
  // Check that two view Id are linked to 0 and 1
  CHECK_EQUAL(2, map_viewCount_per_intrinsic_id.size());
  CHECK_EQUAL(1, map_viewCount_per_intrinsic_id.count(0));
  CHECK_EQUAL(2, map_viewCount_per_intrinsic_id[0].size());
  CHECK_EQUAL(1, map_viewCount_per_intrinsic_id.count(1));
  CHECK_EQUAL(2, map_viewCount_per_intrinsic_id[1].size());
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
