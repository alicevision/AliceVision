// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/sfm/sfm.hpp"
#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"
#include <sstream>

#define BOOST_TEST_MODULE sfmDataIO
#include <boost/test/included/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::sfm;

// Create a SfM scene with desired count of views & poses & intrinsic (shared or not)
// Add a 3D point with observation in 2 view (just in order to have non empty data)
SfMData create_test_scene(IndexT viewsCount, bool bSharedIntrinsic)
{
  SfMData sfm_data;
  sfm_data.s_root_path = "./";

  for(IndexT i = 0; i < viewsCount; ++i)
  {
    // Add views
    std::ostringstream os;
    os << "dataset/" << i << ".jpg";
    const IndexT id_view = i, id_pose = i;
    const IndexT id_intrinsic = bSharedIntrinsic ? 0 : i; //(shared or not intrinsics)

    std::shared_ptr<View> view = std::make_shared<View>(os.str(),id_view, id_intrinsic, id_pose, 1000, 1000);

    sfm_data.views[id_view] = view;

    // Add poses
    sfm_data.setPose(*view, Pose3());

    // Add intrinsics
    if (bSharedIntrinsic)
    {
      if (i == 0)
        sfm_data.intrinsics[0] = std::make_shared<Pinhole>();
    }
    else
    {
      sfm_data.intrinsics[i] = std::make_shared<Pinhole>();
    }
  }

  // Fill with not meaningful tracks
  Observations observations;
  observations[0] = Observation( Vec2(10,20), 0);
  observations[1] = Observation( Vec2(30,10), 1);
  sfm_data.structure[0].observations = observations;
  sfm_data.structure[0].X = Vec3(11,22,33);
  return sfm_data;
}

BOOST_AUTO_TEST_CASE(SfMData_IO_SAVE_LOAD_JSON) {

  const std::vector<std::string> ext_Type = {"json", "bin", "xml"};

  for (int i=0; i < ext_Type.size(); ++i)
  {
    std::ostringstream os;
    os << "SAVE_LOAD" << "." << ext_Type[i];
    const std::string filename = os.str();
    ALICEVISION_LOG_DEBUG("Testing:" << filename);

  // SAVE
  {
    const SfMData sfmData = create_test_scene(2, true);
    BOOST_CHECK( Save(sfmData, filename, ALL) );
  }

  // LOAD
  {
    const SfMData sfmData = create_test_scene(2, true);
    BOOST_CHECK( Save(sfmData, filename, ALL) );
    SfMData sfm_data_load;
    ESfMData flags_part = ALL;
    BOOST_CHECK( Load(sfm_data_load, filename, flags_part) );
    BOOST_CHECK_EQUAL( sfm_data_load.views.size(), sfmData.views.size());
    BOOST_CHECK_EQUAL( sfm_data_load.GetPoses().size(), sfmData.GetPoses().size());
    BOOST_CHECK_EQUAL( sfm_data_load.intrinsics.size(), sfmData.intrinsics.size());
    BOOST_CHECK_EQUAL( sfm_data_load.structure.size(), sfmData.structure.size());
    BOOST_CHECK_EQUAL( sfm_data_load.control_points.size(), sfmData.control_points.size());
  }

  // LOAD (only a subpart: VIEWS)
  {
    const SfMData sfmData = create_test_scene(2, true);
    BOOST_CHECK( Save(sfmData, filename, ALL) );
    SfMData sfm_data_load;
    ESfMData flags_part = VIEWS;
    BOOST_CHECK( Load(sfm_data_load, filename, flags_part) );
    BOOST_CHECK_EQUAL( sfm_data_load.views.size(), sfmData.views.size());
    BOOST_CHECK_EQUAL( sfm_data_load.GetPoses().size(), 0);
    BOOST_CHECK_EQUAL( sfm_data_load.intrinsics.size(), 0);
    BOOST_CHECK_EQUAL( sfm_data_load.structure.size(), 0);
    BOOST_CHECK_EQUAL( sfm_data_load.control_points.size(), 0);
  }

  // LOAD (only a subpart: POSES)
  {
    const SfMData sfmData = create_test_scene(2, true);
    BOOST_CHECK( Save(sfmData, filename, ALL) );
    SfMData sfm_data_load;
    ESfMData flags_part = EXTRINSICS;
    BOOST_CHECK( Load(sfm_data_load, filename, flags_part) );
    BOOST_CHECK_EQUAL( sfm_data_load.views.size(), 0);
    BOOST_CHECK_EQUAL( sfm_data_load.GetPoses().size(), sfmData.GetPoses().size());
    BOOST_CHECK_EQUAL( sfm_data_load.intrinsics.size(), 0);
    BOOST_CHECK_EQUAL( sfm_data_load.structure.size(), 0);
    BOOST_CHECK_EQUAL( sfm_data_load.control_points.size(), 0);
  }

  // LOAD (only a subpart: INTRINSICS)
  {
    const SfMData sfmData = create_test_scene(2, true);
    BOOST_CHECK( Save(sfmData, filename, ALL) );
    SfMData sfm_data_load;
    ESfMData flags_part = INTRINSICS;
    BOOST_CHECK( Load(sfm_data_load, filename, flags_part) );
    BOOST_CHECK_EQUAL( sfm_data_load.views.size(), 0);
    BOOST_CHECK_EQUAL( sfm_data_load.GetPoses().size(), 0);
    BOOST_CHECK_EQUAL( sfm_data_load.intrinsics.size(), sfmData.intrinsics.size());
    BOOST_CHECK_EQUAL( sfm_data_load.structure.size(), 0);
    BOOST_CHECK_EQUAL( sfm_data_load.control_points.size(), 0);
  }

  // LOAD (subparts: COMBINED)
  {
    const SfMData sfmData = create_test_scene(2,false); //2 intrinsics group here
    BOOST_CHECK( Save(sfmData, filename, ALL) );
    SfMData sfm_data_load;
    ESfMData flags_part = ESfMData(INTRINSICS | EXTRINSICS);
    BOOST_CHECK( Load(sfm_data_load, filename, flags_part) );
    BOOST_CHECK_EQUAL( sfm_data_load.views.size(), 0);
    BOOST_CHECK_EQUAL( sfm_data_load.GetPoses().size(), sfmData.GetPoses().size());
    BOOST_CHECK_EQUAL( sfm_data_load.intrinsics.size(), sfmData.intrinsics.size());
    BOOST_CHECK_EQUAL( sfm_data_load.structure.size(), 0);
    BOOST_CHECK_EQUAL( sfm_data_load.control_points.size(), 0);
  }

  // LOAD (subparts: COMBINED)
  {
    const SfMData sfmData = create_test_scene(2, true);
    BOOST_CHECK( Save(sfmData, filename, ALL) );
    SfMData sfm_data_load;
    ESfMData flags_part = ESfMData(VIEWS | INTRINSICS | EXTRINSICS);
    BOOST_CHECK( Load(sfm_data_load, filename, flags_part) );
    BOOST_CHECK_EQUAL( sfm_data_load.views.size(), sfmData.views.size());
    BOOST_CHECK_EQUAL( sfm_data_load.GetPoses().size(), sfmData.GetPoses().size());
    BOOST_CHECK_EQUAL( sfm_data_load.intrinsics.size(), sfmData.intrinsics.size());
    BOOST_CHECK_EQUAL( sfm_data_load.structure.size(), 0);
    BOOST_CHECK_EQUAL( sfm_data_load.control_points.size(), 0);
    }
  }
}

BOOST_AUTO_TEST_CASE(SfMData_IO_SAVE_PLY) {

  // SAVE as PLY
  {
    std::ostringstream os;
    os << "SAVE_LOAD" << ".ply";
    const std::string filename = os.str();
    ALICEVISION_LOG_DEBUG("Testing:" << filename);

    const SfMData sfmData = create_test_scene(2, true);
    ESfMData flags_part = ESfMData(EXTRINSICS | STRUCTURE);
    BOOST_CHECK( Save(sfmData, filename, flags_part) );
    BOOST_CHECK( stlplus::is_file(filename) );
  }
}
