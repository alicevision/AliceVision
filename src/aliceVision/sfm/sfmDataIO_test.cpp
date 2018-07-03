// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Timer.hpp>
#include <aliceVision/sfm/sfm.hpp>

#include <boost/filesystem.hpp>

#include <sstream>

#define BOOST_TEST_MODULE sfmDataIO
#include <boost/test/included/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::sfm;
namespace fs = boost::filesystem;

// Create a SfM scene with desired count of views & poses & intrinsic (shared or not)
// Add a 3D point with observation in 2 view (just in order to have non empty data)
SfMData createTestScene(std::size_t viewsCount = 2, std::size_t observationCount = 2, bool sharedIntrinsic = true)
{
  SfMData sfm_data;

  for(IndexT i = 0; i < viewsCount; ++i)
  {
    // Add views
    std::ostringstream os;
    os << "dataset/" << i << ".jpg";
    const IndexT id_view = i, id_pose = i;
    const IndexT id_intrinsic = sharedIntrinsic ? 0 : i; //(shared or not intrinsics)

    std::shared_ptr<View> view = std::make_shared<View>(os.str(),id_view, id_intrinsic, id_pose, 1000, 1000);

    sfm_data.views[id_view] = view;

    // Add poses
    sfm_data.setPose(*view, CameraPose());

    // Add intrinsics
    if (sharedIntrinsic)
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
  for(std::size_t i = 0; i < observationCount; ++i)
  {
    observations[i] = Observation( Vec2(i,i), i);
  }

  sfm_data.structure[0].observations = observations;
  sfm_data.structure[0].X = Vec3(11,22,33);
  sfm_data.structure[0].descType = feature::EImageDescriberType::SIFT;

  return sfm_data;
}

BOOST_AUTO_TEST_CASE(SfMData_IO_SAVE_LOAD_JSON) {

  const std::vector<std::string> ext_Type = {"sfm","json"};

  for(int i = 0; i < ext_Type.size(); ++i)
  {
    std::ostringstream os;
    os << "SAVE_LOAD" << "." << ext_Type[i];
    const std::string filename = os.str();
    ALICEVISION_LOG_DEBUG("Testing:" << filename);

  // SAVE
  {
    const SfMData sfmData = createTestScene(2, 2, true);
    BOOST_CHECK( Save(sfmData, filename, ALL) );
  }

  // LOAD
  {
    const SfMData sfmData = createTestScene(2, 2, true);
    BOOST_CHECK( Save(sfmData, filename, ALL) );
    SfMData sfm_data_load;
    ESfMData flags_part = ALL;
    BOOST_CHECK( Load(sfm_data_load, filename, flags_part) );
    BOOST_CHECK_EQUAL( sfm_data_load.views.size(), sfmData.views.size());
    BOOST_CHECK_EQUAL( sfm_data_load.getPoses().size(), sfmData.getPoses().size());
    BOOST_CHECK_EQUAL( sfm_data_load.intrinsics.size(), sfmData.intrinsics.size());
    BOOST_CHECK_EQUAL( sfm_data_load.structure.size(), sfmData.structure.size());
    BOOST_CHECK_EQUAL( sfm_data_load.control_points.size(), sfmData.control_points.size());
  }

  // LOAD (only a subpart: VIEWS)
  {
    const SfMData sfmData = createTestScene(2, 2, true);
    BOOST_CHECK( Save(sfmData, filename, ALL) );
    SfMData sfm_data_load;
    ESfMData flags_part = VIEWS;
    BOOST_CHECK( Load(sfm_data_load, filename, flags_part) );
    BOOST_CHECK_EQUAL( sfm_data_load.views.size(), sfmData.views.size());
    BOOST_CHECK_EQUAL( sfm_data_load.getPoses().size(), 0);
    BOOST_CHECK_EQUAL( sfm_data_load.intrinsics.size(), 0);
    BOOST_CHECK_EQUAL( sfm_data_load.structure.size(), 0);
    BOOST_CHECK_EQUAL( sfm_data_load.control_points.size(), 0);
  }

  // LOAD (only a subpart: POSES)
  {
    const SfMData sfmData = createTestScene(2, 2, true);
    BOOST_CHECK( Save(sfmData, filename, ALL) );
    SfMData sfm_data_load;
    ESfMData flags_part = EXTRINSICS;
    BOOST_CHECK( Load(sfm_data_load, filename, flags_part) );
    BOOST_CHECK_EQUAL( sfm_data_load.views.size(), 0);
    BOOST_CHECK_EQUAL( sfm_data_load.getPoses().size(), sfmData.getPoses().size());
    BOOST_CHECK_EQUAL( sfm_data_load.intrinsics.size(), 0);
    BOOST_CHECK_EQUAL( sfm_data_load.structure.size(), 0);
    BOOST_CHECK_EQUAL( sfm_data_load.control_points.size(), 0);
  }

  // LOAD (only a subpart: INTRINSICS)
  {
    const SfMData sfmData = createTestScene(2, 2, true);
    BOOST_CHECK( Save(sfmData, filename, ALL) );
    SfMData sfm_data_load;
    ESfMData flags_part = INTRINSICS;
    BOOST_CHECK( Load(sfm_data_load, filename, flags_part) );
    BOOST_CHECK_EQUAL( sfm_data_load.views.size(), 0);
    BOOST_CHECK_EQUAL( sfm_data_load.getPoses().size(), 0);
    BOOST_CHECK_EQUAL( sfm_data_load.intrinsics.size(), sfmData.intrinsics.size());
    BOOST_CHECK_EQUAL( sfm_data_load.structure.size(), 0);
    BOOST_CHECK_EQUAL( sfm_data_load.control_points.size(), 0);
  }

  // LOAD (subparts: COMBINED)
  {
    const SfMData sfmData = createTestScene(2, 2, false); //2 intrinsics group here
    BOOST_CHECK( Save(sfmData, filename, ALL) );
    SfMData sfm_data_load;
    ESfMData flags_part = ESfMData(INTRINSICS | EXTRINSICS);
    BOOST_CHECK( Load(sfm_data_load, filename, flags_part) );
    BOOST_CHECK_EQUAL( sfm_data_load.views.size(), 0);
    BOOST_CHECK_EQUAL( sfm_data_load.getPoses().size(), sfmData.getPoses().size());
    BOOST_CHECK_EQUAL( sfm_data_load.intrinsics.size(), sfmData.intrinsics.size());
    BOOST_CHECK_EQUAL( sfm_data_load.structure.size(), 0);
    BOOST_CHECK_EQUAL( sfm_data_load.control_points.size(), 0);
  }

  // LOAD (subparts: COMBINED)
  {
    const SfMData sfmData = createTestScene(2, 2, true);
    BOOST_CHECK( Save(sfmData, filename, ALL) );
    SfMData sfm_data_load;
    ESfMData flags_part = ESfMData(VIEWS | INTRINSICS | EXTRINSICS);
    BOOST_CHECK( Load(sfm_data_load, filename, flags_part) );
    BOOST_CHECK_EQUAL( sfm_data_load.views.size(), sfmData.views.size());
    BOOST_CHECK_EQUAL( sfm_data_load.getPoses().size(), sfmData.getPoses().size());
    BOOST_CHECK_EQUAL( sfm_data_load.intrinsics.size(), sfmData.intrinsics.size());
    BOOST_CHECK_EQUAL( sfm_data_load.structure.size(), 0);
    BOOST_CHECK_EQUAL( sfm_data_load.control_points.size(), 0);
    }
  }
}

/*
BOOST_AUTO_TEST_CASE(SfMData_IO_BigFile) {
  const int nbViews = 1000;
  const int nbObservationPerView = 100000;
  std::vector<std::string> ext_Type = {"sfm","json"};

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
  ext_Type.push_back("abc");
#endif

  for (int i = 0; i < ext_Type.size(); ++i)
  {
    std::ostringstream os;
    os << "BIG_SAVE_LOAD" << "." << ext_Type[i];
    const std::string filename = os.str();
    ALICEVISION_LOG_DEBUG("Testing:" << filename);

    system::Timer timer;

    // SAVE
    {
      const SfMData sfmData = createTestScene(nbViews, nbObservationPerView, true);
      BOOST_CHECK( Save(sfmData, filename, ALL) );
      ALICEVISION_LOG_DEBUG("Save Duration: " << system::prettyTime(timer.elapsedMs()));
    }

    timer.reset();

    // LOAD
    {
      SfMData sfm_data_load;
      BOOST_CHECK( Load(sfm_data_load, filename, ALL));
      ALICEVISION_LOG_DEBUG("Load Duration: " << system::prettyTime(timer.elapsedMs()));
    }
  }
}
*/

BOOST_AUTO_TEST_CASE(SfMData_IO_SAVE_PLY) {

  // SAVE as PLY
  {
    std::ostringstream os;
    os << "SAVE_LOAD" << ".ply";
    const std::string filename = os.str();
    ALICEVISION_LOG_DEBUG("Testing:" << filename);

    const SfMData sfmData = createTestScene(2, 2, true);
    ESfMData flags_part = ESfMData(EXTRINSICS | STRUCTURE);
    BOOST_CHECK( Save(sfmData, filename, flags_part) );
    BOOST_CHECK( fs::is_regular_file(filename) );
  }
}
