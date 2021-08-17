// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Timer.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <boost/filesystem.hpp>

#include <sstream>

#define BOOST_TEST_MODULE sfmDataIO

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::sfmDataIO;

namespace fs = boost::filesystem;

// Create a SfM scene with desired count of views & poses & intrinsic (shared or not)
// Add a 3D point with observation in 2 view (just in order to have non empty data)
sfmData::SfMData createTestScene(std::size_t viewsCount = 2, std::size_t observationCount = 2, bool sharedIntrinsic = true)
{
  sfmData::SfMData sfmData;

  for(IndexT i = 0; i < viewsCount; ++i)
  {
    // Add views
    std::ostringstream os;
    os << "dataset/" << i << ".jpg";
    const IndexT viewId = i, poseId = i;
    const IndexT intrinsicId = sharedIntrinsic ? 0 : i; //(shared or not intrinsics)

    std::shared_ptr<sfmData::View> view = std::make_shared<sfmData::View>(os.str(),viewId, intrinsicId, poseId, 1500, 1000);

    sfmData.views[viewId] = view;

    // Add poses
    sfmData.setPose(*view, sfmData::CameraPose());

    // Add intrinsics
    if(!sharedIntrinsic || (i == 0))
    {
      sfmData.intrinsics[i] = std::make_shared<Pinhole>(1500, 1000, 700, 600, 10, -20);
    }
  }

  // Fill with not meaningful tracks
  sfmData::Observations observations;
  const double unknownScale = 0.0;
  for(std::size_t i = 0; i < observationCount; ++i)
  {
    observations[i] = sfmData::Observation( Vec2(i,i), i, unknownScale);
  }

  sfmData.structure[0].observations = observations;
  sfmData.structure[0].X = Vec3(11,22,33);
  sfmData.structure[0].descType = feature::EImageDescriberType::SIFT;

  return sfmData;
}

BOOST_AUTO_TEST_CASE(SfMData_IO_SAVE_LOAD)
{
    std::vector<std::string> ext_Type = {"sfm", "json"};

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
    ext_Type.push_back("abc");
#endif

    for(int i = 0; i < ext_Type.size(); ++i)
    {
        std::ostringstream os;
        os << "SAVE_LOAD"
           << "." << ext_Type[i];
        const std::string filename = os.str();
        ALICEVISION_LOG_DEBUG("Testing:" << filename);

        BOOST_TEST_CONTEXT("SAVE, file format: " << ext_Type[i])
        {
            const sfmData::SfMData sfmData = createTestScene(2, 2, true);
            BOOST_CHECK(Save(sfmData, filename, ALL));
        }

        BOOST_TEST_CONTEXT("LOAD ALL, file format: " << ext_Type[i])
        {
            const sfmData::SfMData sfmData = createTestScene(2, 2, true);
            BOOST_CHECK(Save(sfmData, filename, ESfMData::ALL));
            sfmData::SfMData sfmDataLoad;
            ESfMData flags_part = ESfMData::ALL;
            BOOST_CHECK(Load(sfmDataLoad, filename, flags_part));
            BOOST_CHECK_EQUAL(sfmDataLoad.views.size(), sfmData.views.size());
            BOOST_CHECK_EQUAL(sfmDataLoad.getPoses().size(), sfmData.getPoses().size());
            BOOST_CHECK_EQUAL(sfmDataLoad.intrinsics.size(), sfmData.intrinsics.size());
            BOOST_CHECK_EQUAL(sfmDataLoad.structure.size(), sfmData.structure.size());
            BOOST_CHECK_EQUAL(sfmDataLoad.control_points.size(), sfmData.control_points.size());

            BOOST_CHECK(sfmData == sfmDataLoad);
        }

        BOOST_TEST_CONTEXT("LOAD (only a subpart: VIEWS), file format: " << ext_Type[i])
        {
            const sfmData::SfMData sfmData = createTestScene(2, 2, true);
            BOOST_CHECK(Save(sfmData, filename, ALL));
            sfmData::SfMData sfmDataLoad;
            ESfMData flags_part = ESfMData::VIEWS;
            BOOST_CHECK(Load(sfmDataLoad, filename, flags_part));
            BOOST_CHECK_EQUAL(sfmDataLoad.views.size(), sfmData.views.size());
            BOOST_CHECK_EQUAL(sfmDataLoad.getPoses().size(), 0);
            BOOST_CHECK_EQUAL(sfmDataLoad.intrinsics.size(), 0);
            BOOST_CHECK_EQUAL(sfmDataLoad.structure.size(), 0);
            BOOST_CHECK_EQUAL(sfmDataLoad.control_points.size(), 0);
        }

        BOOST_TEST_CONTEXT("LOAD (only a subpart: POSES), file format: " << ext_Type[i])
        {
            const sfmData::SfMData sfmData = createTestScene(2, 2, true);
            BOOST_CHECK(Save(sfmData, filename, ALL));
            sfmData::SfMData sfmDataLoad;
            ESfMData flags_part = ESfMData::EXTRINSICS;
            BOOST_CHECK(Load(sfmDataLoad, filename, flags_part));
            BOOST_CHECK_EQUAL(sfmDataLoad.views.size(), 0);
            BOOST_CHECK_EQUAL(sfmDataLoad.getPoses().size(), sfmData.getPoses().size());
            BOOST_CHECK_EQUAL(sfmDataLoad.intrinsics.size(), 0);
            BOOST_CHECK_EQUAL(sfmDataLoad.structure.size(), 0);
            BOOST_CHECK_EQUAL(sfmDataLoad.control_points.size(), 0);
        }

        BOOST_TEST_CONTEXT("LOAD (only a subpart: INTRINSICS), file format: " << ext_Type[i])
        {
            const sfmData::SfMData sfmData = createTestScene(2, 2, true);
            BOOST_CHECK(Save(sfmData, filename, ALL));
            sfmData::SfMData sfmDataLoad;
            ESfMData flags_part = ESfMData::INTRINSICS;
            BOOST_CHECK(Load(sfmDataLoad, filename, flags_part));
            BOOST_CHECK_EQUAL(sfmDataLoad.views.size(), 0);
            BOOST_CHECK_EQUAL(sfmDataLoad.getPoses().size(), 0);
            BOOST_CHECK_EQUAL(sfmDataLoad.intrinsics.size(), sfmData.intrinsics.size());
            BOOST_CHECK_EQUAL(sfmDataLoad.structure.size(), 0);
            BOOST_CHECK_EQUAL(sfmDataLoad.control_points.size(), 0);
        }

        BOOST_TEST_CONTEXT("LOAD (subparts: INTRINSICS | EXTRINSICS), file format: " << ext_Type[i])
        {
            const sfmData::SfMData sfmData = createTestScene(2, 2, false); // 2 intrinsics group here
            BOOST_CHECK(Save(sfmData, filename, ALL));
            sfmData::SfMData sfmDataLoad;
            ESfMData flags_part = ESfMData(ESfMData::INTRINSICS | ESfMData::EXTRINSICS);
            BOOST_CHECK(Load(sfmDataLoad, filename, flags_part));
            BOOST_CHECK_EQUAL(sfmDataLoad.views.size(), 0);
            BOOST_CHECK_EQUAL(sfmDataLoad.getPoses().size(), sfmData.getPoses().size());
            BOOST_CHECK_EQUAL(sfmDataLoad.intrinsics.size(), sfmData.intrinsics.size());
            BOOST_CHECK_EQUAL(sfmDataLoad.structure.size(), 0);
            BOOST_CHECK_EQUAL(sfmDataLoad.control_points.size(), 0);
        }

        BOOST_TEST_CONTEXT("LOAD (subparts: VIEWS | INTRINSICS | EXTRINSICS), file format: " << ext_Type[i])
        {
            const sfmData::SfMData sfmData = createTestScene(2, 2, true);
            BOOST_CHECK(Save(sfmData, filename, ALL));
            sfmData::SfMData sfmDataLoad;
            ESfMData flags_part = ESfMData(ESfMData::VIEWS | ESfMData::INTRINSICS | ESfMData::EXTRINSICS);
            BOOST_CHECK(Load(sfmDataLoad, filename, flags_part));
            BOOST_CHECK_EQUAL(sfmDataLoad.views.size(), sfmData.views.size());
            BOOST_CHECK_EQUAL(sfmDataLoad.getPoses().size(), sfmData.getPoses().size());
            BOOST_CHECK_EQUAL(sfmDataLoad.intrinsics.size(), sfmData.intrinsics.size());
            BOOST_CHECK_EQUAL(sfmDataLoad.structure.size(), 0);
            BOOST_CHECK_EQUAL(sfmDataLoad.control_points.size(), 0);
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
      const sfmData::SfMData sfmData = createTestScene(nbViews, nbObservationPerView, true);
      BOOST_CHECK( Save(sfmData, filename, ALL) );
      ALICEVISION_LOG_DEBUG("Save Duration: " << system::prettyTime(timer.elapsedMs()));
    }

    timer.reset();

    // LOAD
    {
      sfmData::SfMData sfm_data_load;
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

    const sfmData::SfMData sfmData = createTestScene(2, 2, true);
    ESfMData flags_part = ESfMData(EXTRINSICS | STRUCTURE);
    BOOST_CHECK( Save(sfmData, filename, flags_part) );
    BOOST_CHECK( fs::is_regular_file(filename) );
  }
}
