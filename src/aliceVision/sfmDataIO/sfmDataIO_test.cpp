// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Timer.hpp>

#include <aliceVision/config.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/camera/Pinhole.hpp>
#include <aliceVision/camera/camera.hpp>

#include <filesystem>
#include <sstream>

#define BOOST_TEST_MODULE sfmDataIO

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::sfmDataIO;

namespace fs = std::filesystem;

// Create a SfM scene with desired count of views & poses & intrinsic (shared or not)
// Add a 3D point with observation in 2 view (just in order to have non empty data)
sfmData::SfMData createTestScene(std::size_t viewsCount = 2, std::size_t observationCount = 2, bool sharedIntrinsic = true)
{
    sfmData::SfMData sfmData;

    for (IndexT i = 0; i < viewsCount; ++i)
    {
        // Add views
        std::ostringstream os;
        os << "dataset/" << i << ".jpg";
        const IndexT viewId = i, poseId = i;
        const IndexT intrinsicId = sharedIntrinsic ? 0 : i;  //(shared or not intrinsics)

        std::shared_ptr<sfmData::View> view = std::make_shared<sfmData::View>(os.str(), viewId, intrinsicId, poseId, 1500, 1000);

        sfmData.getViews().emplace(viewId, view);

        // Add poses
        sfmData::CameraPose pose;
        if (i == 0)
        {
            pose.lock();
        }

        sfmData.setPose(*view, pose);

        // Add intrinsics
        if (!sharedIntrinsic || (i == 0))
        {
            sfmData.getIntrinsics().emplace(i, std::make_shared<Pinhole>(1500, 1000, 700, 600, 10, -20));
        }
    }

    // Fill with not meaningful tracks
    sfmData::Observations observations;
    const double unknownScale = 0.0;
    for (std::size_t i = 0; i < observationCount; ++i)
    {
        observations[i] = sfmData::Observation(Vec2(i, i), i, unknownScale);
    }

    sfmData.getLandmarks()[0].getObservations() = observations;
    sfmData.getLandmarks()[0].setX(Vec3(11, 22, 33));
    sfmData.getLandmarks()[0].setDescType(feature::EImageDescriberType::SIFT);

    return sfmData;
}

BOOST_AUTO_TEST_CASE(SfMData_IO_SAVE_LOAD)
{
    std::vector<std::string> ext_Type = {"sfm", "json"};

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
    ext_Type.push_back("abc");
#endif

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_USD)
    ext_Type.push_back("usda");
#endif
    for (int i = 0; i < ext_Type.size(); ++i)
    {
        std::ostringstream os;
        os << "SAVE_LOAD"
           << "." << ext_Type[i];
        const std::string filename = os.str();
        ALICEVISION_LOG_DEBUG("Testing:" << filename);

        BOOST_TEST_CONTEXT("SAVE, file format: " << ext_Type[i])
        {
            const sfmData::SfMData sfmData = createTestScene(2, 2, true);
            BOOST_CHECK(save(sfmData, filename, ALL));
        }

        BOOST_TEST_CONTEXT("LOAD ALL, file format: " << ext_Type[i])
        {
            const sfmData::SfMData sfmData = createTestScene(2, 2, true);
            BOOST_CHECK(save(sfmData, filename, ESfMData::ALL));
            sfmData::SfMData sfmDataLoad;
            ESfMData flags_part = ESfMData::ALL;
            BOOST_CHECK(load(sfmDataLoad, filename, flags_part));
            BOOST_CHECK_EQUAL(sfmDataLoad.getViews().size(), sfmData.getViews().size());
            BOOST_CHECK_EQUAL(sfmDataLoad.getPoses().size(), sfmData.getPoses().size());
            BOOST_CHECK_EQUAL(sfmDataLoad.getIntrinsics().size(), sfmData.getIntrinsics().size());
            BOOST_CHECK_EQUAL(sfmDataLoad.getLandmarks().size(), sfmData.getLandmarks().size());

            BOOST_CHECK(sfmData == sfmDataLoad);
        }

        BOOST_TEST_CONTEXT("LOAD (only a subpart: VIEWS), file format: " << ext_Type[i])
        {
            const sfmData::SfMData sfmData = createTestScene(2, 2, true);
            BOOST_CHECK(save(sfmData, filename, ALL));
            sfmData::SfMData sfmDataLoad;
            ESfMData flags_part = ESfMData::VIEWS;
            BOOST_CHECK(load(sfmDataLoad, filename, flags_part));
            BOOST_CHECK_EQUAL(sfmDataLoad.getViews().size(), sfmData.getViews().size());
            BOOST_CHECK_EQUAL(sfmDataLoad.getPoses().size(), 0);
            BOOST_CHECK_EQUAL(sfmDataLoad.getIntrinsics().size(), 0);
            BOOST_CHECK_EQUAL(sfmDataLoad.getLandmarks().size(), 0);
        }

        BOOST_TEST_CONTEXT("LOAD (only a subpart: POSES), file format: " << ext_Type[i])
        {
            const sfmData::SfMData sfmData = createTestScene(2, 2, true);
            BOOST_CHECK(save(sfmData, filename, ALL));
            sfmData::SfMData sfmDataLoad;
            ESfMData flags_part = ESfMData::EXTRINSICS;
            BOOST_CHECK(load(sfmDataLoad, filename, flags_part));
            BOOST_CHECK_EQUAL(sfmDataLoad.getViews().size(), 0);
            BOOST_CHECK_EQUAL(sfmDataLoad.getPoses().size(), sfmData.getPoses().size());
            BOOST_CHECK_EQUAL(sfmDataLoad.getIntrinsics().size(), 0);
            BOOST_CHECK_EQUAL(sfmDataLoad.getLandmarks().size(), 0);
        }

        BOOST_TEST_CONTEXT("LOAD (only a subpart: INTRINSICS), file format: " << ext_Type[i])
        {
            const sfmData::SfMData sfmData = createTestScene(2, 2, true);
            BOOST_CHECK(save(sfmData, filename, ALL));
            sfmData::SfMData sfmDataLoad;
            ESfMData flags_part = ESfMData::INTRINSICS;
            BOOST_CHECK(load(sfmDataLoad, filename, flags_part));
            BOOST_CHECK_EQUAL(sfmDataLoad.getViews().size(), 0);
            BOOST_CHECK_EQUAL(sfmDataLoad.getPoses().size(), 0);
            BOOST_CHECK_EQUAL(sfmDataLoad.getIntrinsics().size(), sfmData.getIntrinsics().size());
            BOOST_CHECK_EQUAL(sfmDataLoad.getLandmarks().size(), 0);
        }

        BOOST_TEST_CONTEXT("LOAD (subparts: INTRINSICS | EXTRINSICS), file format: " << ext_Type[i])
        {
            const sfmData::SfMData sfmData = createTestScene(2, 2, false);  // 2 intrinsics group here
            BOOST_CHECK(save(sfmData, filename, ALL));
            sfmData::SfMData sfmDataLoad;
            ESfMData flags_part = ESfMData(ESfMData::INTRINSICS | ESfMData::EXTRINSICS);
            BOOST_CHECK(load(sfmDataLoad, filename, flags_part));
            BOOST_CHECK_EQUAL(sfmDataLoad.getViews().size(), 0);
            BOOST_CHECK_EQUAL(sfmDataLoad.getPoses().size(), sfmData.getPoses().size());
            BOOST_CHECK_EQUAL(sfmDataLoad.getIntrinsics().size(), sfmData.getIntrinsics().size());
            BOOST_CHECK_EQUAL(sfmDataLoad.getLandmarks().size(), 0);
        }

        BOOST_TEST_CONTEXT("LOAD (subparts: VIEWS | INTRINSICS | EXTRINSICS), file format: " << ext_Type[i])
        {
            const sfmData::SfMData sfmData = createTestScene(2, 2, true);
            BOOST_CHECK(save(sfmData, filename, ALL));
            sfmData::SfMData sfmDataLoad;
            ESfMData flags_part = ESfMData(ESfMData::VIEWS | ESfMData::INTRINSICS | ESfMData::EXTRINSICS);
            BOOST_CHECK(load(sfmDataLoad, filename, flags_part));
            BOOST_CHECK_EQUAL(sfmDataLoad.getViews().size(), sfmData.getViews().size());
            BOOST_CHECK_EQUAL(sfmDataLoad.getPoses().size(), sfmData.getPoses().size());
            BOOST_CHECK_EQUAL(sfmDataLoad.getIntrinsics().size(), sfmData.getIntrinsics().size());
            BOOST_CHECK_EQUAL(sfmDataLoad.getLandmarks().size(), 0);
        }
    }
}

BOOST_AUTO_TEST_CASE(SfMData_IO_SAVE_LOAD_RescaledUndistortion)
{
    std::vector<std::string> ext_Type = {"sfm", "json"};

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
    ext_Type.push_back("abc");
#endif

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_USD)
    ext_Type.push_back("usda");
#endif

    // Non integer rescales: the rescaled diagonal differs from the one of the rounded image size.
    // The non uniform one also changes the pixel aspect ratio of the undistortion.
    const std::vector<std::pair<float, float>> factors = {{0.33f, 0.33f}, {0.33f, 0.5f}};

    for (const auto& [factorW, factorH] : factors)
    {
        std::shared_ptr<Pinhole> intrinsic =
          createPinhole(EDISTORTION::DISTORTION_NONE, EUNDISTORTION::UNDISTORTION_3DERADIAL4, 1920, 1080, 1500, 1500, 10, -20);
        std::shared_ptr<Undistortion> undistortion = intrinsic->getUndistortion();
        BOOST_REQUIRE(undistortion);
        std::vector<double> params = undistortion->getParameters();
        for (std::size_t i = 0; i < params.size(); ++i)
        {
            params[i] = 0.01 * (i + 1);
        }
        undistortion->setParameters(params);
        undistortion->setOffset(Vec2(3.5, -2.25));

        intrinsic->rescale(factorW, factorH);

        sfmData::SfMData sfmData = createTestScene(1, 2, true);
        sfmData.getIntrinsics().erase(0);
        sfmData.getIntrinsics().emplace(0, intrinsic);

        for (const std::string& ext : ext_Type)
        {
            BOOST_TEST_CONTEXT("factors: " << factorW << "x" << factorH << ", file format: " << ext)
            {
                const std::string filename = "SAVE_LOAD_RESCALED_UNDISTORTION." + ext;
                BOOST_CHECK(save(sfmData, filename, ALL));

                sfmData::SfMData sfmDataLoad;
                BOOST_REQUIRE(load(sfmDataLoad, filename, ALL));

                auto intrinsicLoad = std::dynamic_pointer_cast<Pinhole>(sfmDataLoad.getIntrinsics().at(0));
                BOOST_REQUIRE(intrinsicLoad);
                std::shared_ptr<Undistortion> undistortionLoad = intrinsicLoad->getUndistortion();
                BOOST_REQUIRE(undistortionLoad);

                BOOST_CHECK_CLOSE(undistortionLoad->getDiagonal(), undistortion->getDiagonal(), 1e-9);
                BOOST_CHECK_CLOSE(undistortionLoad->getPixelAspectRatio(), undistortion->getPixelAspectRatio(), 1e-9);
                BOOST_CHECK_EQUAL(undistortionLoad->isDesqueezed(), undistortion->isDesqueezed());
                BOOST_CHECK_SMALL((undistortionLoad->getCenter() - undistortion->getCenter()).norm(), 1e-9);

                for (const Vec2& pt : {Vec2(0.0, 0.0), Vec2(100.25, 50.5), Vec2(633.0, 356.0)})
                {
                    BOOST_CHECK_SMALL((undistortionLoad->undistort(pt) - undistortion->undistort(pt)).norm(), 1e-6);
                }
            }
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
      BOOST_CHECK(save(sfmData, filename, ALL));
      ALICEVISION_LOG_DEBUG("Save Duration: " << system::prettyTime(timer.elapsedMs()));
    }

    timer.reset();

    // LOAD
    {
      sfmData::SfMData sfm_data_load;
      BOOST_CHECK(load(sfm_data_load, filename, ALL));
      ALICEVISION_LOG_DEBUG("Load Duration: " << system::prettyTime(timer.elapsedMs()));
    }
  }
}
*/

BOOST_AUTO_TEST_CASE(SfMData_IO_SAVE_PLY)
{
    // SAVE as PLY
    {
        std::ostringstream os;
        os << "SAVE_LOAD"
           << ".ply";
        const std::string filename = os.str();
        ALICEVISION_LOG_DEBUG("Testing:" << filename);

        const sfmData::SfMData sfmData = createTestScene(2, 2, true);
        ESfMData flags_part = ESfMData(EXTRINSICS | STRUCTURE);
        BOOST_CHECK(save(sfmData, filename, flags_part));
        BOOST_CHECK(fs::is_regular_file(filename));
    }
}
