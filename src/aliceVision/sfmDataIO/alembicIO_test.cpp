// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "AlembicImporter.hpp"
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#define BOOST_TEST_MODULE alembicIO

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

#include <iostream>

using namespace aliceVision;
using namespace aliceVision::sfmData;
using namespace aliceVision::sfmDataIO;

// Create a SfM scene with desired count of views & poses & intrinsic (shared or not)
// Add a 3D point with observation in 2 view (just in order to have non empty data)
SfMData createTestScene(IndexT singleViewsCount,
                           IndexT pointCount,
                           IndexT rigCount,
                           IndexT subPoseCount,
                           bool bSharedIntrinsic)
{
  SfMData sfm_data;

  std::srand(time(nullptr));

  for(IndexT i = 0; i < singleViewsCount; ++i)
  {
    // Add views
    std::ostringstream os;
    os << "dataset/" << i << ".jpg";
    const IndexT id_view = i, id_pose = i;
    const IndexT id_intrinsic = bSharedIntrinsic ? 0 : i; //(shared or not intrinsics)

    std::shared_ptr<View> view = std::make_shared<View>(os.str(),id_view, id_intrinsic, id_pose, 1000, 1000);
    view->addMetadata("A","A");
    view->addMetadata("B","B");
    view->addMetadata("C","C");
    sfm_data.views[id_view] = view;

    // Add poses
    const Mat3 r = Mat3::Random();
    const Vec3 c = Vec3::Random();
    sfm_data.setPose(*view, CameraPose(geometry::Pose3(r, c)));

    // Add intrinsics
    if(bSharedIntrinsic)
    {
      if(i == 0)
      {
        sfm_data.intrinsics[0] = std::make_shared<camera::Pinhole>(1000, 1000, 36.0, 36.0, std::rand()%10000, std::rand()%10000);
      }
    }
    else
    {
      sfm_data.intrinsics[i] = std::make_shared<camera::Pinhole>(1000, 1000, 36.0, 36.0, std::rand()%10000, std::rand()%10000);
    }
  }


  std::size_t nbIntrinsics = (bSharedIntrinsic ? 1 : singleViewsCount);
  std::size_t nbPoses = singleViewsCount;
  std::size_t nbViews = singleViewsCount;
  const std::size_t nbRigPoses = 5;

  for(IndexT rigId = 0; rigId < rigCount; ++rigId)
  {
    sfm_data.getRigs().emplace(rigId, Rig(subPoseCount));
    Rig& rig = sfm_data.getRigs().at(rigId);

    for(IndexT subPoseId = 0; subPoseId < subPoseCount; ++subPoseId)
    {
      {
        const Mat3 r = Mat3::Random();
        const Vec3 c = Vec3::Random();
        rig.setSubPose(subPoseId, RigSubPose(geometry::Pose3(r, c), ERigSubPoseStatus::ESTIMATED));
      }

      sfm_data.intrinsics[nbIntrinsics + subPoseId] = std::make_shared<camera::Pinhole>(1000, 1000, 36.0, 36.0, std::rand()%10000, std::rand()%10000);

      for(std::size_t pose = 0; pose < nbRigPoses; ++pose)
      {
        std::ostringstream os;
        os << "dataset/rig_" << rigId << "_"  <<  subPoseId << "_" << pose << ".jpg";

        std::shared_ptr<View> view = std::make_shared<View>(os.str(),
                                                            nbViews,
                                                            nbIntrinsics + subPoseId,
                                                            nbPoses + pose,
                                                            1000,
                                                            1000,
                                                            rigId,
                                                            subPoseId);

        if(subPoseId == 0)
        {
          const Mat3 r = Mat3::Random();
          const Vec3 c = Vec3::Random();
          sfm_data.setPose(*view, CameraPose(geometry::Pose3(r, c)));
        }

        view->setFrameId(nbPoses + pose);
        view->setIndependantPose(false);

        sfm_data.views[nbViews] = view;
        ++nbViews;
      }
    }
    nbPoses += nbRigPoses;
    nbIntrinsics += subPoseCount;
  }

  const double unknownScale = 0.0;
  // Fill with not meaningful tracks
  for(IndexT i = 0; i < pointCount; ++i)
  {
    // Add structure
    Observations observations;
    observations[0] = Observation( Vec2(std::rand() % 10000, std::rand() % 10000), 0, unknownScale);
    observations[1] = Observation( Vec2(std::rand() % 10000, std::rand() % 10000), 1, unknownScale);
    sfm_data.structure[i].observations = observations;
    sfm_data.structure[i].X = Vec3(std::rand() % 10000, std::rand() % 10000, std::rand() % 10000);
    sfm_data.structure[i].rgb = image::RGBColor((std::rand() % 1000) / 1000.0, (std::rand() % 1000) / 1000.0, (std::rand() % 1000) / 1000.0);
    sfm_data.structure[i].descType = feature::EImageDescriberType::SIFT;

    // Add control points    
  }

  return sfm_data;
}

//-----------------
// Test summary:
//-----------------
// - Create a random scene (.json)
// - Export to Alembic
// - Import to Alembic
// - Import to .json
//-----------------
BOOST_AUTO_TEST_CASE(AlembicImporter_importExport) {

    int flags = ALL;

    // Create a random scene
    const SfMData sfmData = createTestScene(5, 50, 2, 3, true);
    
    // JSON -> JSON

    // Export as JSON
    const std::string jsonFile = "importExport.sfm";
    {
        BOOST_CHECK(Save(
        sfmData,
        jsonFile.c_str(),
        ESfMData(flags)));
    }

    // Reload
    SfMData sfmJsonToJson;
    {
        BOOST_CHECK(Load(sfmJsonToJson, jsonFile, ESfMData(flags)));
        BOOST_CHECK(sfmData == sfmJsonToJson);
    }
    
    // ABC -> ABC

    // Export as ABC
    const std::string abcFile = "abcToAbc.abc";
    {
        BOOST_CHECK(Save(
        sfmData,
        abcFile.c_str(),              
        ESfMData(flags)));
    }

    // Reload
    SfMData sfmAbcToAbc;
    {
        BOOST_CHECK(Load(sfmAbcToAbc, abcFile, ESfMData(flags)));
        std::string abcFile2 = "abcToJson.sfm";
        BOOST_CHECK(Save(
        sfmAbcToAbc,
        abcFile2.c_str(),
        ESfMData(flags)));
        BOOST_CHECK(sfmData == sfmAbcToAbc);
    }

    // Export as ABC
    const std::string abcFile2 = "abcToAbc2.abc";
    {
        BOOST_CHECK(Save(
        sfmAbcToAbc,
        abcFile2.c_str(),
        ESfMData(flags)));
    }

    // JSON -> ABC -> ABC -> JSON

    // Export as JSON
    const std::string jsonFile3 = "jsonToABC.sfm";
    {
        BOOST_CHECK(Save(
        sfmData,
        jsonFile3.c_str(),
        ESfMData(flags)));
    }

    // Reload
    SfMData sfmJsonToABC;
    {
        BOOST_CHECK(Load(sfmJsonToABC, jsonFile3, ESfMData(flags)));
        BOOST_CHECK_EQUAL( sfmData.views.size(), sfmJsonToABC.views.size());
        BOOST_CHECK_EQUAL( sfmData.getPoses().size(), sfmJsonToABC.getPoses().size());
        BOOST_CHECK_EQUAL( sfmData.intrinsics.size(), sfmJsonToABC.intrinsics.size());
        BOOST_CHECK_EQUAL( sfmData.structure.size(), sfmJsonToABC.structure.size());
        BOOST_CHECK_EQUAL( sfmData.control_points.size(), sfmJsonToABC.control_points.size());
    }

    // Export as ABC
    const std::string abcFile3 = "jsonToABC.abc";
    {
        BOOST_CHECK(Save(
        sfmJsonToABC,
        abcFile3.c_str(),
        ESfMData(flags)));
    }

    // Reload
    SfMData sfmJsonToABC2;
    {
        BOOST_CHECK(Load(sfmJsonToABC2, abcFile3, ESfMData(flags)));
        BOOST_CHECK_EQUAL( sfmData.views.size(), sfmJsonToABC2.views.size());
        BOOST_CHECK_EQUAL( sfmData.getPoses().size(), sfmJsonToABC2.getPoses().size());
        BOOST_CHECK_EQUAL( sfmData.intrinsics.size(), sfmJsonToABC2.intrinsics.size());
        BOOST_CHECK_EQUAL( sfmData.structure.size(), sfmJsonToABC2.structure.size());
        BOOST_CHECK_EQUAL( sfmData.control_points.size(), sfmJsonToABC2.control_points.size());
    }

    // Export as ABC
    const std::string abcFile4 = "jsonToABC2.abc";
    {
        BOOST_CHECK(Save(
        sfmJsonToABC2,
        abcFile4.c_str(),
        ESfMData(flags)));
    }

    // Reload
    SfMData sfmJsonToABC3;
    {
        BOOST_CHECK(Load(sfmJsonToABC3, abcFile4, ESfMData(flags)));
        BOOST_CHECK_EQUAL( sfmData.views.size(), sfmJsonToABC3.views.size());
        BOOST_CHECK_EQUAL( sfmData.getPoses().size(), sfmJsonToABC3.getPoses().size());
        BOOST_CHECK_EQUAL( sfmData.intrinsics.size(), sfmJsonToABC3.intrinsics.size());
        BOOST_CHECK_EQUAL( sfmData.structure.size(), sfmJsonToABC3.structure.size());
        BOOST_CHECK_EQUAL( sfmData.control_points.size(), sfmJsonToABC3.control_points.size());
    }

    // Export as JSON
    const std::string jsonFile4 = "jsonToABC2.sfm";
    {
        BOOST_CHECK(Save(
        sfmJsonToABC3,
        jsonFile4.c_str(),
        ESfMData(flags)));
    }

}
