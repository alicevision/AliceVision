#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmData/LandmarkTable.hpp>
#include <aliceVision/camera/Pinhole.hpp>

#define BOOST_TEST_MODULE sfmData

#include <boost/test/unit_test.hpp>

#include <filesystem>

using namespace aliceVision;
namespace fs = std::filesystem;

BOOST_AUTO_TEST_CASE(SfMData_InternalFolders)
{
    const std::string filename = "InternalFolders.sfm";
    sfmData::SfMData sfmData;

    // add relative features/matches folders with duplicates
    std::string refFolder("..");
    sfmData.addFeaturesFolders({refFolder, refFolder});
    sfmData.addMatchesFolders({refFolder, refFolder});
    auto featuresFolders = sfmData.getFeaturesFolders();
    auto matchesFolders = sfmData.getMatchesFolders();
    // ensure duplicates were removed
    BOOST_CHECK_EQUAL(featuresFolders.size(), 1);
    BOOST_CHECK_EQUAL(matchesFolders.size(), 1);
    // sfmData has no absolute path set, folders are still in relative form
    BOOST_CHECK_EQUAL(featuresFolders[0], refFolder);
    BOOST_CHECK_EQUAL(matchesFolders[0], refFolder);

    // set absolutePath to current filename
    sfmData.setAbsolutePath(fs::absolute(filename).string());
    featuresFolders = sfmData.getFeaturesFolders();
    matchesFolders = sfmData.getMatchesFolders();
    // internal folders were kept...
    BOOST_CHECK_EQUAL(featuresFolders.size(), 1);
    BOOST_CHECK_EQUAL(matchesFolders.size(), 1);
    // ... and are now absolute paths
    BOOST_CHECK(fs::path(featuresFolders[0]).is_absolute());
    BOOST_CHECK(fs::equivalent(featuresFolders[0], refFolder));
    BOOST_CHECK(fs::path(matchesFolders[0]).is_absolute());
    BOOST_CHECK(fs::equivalent(matchesFolders[0], refFolder));

    // update sfm absolute path to be in parent/parent folder
    fs::path otherFolder = fs::path("../..");
    std::string updatedFilename = (otherFolder / filename).string();
    sfmData.setAbsolutePath(updatedFilename);
    // internal folders still reference the same folder as before
    BOOST_CHECK(fs::equivalent(featuresFolders[0], refFolder));
    BOOST_CHECK(fs::equivalent(matchesFolders[0], refFolder));
    BOOST_CHECK_EQUAL(sfmData.getRelativeFeaturesFolders()[0], fs::relative(refFolder, otherFolder));
    BOOST_CHECK_EQUAL(sfmData.getRelativeMatchesFolders()[0], fs::relative(refFolder, otherFolder));
}

BOOST_AUTO_TEST_CASE(SfMData_IsFullyReconstructed)
{
    sfmData::SfMData sfmData;

    // Add a view with undefined pose/intrinsic: should not be fully reconstructed
    auto view1 = std::make_shared<sfmData::View>("image1.jpg", 0, aliceVision::UndefinedIndexT, aliceVision::UndefinedIndexT, 100, 100);
    sfmData.getViews().emplace(0, view1);
    BOOST_CHECK(!sfmData.isFullyReconstructed());

    // Add valid intrinsic and pose
    aliceVision::camera::IntrinsicBase::sptr intrinsic = std::make_shared<aliceVision::camera::Pinhole>();
    sfmData.getViews().at(0)->setIntrinsicId(1);
    sfmData.getViews().at(0)->setPoseId(2);
    sfmData.getIntrinsics().emplace(1, intrinsic);
    sfmData.getPoses().emplace(2, std::make_shared<sfmData::CameraPose>());
    BOOST_CHECK(sfmData.isFullyReconstructed());

    // Add another view with missing intrinsic
    auto view2 = std::make_shared<sfmData::View>("image2.jpg", 1, aliceVision::UndefinedIndexT, 3, 100, 100);
    sfmData.getViews().emplace(1, view2);
    sfmData.getPoses().emplace(3, std::make_shared<sfmData::CameraPose>());
    BOOST_CHECK(!sfmData.isFullyReconstructed());

    // Fix intrinsic for view2
    view2->setIntrinsicId(1);
    BOOST_CHECK(sfmData.isFullyReconstructed());
}

BOOST_AUTO_TEST_CASE(SfMData_LandmarkTablePacking)
{
    sfmData::SfMData sfmData;

    sfmData::Landmark landmarkA({1.0, 2.0, 3.0});
    landmarkA.setParallaxRobust(true);
    landmarkA.setLocked(false);
    landmarkA.setState(EEstimatorParameterState::CONSTANT);
    landmarkA.getObservations().emplace(4, sfmData::Observation({10.0, 20.0}, 7, 1.5));
    landmarkA.getObservations().emplace(8, sfmData::Observation({30.0, 40.0}, 9, 2.5));
    sfmData.getLandmarks().emplace(42, landmarkA);

    sfmData::Landmark landmarkB({-1.0, -2.0, -3.0});
    landmarkB.setParallaxRobust(false);
    landmarkB.setLocked(true);
    landmarkB.setState(EEstimatorParameterState::REFINED);
    landmarkB.getObservations().emplace(8, sfmData::Observation({50.0, 60.0}, 11, 3.5));
    sfmData.getLandmarks().emplace(84, landmarkB);

    sfmData._landmarksUncertainty[42] = Vec3(0.1, 0.2, 0.3);

    const sfmData::LandmarkTable table = sfmData::buildLandmarkTable(sfmData, false);

    BOOST_CHECK_EQUAL(table.ids.size(), 2);
    BOOST_CHECK_EQUAL(table.points.size(), 2);
    BOOST_CHECK_EQUAL(table.observationOffsets.size(), 3);
    BOOST_CHECK_EQUAL(table.observationOffsets[0], 0);
    BOOST_CHECK_EQUAL(table.observationOffsets[1], 2);
    BOOST_CHECK_EQUAL(table.observationOffsets[2], 3);
    BOOST_CHECK_EQUAL(table.observationViewIds.size(), 3);
    BOOST_CHECK_EQUAL(table.observationFeatureIds.size(), 3);
    BOOST_CHECK_EQUAL(table.observationXY.size(), 3);
    BOOST_CHECK_EQUAL(table.observationScales.size(), 3);
    BOOST_CHECK_EQUAL(table.observationDepths.size(), 3);

    BOOST_CHECK_EQUAL(table.ids[0], 42);
    BOOST_CHECK_EQUAL(table.ids[1], 84);
    BOOST_CHECK_EQUAL(table.states[0], static_cast<std::uint8_t>(EEstimatorParameterState::CONSTANT));
    BOOST_CHECK_EQUAL(table.states[1], static_cast<std::uint8_t>(EEstimatorParameterState::REFINED));
    BOOST_CHECK_EQUAL(table.flags[0], 1u);
    BOOST_CHECK_EQUAL(table.flags[1], 2u);
}

BOOST_AUTO_TEST_CASE(SfMData_LandmarkTableViewIndex)
{
    sfmData::SfMData sfmData;

    sfmData::Landmark landmarkA({0.0, 0.0, 0.0});
    landmarkA.getObservations().emplace(5, sfmData::Observation({1.0, 2.0}, 1, 1.0));
    landmarkA.getObservations().emplace(7, sfmData::Observation({3.0, 4.0}, 2, 1.0));
    sfmData.getLandmarks().emplace(1, landmarkA);

    sfmData::Landmark landmarkB({0.0, 0.0, 0.0});
    landmarkB.getObservations().emplace(7, sfmData::Observation({5.0, 6.0}, 3, 1.0));
    sfmData.getLandmarks().emplace(2, landmarkB);

    const sfmData::LandmarkTable table = sfmData::buildLandmarkTable(sfmData, true);

    BOOST_CHECK_EQUAL(table.viewIds.size(), 2);
    BOOST_CHECK_EQUAL(table.viewIds[0], 5);
    BOOST_CHECK_EQUAL(table.viewIds[1], 7);
    BOOST_CHECK_EQUAL(table.viewObservationOffsets.size(), 3);
    BOOST_CHECK_EQUAL(table.viewObservationOffsets[0], 0);
    BOOST_CHECK_EQUAL(table.viewObservationOffsets[1], 1);
    BOOST_CHECK_EQUAL(table.viewObservationOffsets[2], 3);
    BOOST_CHECK_EQUAL(table.viewObservationIndices.size(), 3);
    BOOST_CHECK_EQUAL(table.viewObservationLandmarkIndices.size(), 3);
}
