#include <aliceVision/sfmData/SfMData.hpp>
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
