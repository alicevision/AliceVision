
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/vfs/filesystem.hpp>

#define BOOST_TEST_MODULE sfmData

#include <boost/test/unit_test.hpp>

using namespace aliceVision;

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
  sfmData.setAbsolutePath(vfs::absolute(filename).string());
  featuresFolders = sfmData.getFeaturesFolders();
  matchesFolders = sfmData.getMatchesFolders();
  // internal folders were kept...
  BOOST_CHECK_EQUAL(featuresFolders.size(), 1);
  BOOST_CHECK_EQUAL(matchesFolders.size(), 1);
  // ... and are now absolute paths
  BOOST_CHECK(vfs::path(featuresFolders[0]).is_absolute());
  BOOST_CHECK(vfs::equivalent(featuresFolders[0], refFolder));
  BOOST_CHECK(vfs::path(matchesFolders[0]).is_absolute());
  BOOST_CHECK(vfs::equivalent(matchesFolders[0], refFolder));

  // update sfm absolute path to be in parent/parent folder
  vfs::path otherFolder = vfs::path("../..");
  std::string updatedFilename = ( otherFolder / filename).string();
  sfmData.setAbsolutePath(updatedFilename);
  // internal folders still reference the same folder as before
  BOOST_CHECK(vfs::equivalent(featuresFolders[0], refFolder));
  BOOST_CHECK(vfs::equivalent(matchesFolders[0], refFolder));
  BOOST_CHECK_EQUAL(sfmData.getRelativeFeaturesFolders()[0], vfs::relative(refFolder, otherFolder).string());
  BOOST_CHECK_EQUAL(sfmData.getRelativeMatchesFolders()[0], vfs::relative(refFolder, otherFolder).string());
}

