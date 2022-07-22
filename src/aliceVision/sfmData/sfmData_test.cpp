
#include <aliceVision/sfmData/SfMData.hpp>

#define BOOST_TEST_MODULE sfmData

#include <boost/test/unit_test.hpp>

using namespace aliceVision;

BOOST_AUTO_TEST_CASE(SfMData_InternalFolders)
{
  vfs::filesystem fs;

  const std::string filename = "InternalFolders.sfm";
  sfmData::SfMData sfmData;

  // add relative features/matches folders with duplicates
  std::string refFolder("..");
  sfmData.addFeaturesFolders(fs, {refFolder, refFolder});
  sfmData.addMatchesFolders(fs, {refFolder, refFolder});
  auto featuresFolders = sfmData.getFeaturesFolders(fs);
  auto matchesFolders = sfmData.getMatchesFolders(fs);
  // ensure duplicates were removed
  BOOST_CHECK_EQUAL(featuresFolders.size(), 1);
  BOOST_CHECK_EQUAL(matchesFolders.size(), 1);
  // sfmData has no absolute path set, folders are still in relative form
  BOOST_CHECK_EQUAL(featuresFolders[0], refFolder);
  BOOST_CHECK_EQUAL(matchesFolders[0], refFolder);

  // set absolutePath to current filename
  sfmData.setAbsolutePath(fs, fs.absolute(filename).string());
  featuresFolders = sfmData.getFeaturesFolders(fs);
  matchesFolders = sfmData.getMatchesFolders(fs);
  // internal folders were kept...
  BOOST_CHECK_EQUAL(featuresFolders.size(), 1);
  BOOST_CHECK_EQUAL(matchesFolders.size(), 1);
  // ... and are now absolute paths
  BOOST_CHECK(vfs::path(featuresFolders[0]).is_absolute());
  BOOST_CHECK(fs.equivalent(featuresFolders[0], refFolder));
  BOOST_CHECK(vfs::path(matchesFolders[0]).is_absolute());
  BOOST_CHECK(fs.equivalent(matchesFolders[0], refFolder));

  // update sfm absolute path to be in parent/parent folder
  vfs::path otherFolder = vfs::path("../..");
  std::string updatedFilename = ( otherFolder / filename).string();
  sfmData.setAbsolutePath(fs, updatedFilename);
  // internal folders still reference the same folder as before
  BOOST_CHECK(fs.equivalent(featuresFolders[0], refFolder));
  BOOST_CHECK(fs.equivalent(matchesFolders[0], refFolder));
  BOOST_CHECK_EQUAL(sfmData.getRelativeFeaturesFolders()[0], fs.relative(refFolder, otherFolder).string());
  BOOST_CHECK_EQUAL(sfmData.getRelativeMatchesFolders()[0], fs.relative(refFolder, otherFolder).string());
}

