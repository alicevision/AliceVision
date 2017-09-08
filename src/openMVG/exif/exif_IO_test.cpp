// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "exif_IO_EasyExif.hpp"
#include <aliceVision/system/Logger.hpp>

#include "testing/testing.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <iostream>
#include <memory>

using namespace std;
using namespace aliceVision;
using namespace aliceVision::exif;

const std::string sImg =
  stlplus::folder_part(
  stlplus::folder_part(
  stlplus::folder_up(string(THIS_SOURCE_DIR))))
    + "/aliceVision_Samples/imageData/Exif_Test/100_7100.JPG";

TEST(Matching, Exif_IO_easyexif_ReadData_invalidFile)
{
  std::unique_ptr<Exif_IO> exif_io ( new Exif_IO_EasyExif( "tmp.jpg" ) );

  EXPECT_FALSE(exif_io->doesHaveExifInfo());
}

TEST(Matching, Exif_IO_easyexif_ReadData)
{
  std::unique_ptr<Exif_IO> exif_io(new Exif_IO_EasyExif(sImg));

  OPENMVG_LOG_DEBUG("Read Metadata of file: " << sImg);

  OPENMVG_LOG_DEBUG("-----");
  OPENMVG_LOG_DEBUG(exif_io->getExifDataString());
  OPENMVG_LOG_DEBUG("-----");

  EXPECT_TRUE(exif_io->doesHaveExifInfo());

  EXPECT_EQ("EASTMAN KODAK COMPANY", exif_io->getBrand());
  EXPECT_EQ("KODAK Z612 ZOOM DIGITAL CAMERA", exif_io->getModel());

  EXPECT_EQ(2832, exif_io->getWidth());
  EXPECT_EQ(2128, exif_io->getHeight());
  EXPECT_NEAR(5.85, exif_io->getFocal(), 1e-2);

  EXPECT_EQ("", exif_io->getImageUniqueID());
  EXPECT_EQ("", exif_io->getSerialNumber());
  EXPECT_EQ("", exif_io->getLensModel());
  EXPECT_EQ("", exif_io->getLensSerialNumber());
  EXPECT_EQ("", exif_io->getDateTime());
  EXPECT_EQ("2010:10:12 14:43:07", exif_io->getDateTimeOriginal());
  EXPECT_EQ("2010:10:12 14:43:07", exif_io->getDateTimeDigitized());
  EXPECT_EQ("", exif_io->getSubSecTimeOriginal());
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */

