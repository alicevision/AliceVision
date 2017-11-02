// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "EasyExifIO.hpp"
#include <aliceVision/system/Logger.hpp>

#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"

#include <iostream>
#include <memory>

#define BOOST_TEST_MODULE EasyExifIO
#include <boost/test/included/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

using namespace std;
using namespace aliceVision;
using namespace aliceVision::exif;

const std::string sImg =
  stlplus::folder_part(
  stlplus::folder_part(
  stlplus::folder_up(string(THIS_SOURCE_DIR))))
    + "/samples/imageData/exifTest/100_7100.JPG";

BOOST_AUTO_TEST_CASE(ExifIO_easyexif_ReadData_invalidFile)
{
  std::unique_ptr<ExifIO> exif_io ( new EasyExifIO( "tmp.jpg" ) );

  BOOST_CHECK(exif_io->doesHaveExifInfo() == false);
}

BOOST_AUTO_TEST_CASE(ExifIO_easyexif_ReadData)
{
  std::unique_ptr<ExifIO> exif_io(new EasyExifIO(sImg));

  ALICEVISION_LOG_DEBUG("Read Metadata of file: " << sImg);

  ALICEVISION_LOG_DEBUG("-----");
  ALICEVISION_LOG_DEBUG(exif_io->getExifDataString());
  ALICEVISION_LOG_DEBUG("-----");

  BOOST_CHECK(exif_io->doesHaveExifInfo());

  BOOST_CHECK(exif_io->getBrand() == "EASTMAN KODAK COMPANY");
  BOOST_CHECK(exif_io->getModel() == "KODAK Z612 ZOOM DIGITAL CAMERA");

  BOOST_CHECK_EQUAL(2832, exif_io->getWidth());
  BOOST_CHECK_EQUAL(2128, exif_io->getHeight());
  BOOST_CHECK_SMALL(5.85 - exif_io->getFocal(), 1e-2);

  BOOST_CHECK(exif_io->getImageUniqueID() == "");
  BOOST_CHECK(exif_io->getSerialNumber() == "");
  BOOST_CHECK(exif_io->getLensModel() == "");
  BOOST_CHECK(exif_io->getLensSerialNumber() == "");
  BOOST_CHECK(exif_io->getDateTime() == "");
  BOOST_CHECK(exif_io->getDateTimeOriginal() == "2010:10:12 14:43:07");
  BOOST_CHECK(exif_io->getDateTimeDigitized() == "2010:10:12 14:43:07");
  BOOST_CHECK(exif_io->getSubSecTimeOriginal() == "");
}
