// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "ExifIO.hpp"

#include "aliceVision/stl/hash.hpp"

#include <cstdlib>

namespace aliceVision {
namespace exif  {

std::size_t computeUID(const ExifIO& exifReader, const std::string& imageFilename)
{
  std::size_t uid = 0;

  if( !exifReader.getImageUniqueID().empty() ||
      !exifReader.getSerialNumber().empty() ||
      !exifReader.getLensSerialNumber().empty()
    )
  {
    stl::hash_combine(uid, exifReader.getImageUniqueID());
    stl::hash_combine(uid, exifReader.getSerialNumber());
    stl::hash_combine(uid, exifReader.getLensSerialNumber());
  }
  else
  {
    // No metadata to identify the image, fallback to the filename
    stl::hash_combine(uid, imageFilename);
  }
  
  if( !exifReader.getDateTimeOriginal().empty() )
  {
    stl::hash_combine(uid, exifReader.getDateTimeOriginal());
    stl::hash_combine(uid, exifReader.getSubSecTimeOriginal());
  }
  else
  {
    // If no original date/time, fallback to the file date/time
    stl::hash_combine(uid, exifReader.getDateTime());
  }

  stl::hash_combine(uid, exifReader.getWidth());
  stl::hash_combine(uid, exifReader.getHeight());
  
  // Limit to integer to maximize compatibility (like Alembic in Maya)
  uid = std::abs((int) uid);

  return uid;
}

} // namespace exif
} // namespace aliceVision
