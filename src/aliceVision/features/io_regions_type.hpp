// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef IO_REGIONS_TYPE_HPP
#define IO_REGIONS_TYPE_HPP

#include "aliceVision/features/features.hpp"
#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"
#include <cereal/archives/json.hpp>

#include <fstream>
#include <vector>

namespace aliceVision {
namespace features {

// Init the regions_type from an image describer file (used for regions loading)
static inline std::unique_ptr<features::Regions> Init_region_type_from_file
(
  const std::string & sImage_describer_file
)
{
  using namespace aliceVision::features;
  std::unique_ptr<Regions> regions_type;
  if (stlplus::is_file(sImage_describer_file))
  {
    // Dynamically load the regions type from the file
    std::ifstream stream(sImage_describer_file.c_str());
    if (stream.is_open())
    {
      cereal::JSONInputArchive archive(stream);
      archive(cereal::make_nvp("regions_type", regions_type));
    }
  }
  else // By default init a SIFT regions type (keep compatibility)
  {
    regions_type.reset(new features::SIFT_Regions());
  }
  return regions_type;
}

} // namespace features
} // namespace aliceVision

#endif // IO_REGIONS_TYPE_HPP
