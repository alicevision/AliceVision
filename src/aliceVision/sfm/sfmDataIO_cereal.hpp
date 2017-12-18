// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfm/sfmDataIO.hpp>
#include <aliceVision/stl/split.hpp>

#include <cereal/archives/portable_binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

#include <cereal/types/map.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/string.hpp>

#include <iomanip>
#include <fstream>

namespace aliceVision {
namespace sfm {

template <
// JSONInputArchive/ ...
typename archiveType
>
bool Load_Cereal(
  SfMData & data,
  const std::string & filename,
  ESfMData flags_part)
{
  const bool bBinary = stlplus::extension_part(filename) == "bin";

  // List which part of the file must be considered
  const bool b_views = (flags_part & VIEWS) == VIEWS;
  const bool b_intrinsics = (flags_part & INTRINSICS) == INTRINSICS;
  const bool b_extrinsics = (flags_part & EXTRINSICS) == EXTRINSICS;
  const bool b_structure = (flags_part & STRUCTURE) == STRUCTURE;
  // TODO: check observations
  const bool b_control_point = (flags_part & CONTROL_POINTS) == CONTROL_POINTS;

  //Create the stream and check it is ok
  std::ifstream stream(filename.c_str(), std::ios::binary | std::ios::in);
  if (!stream.is_open())
    return false;

  // Data serialization
  try
  {
    archiveType archive(stream);

    std::string versionStr;
    archive(cereal::make_nvp("sfm_data_version", versionStr));
    
    std::vector<int> versions;
    {
      std::vector<std::string> versionsStr;
      stl::split(versionStr, ".", versionsStr);
      for(auto& v: versionsStr)
        versions.push_back(std::stoi(v));
    }

    if(versions[1] > 2) // version > 0.2
    {
      archive(cereal::make_nvp("featuresFolders", data._featuresFolders));
      archive(cereal::make_nvp("matchesFolders", data._matchesFolders));
    }
    
    if (b_views)
      archive(cereal::make_nvp("views", data.views));
    else if (bBinary) // Binary file require read all the member
      archive(cereal::make_nvp("views", Views()));


    if (b_intrinsics)
      archive(cereal::make_nvp("intrinsics", data.intrinsics));
    else if (bBinary) // Binary file require read all the member
      archive(cereal::make_nvp("intrinsics", Intrinsics()));

    if (b_extrinsics)
    {
      archive(cereal::make_nvp("extrinsics", data.GetPoses()));
      archive(cereal::make_nvp("rigs", data.getRigs()));
    }
    else if (bBinary)
    {
      // Binary file require read all the member
      archive(cereal::make_nvp("extrinsics", Poses()));
      archive(cereal::make_nvp("rigs", Rigs()));
    }

    if (b_structure)
      archive(cereal::make_nvp("structure", data.structure));
    else if (bBinary) // Binary file require read all the member
      archive(cereal::make_nvp("structure", Landmarks()));

    if (versions[1] > 1) // version > 0.1
    {
      if (b_control_point)
        archive(cereal::make_nvp("control_points", data.control_points));
      else if (bBinary) // Binary file require read all the member
        archive(cereal::make_nvp("control_points", Landmarks()));
    }
  }
  catch (const cereal::Exception & e)
  {
    ALICEVISION_LOG_WARNING(e.what());
    return false;
  }
  stream.close();
  return true;
}

template <
// JSONOutputArchive/ ...
typename archiveType
>
bool Save_Cereal(
  const SfMData & data,
  const std::string & filename,
  ESfMData flags_part)
{
  // List which part of the file must be considered
  const bool b_views = (flags_part & VIEWS) == VIEWS;
  const bool b_intrinsics = (flags_part & INTRINSICS) == INTRINSICS;
  const bool b_extrinsics = (flags_part & EXTRINSICS) == EXTRINSICS;
  const bool b_structure = (flags_part & STRUCTURE) == STRUCTURE;
  // TODO: check observations
  const bool b_control_point = (flags_part & CONTROL_POINTS) == CONTROL_POINTS;

  //Create the stream and check it is ok
  std::ofstream stream(filename.c_str(), std::ios::binary | std::ios::out);
  if (!stream.is_open())
    return false;

  // Data serialization
  {
    archiveType archive(stream);
    // since AliceVision 0.9, the sfm_data version 0.2 is introduced
    //  - it adds control_points storage
    const std::string version = "0.3.1";
    archive(cereal::make_nvp("sfm_data_version", version));
    archive(cereal::make_nvp("featuresFolders", data._featuresFolders));
    archive(cereal::make_nvp("matchesFolders", data._matchesFolders));

    if (b_views)
      archive(cereal::make_nvp("views", data.views));
    else
      archive(cereal::make_nvp("views", Views()));

    if (b_intrinsics)
      archive(cereal::make_nvp("intrinsics", data.intrinsics));
    else
      archive(cereal::make_nvp("intrinsics", Intrinsics()));

    if (b_extrinsics)
    {
      archive(cereal::make_nvp("extrinsics", data.GetPoses()));
      archive(cereal::make_nvp("rigs", data.getRigs()));
    }
    else
    {
      archive(cereal::make_nvp("extrinsics", Poses()));
      archive(cereal::make_nvp("rigs", Rigs()));
    }

    // Structure -> See for export in another file
    if (b_structure)
      archive(cereal::make_nvp("structure", data.structure));
    else
      archive(cereal::make_nvp("structure", Landmarks()));

    if (b_control_point)
      archive(cereal::make_nvp("control_points", data.control_points));
    else
      archive(cereal::make_nvp("control_points", Landmarks()));

  }
  stream.close();
  return true;
}

} // namespace sfm
} // namespace aliceVision
