// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/sfm/sfmDataIO.hpp"

#include "aliceVision/config.hpp"
#include "aliceVision/stl/mapUtils.hpp"
#include "aliceVision/sfm/sfmDataIO_json.hpp"
#include "aliceVision/sfm/sfmDataIO_cereal.hpp"
#include "aliceVision/sfm/sfmDataIO_ply.hpp"
#include "aliceVision/sfm/sfmDataIO_baf.hpp"
#include "aliceVision/sfm/sfmDataIO_gt.hpp"

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
#include "aliceVision/sfm/AlembicExporter.hpp"
#include "aliceVision/sfm/AlembicImporter.hpp"
#endif

#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"

namespace aliceVision {
namespace sfm {

///Check that each pose have a valid intrinsic and pose id in the existing View ids
bool ValidIds(const SfMData& sfmData, ESfMData partFlag)
{
  const bool bCheck_Intrinsic = (partFlag & INTRINSICS);
  const bool bCheck_Extrinsic = (partFlag & EXTRINSICS);

  std::set<IndexT> intrinsicIdsDeclared;
  transform(sfmData.GetIntrinsics().begin(), sfmData.GetIntrinsics().end(),
    std::inserter(intrinsicIdsDeclared, intrinsicIdsDeclared.begin()), stl::RetrieveKey());

  std::set<IndexT> extrinsicIdsDeclared; //unique so can use a set
  transform(sfmData.GetPoses().begin(), sfmData.GetPoses().end(),
    std::inserter(extrinsicIdsDeclared, extrinsicIdsDeclared.begin()), stl::RetrieveKey());

  // Collect id_intrinsic and id_extrinsic referenced from views
  std::set<IndexT> intrinsicIdsReferenced;
  std::set<IndexT> extrinsicIdsReferenced;
  for(const auto& v: sfmData.GetViews())
  {
    const IndexT id_intrinsic = v.second.get()->getIntrinsicId();
    intrinsicIdsReferenced.insert(id_intrinsic);

    const IndexT id_pose = v.second.get()->getPoseId();
    extrinsicIdsReferenced.insert(id_pose);
  }

  // We may have some views with undefined Intrinsics,
  // so erase the UndefinedIndex value if exist.
  intrinsicIdsReferenced.erase(UndefinedIndexT);
  extrinsicIdsReferenced.erase(UndefinedIndexT);

  // Check if defined intrinsic & extrinsic are at least connected to views
  bool bRet = true;
  if(bCheck_Intrinsic && intrinsicIdsDeclared != intrinsicIdsReferenced)
  {
    ALICEVISION_LOG_WARNING("The number of intrinsics is incoherent:");
    ALICEVISION_LOG_WARNING(intrinsicIdsDeclared.size() << " intrinsics declared and " << intrinsicIdsReferenced.size() << " intrinsics used.");
    std::set<IndexT> undefinedIntrinsicIds;
    // undefinedIntrinsicIds = intrinsicIdsReferenced - intrinsicIdsDeclared
    std::set_difference(intrinsicIdsReferenced.begin(), intrinsicIdsReferenced.end(),
                        intrinsicIdsDeclared.begin(), intrinsicIdsDeclared.end(), 
                        std::inserter(undefinedIntrinsicIds, undefinedIntrinsicIds.begin()));
    // If undefinedIntrinsicIds is not empty,
    // some intrinsics are used in Views but never declared.
    // So the file structure is invalid and may create troubles.
    if(!undefinedIntrinsicIds.empty())
      bRet = false; // error
  }
  
  if (bCheck_Extrinsic && extrinsicIdsDeclared != extrinsicIdsReferenced)
  {
    ALICEVISION_LOG_TRACE(extrinsicIdsDeclared.size() << " extrinsics declared and " << extrinsicIdsReferenced.size() << " extrinsics used.");
    std::set<IndexT> undefinedExtrinsicIds;
    // undefinedExtrinsicIds = extrinsicIdsReferenced - extrinsicIdsDeclared
    std::set_difference(extrinsicIdsDeclared.begin(), extrinsicIdsDeclared.end(),
                        extrinsicIdsReferenced.begin(), extrinsicIdsReferenced.end(),
                        std::inserter(undefinedExtrinsicIds, undefinedExtrinsicIds.begin()));
    // If undefinedExtrinsicIds is not empty,
    // some extrinsics are used in Views but never declared.
    // So the file structure is invalid and may create troubles.
    if(!undefinedExtrinsicIds.empty())
      bRet = false; // error
  }

  return bRet;
}

bool Load(SfMData& sfmData, const std::string& filename, ESfMData partFlag)
{
  bool bStatus = false;
  const std::string ext = stlplus::extension_part(filename);
  if(ext == "sfm")
    bStatus = loadJSON(sfmData, filename, partFlag);
  else if (ext == "json")
    bStatus = Load_Cereal<cereal::JSONInputArchive>(sfmData, filename, partFlag);
  else if (ext == "bin")
    bStatus = Load_Cereal<cereal::PortableBinaryInputArchive>(sfmData, filename, partFlag);
  else if (ext == "xml")
    bStatus = Load_Cereal<cereal::XMLInputArchive>(sfmData, filename, partFlag);
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
  else if (ext == "abc") {
    AlembicImporter(filename).populateSfM(sfmData, partFlag);
    bStatus = true;
  }
#endif // ALICEVISION_HAVE_ALEMBIC
  else if (stlplus::folder_exists(filename))
  {
    bStatus = readGt(filename, sfmData);
  }
  // It is not a folder or known format, return false
  else
  {
    ALICEVISION_LOG_WARNING("Unknown sfm_data input format: " << ext);
    return false;
  }

  // Assert that loaded intrinsics | extrinsics are linked to valid view
  if(bStatus &&
     (partFlag & VIEWS) &&
     ((partFlag & INTRINSICS) || (partFlag & EXTRINSICS)))
  {
    return ValidIds(sfmData, partFlag);
  }
  return bStatus;
}

bool Save(const SfMData& sfmData, const std::string& filename, ESfMData partFlag)
{
  const std::string ext = stlplus::extension_part(filename);
  if(ext == "sfm")
    return saveJSON(sfmData, filename, partFlag);
  else if (ext == "json")
    return Save_Cereal<cereal::JSONOutputArchive>(sfmData, filename, partFlag);
  else if (ext == "bin")
    return Save_Cereal<cereal::PortableBinaryOutputArchive>(sfmData, filename, partFlag);
  else if (ext == "xml")
    return Save_Cereal<cereal::XMLOutputArchive>(sfmData, filename, partFlag);
  else if (ext == "ply")
    return Save_PLY(sfmData, filename, partFlag);
  else if (ext == "baf") // Bundle Adjustment file
    return Save_BAF(sfmData, filename, partFlag);
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
  else if (ext == "abc") // Alembic
  {
    aliceVision::sfm::AlembicExporter(filename).addSfM(sfmData, partFlag);
    return true;
  }
#endif // ALICEVISION_HAVE_ALEMBIC
  ALICEVISION_LOG_WARNING("ERROR: Cannot save the SfM Data: " << filename << ".\n"
            << "The file extension is not recognized.");
  return false;
}

} // namespace sfm
} // namespace aliceVision


