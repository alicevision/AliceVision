// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/sfm/sfm_data_io.hpp"

#include "aliceVision/stl/mapUtils.hpp"
#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"

#include "aliceVision/sfm/sfm_data_io_cereal.hpp"
#include "aliceVision/sfm/sfm_data_io_ply.hpp"
#include "aliceVision/sfm/sfm_data_io_baf.hpp"
#include "aliceVision/sfm/sfm_data_io_gt.hpp"

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
#include "aliceVision/sfm/AlembicExporter.hpp"
#include "aliceVision/sfm/AlembicImporter.hpp"
#endif

#include "aliceVision/config.hpp"

namespace aliceVision {
namespace sfm {

///Check that each pose have a valid intrinsic and pose id in the existing View ids
bool ValidIds(const SfM_Data & sfm_data, ESfM_Data flags_part)
{
  const bool bCheck_Intrinsic = (flags_part & INTRINSICS);
  const bool bCheck_Extrinsic = (flags_part & EXTRINSICS);

  std::set<IndexT> intrinsicIdsDeclared;
  transform(sfm_data.GetIntrinsics().begin(), sfm_data.GetIntrinsics().end(),
    std::inserter(intrinsicIdsDeclared, intrinsicIdsDeclared.begin()), stl::RetrieveKey());

  std::set<IndexT> extrinsicIdsDeclared; //unique so can use a set
  transform(sfm_data.GetPoses().begin(), sfm_data.GetPoses().end(),
    std::inserter(extrinsicIdsDeclared, extrinsicIdsDeclared.begin()), stl::RetrieveKey());

  // Collect id_intrinsic and id_extrinsic referenced from views
  std::set<IndexT> intrinsicIdsReferenced;
  std::set<IndexT> extrinsicIdsReferenced;
  for(const auto& v: sfm_data.GetViews())
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
    ALICEVISION_LOG_WARNING("The number of extrinsics is incoherent:");
    ALICEVISION_LOG_WARNING(extrinsicIdsDeclared.size() << " extrinsics declared and " << extrinsicIdsReferenced.size() << " extrinsics used.");
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

bool Load(SfM_Data & sfm_data, const std::string & filename, ESfM_Data flags_part)
{
  bool bStatus = false;
  const std::string ext = stlplus::extension_part(filename);
  if (ext == "json")
    bStatus = Load_Cereal<cereal::JSONInputArchive>(sfm_data, filename, flags_part);
  else if (ext == "bin")
    bStatus = Load_Cereal<cereal::PortableBinaryInputArchive>(sfm_data, filename, flags_part);
  else if (ext == "xml")
    bStatus = Load_Cereal<cereal::XMLInputArchive>(sfm_data, filename, flags_part);
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
  else if (ext == "abc") {
    aliceVision::sfm::AlembicImporter(filename).populate(sfm_data, flags_part);
    bStatus = true;
  }
#endif // ALICEVISION_HAVE_ALEMBIC
  else if (stlplus::folder_exists(filename))
  {
    bStatus = readGt(filename, sfm_data);
  }
  // It is not a folder or known format, return false
  else
  {
    ALICEVISION_LOG_WARNING("Unknown sfm_data input format: " << ext);
    return false;
  }

  // Assert that loaded intrinsics | extrinsics are linked to valid view
  if(bStatus &&
     (flags_part & VIEWS) &&
     ((flags_part & INTRINSICS) || (flags_part & EXTRINSICS)))
  {
    return ValidIds(sfm_data, flags_part);
  }
  return bStatus;
}

bool Save(const SfM_Data & sfm_data, const std::string & filename, ESfM_Data flags_part)
{
  const std::string ext = stlplus::extension_part(filename);
  if (ext == "json")
    return Save_Cereal<cereal::JSONOutputArchive>(sfm_data, filename, flags_part);
  else if (ext == "bin")
    return Save_Cereal<cereal::PortableBinaryOutputArchive>(sfm_data, filename, flags_part);
  else if (ext == "xml")
    return Save_Cereal<cereal::XMLOutputArchive>(sfm_data, filename, flags_part);
  else if (ext == "ply")
    return Save_PLY(sfm_data, filename, flags_part);
  else if (ext == "baf") // Bundle Adjustment file
    return Save_BAF(sfm_data, filename, flags_part);
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
  else if (ext == "abc") // Alembic
  {
    aliceVision::sfm::AlembicExporter(filename).add(sfm_data, flags_part);
    return true;
  }
#endif // ALICEVISION_HAVE_ALEMBIC
  ALICEVISION_LOG_WARNING("ERROR: Cannot save the SfM Data: " << filename << ".\n"
            << "The file extension is not recognized.");
  return false;
}

} // namespace sfm
} // namespace aliceVision


