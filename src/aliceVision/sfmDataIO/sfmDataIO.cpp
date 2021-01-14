// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "sfmDataIO.hpp"
#include <aliceVision/config.hpp>
#include <aliceVision/stl/mapUtils.hpp>
#include <aliceVision/sfmDataIO/jsonIO.hpp>
#include <aliceVision/sfmDataIO/plyIO.hpp>
#include <aliceVision/sfmDataIO/bafIO.hpp>
#include <aliceVision/sfmDataIO/gtIO.hpp>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
#include <aliceVision/sfmDataIO/AlembicExporter.hpp>
#include <aliceVision/sfmDataIO/AlembicImporter.hpp>
#endif

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace aliceVision {
namespace sfmDataIO {

///Check that each pose have a valid intrinsic and pose id in the existing View ids
bool ValidIds(const sfmData::SfMData& sfmData, ESfMData partFlag)
{
  const bool bCheck_Intrinsic = (partFlag & INTRINSICS);
  const bool bCheck_Extrinsic = (partFlag & EXTRINSICS);

  std::set<IndexT> intrinsicIdsDeclared;
  transform(sfmData.getIntrinsics().begin(), sfmData.getIntrinsics().end(),
    std::inserter(intrinsicIdsDeclared, intrinsicIdsDeclared.begin()), stl::RetrieveKey());

  std::set<IndexT> extrinsicIdsDeclared; //unique so can use a set
  transform(sfmData.getPoses().begin(), sfmData.getPoses().end(),
    std::inserter(extrinsicIdsDeclared, extrinsicIdsDeclared.begin()), stl::RetrieveKey());

  // Collect id_intrinsic and id_extrinsic referenced from views
  std::set<IndexT> intrinsicIdsReferenced;
  std::set<IndexT> extrinsicIdsReferenced;
  for(const auto& v: sfmData.getViews())
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

bool Load(sfmData::SfMData& sfmData, const std::string& filename, ESfMData partFlag)
{
  const std::string extension = fs::extension(filename);
  bool status = false;

  if(extension == ".sfm" || extension == ".json") // JSON File
  {
    status = loadJSON(sfmData, filename, partFlag);
  }
  else if (extension == ".abc") // Alembic
  {
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
      AlembicImporter(filename).populateSfM(sfmData, partFlag);
      status = true;
#else
      ALICEVISION_THROW_ERROR("Cannot load the ABC file: \"" << filename << "\", AliceVision is built without Alembic support.");
#endif
  }
  else if(fs::is_directory(filename))
  {
    status = readGt(filename, sfmData);
  }
  else // It is not a folder or known format, return false
  {
    ALICEVISION_LOG_ERROR("Unknown input SfM data format: '" << extension << "'");
    return false;
  }

  if(status)
    sfmData.setAbsolutePath(filename);

  // Assert that loaded intrinsics | extrinsics are linked to valid view
  if(status && (partFlag & VIEWS) && ((partFlag & INTRINSICS) || (partFlag & EXTRINSICS)))
    return ValidIds(sfmData, partFlag);

  return status;
}

bool Save(const sfmData::SfMData& sfmData, const std::string& filename, ESfMData partFlag)
{
  const fs::path bPath = fs::path(filename);
  const std::string extension = bPath.extension().string();
  const std::string tmpPath = (bPath.parent_path() / bPath.stem()).string() + "." + fs::unique_path().string() + extension;
  bool status = false;

  if(extension == ".sfm" || extension == ".json") // JSON File
  {
    status = saveJSON(sfmData, tmpPath, partFlag);
  }
  else if(extension == ".ply") // Polygon File
  {
    status = savePLY(sfmData, tmpPath, partFlag);
  }
  else if (extension == ".baf") // Bundle Adjustment File
  {
    status = saveBAF(sfmData, tmpPath, partFlag);
  }
  else if (extension == ".abc") // Alembic
  {
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
      AlembicExporter(tmpPath).addSfM(sfmData, partFlag);
      status = true;
#else
      ALICEVISION_THROW_ERROR("Cannot save the ABC file: \"" << filename
                                                             << "\", AliceVision is built without Alembic support.");
#endif
  }
  else
  {
    ALICEVISION_LOG_ERROR("Cannot save the SfM data file: '" << filename << "'."
                          << std::endl << "The file extension is not recognized.");
    return false;
  }

  // rename temporay filename
  if(status)
    fs::rename(tmpPath, filename);

  return status;
}

} // namespace sfmDataIO
} // namespace aliceVision


