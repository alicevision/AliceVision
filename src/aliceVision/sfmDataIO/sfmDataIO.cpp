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
#include <aliceVision/utils/filesIO.hpp>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
    #include <aliceVision/sfmDataIO/AlembicExporter.hpp>
    #include <aliceVision/sfmDataIO/AlembicImporter.hpp>
#endif

#include <filesystem>

namespace fs = std::filesystem;

namespace aliceVision {
namespace sfmDataIO {

/// Check that each view references a declared intrinsic
bool validIds(const aliceVision::sfmData::SfMData& sfmData, ESfMData partFlag)
{
    const bool bCheck_Intrinsic = (partFlag & INTRINSICS);
    const bool bCheck_Rig = (partFlag & EXTRINSICS);

    // Collect intrinsic IDs  declared
    std::set<IndexT> intrinsicIdsDeclared;
    transform(sfmData.getIntrinsics().begin(),
              sfmData.getIntrinsics().end(),
              std::inserter(intrinsicIdsDeclared, intrinsicIdsDeclared.begin()),
              stl::RetrieveKey());

    // Collect rig IDs  declared
    std::set<IndexT> rigIdsDeclared;
    transform(sfmData.getRigs().begin(), sfmData.getRigs().end(), std::inserter(rigIdsDeclared, rigIdsDeclared.begin()), stl::RetrieveKey());

    // Collect intrinsic IDs referenced from views
    std::set<IndexT> intrinsicIdsReferenced;
    for (const auto& v : sfmData.getViews())
    {
        const IndexT id_intrinsic = v.second.get()->getIntrinsicId();
        intrinsicIdsReferenced.insert(id_intrinsic);
    }

    // Collect rig IDs referenced from views
    std::set<IndexT> rigIdsReferenced;
    for (const auto& v : sfmData.getViews())
    {
        const IndexT id_rig = v.second.get()->getRigId();
        rigIdsReferenced.insert(id_rig);
    }

    // We may have some views with undefined intrinsic/rig,
    // so erase the UndefinedIndex value if exist.
    intrinsicIdsReferenced.erase(UndefinedIndexT);
    rigIdsReferenced.erase(UndefinedIndexT);

    // Check if defined intrinsics are at least connected to views
    bool bRet = true;
    if (bCheck_Intrinsic && intrinsicIdsDeclared != intrinsicIdsReferenced)
    {
        ALICEVISION_LOG_WARNING("The number of intrinsics is incoherent:");
        ALICEVISION_LOG_WARNING(intrinsicIdsDeclared.size() << " intrinsics declared and " << intrinsicIdsReferenced.size() << " intrinsics used.");
        std::set<IndexT> undefinedIntrinsicIds;
        // undefinedIntrinsicIds = intrinsicIdsReferenced - intrinsicIdsDeclared
        std::set_difference(intrinsicIdsReferenced.begin(),
                            intrinsicIdsReferenced.end(),
                            intrinsicIdsDeclared.begin(),
                            intrinsicIdsDeclared.end(),
                            std::inserter(undefinedIntrinsicIds, undefinedIntrinsicIds.begin()));
        // If undefinedIntrinsicIds is not empty,
        // some intrinsics are used in Views but never declared.
        // So the file structure is invalid and may create troubles.
        if (!undefinedIntrinsicIds.empty())
            bRet = false;  // error
    }

    // Check if defined rigs are at least connected to views
    if (bCheck_Rig && rigIdsDeclared != rigIdsReferenced)
    {
        ALICEVISION_LOG_WARNING("The number of rigs is incoherent:");
        ALICEVISION_LOG_WARNING(rigIdsDeclared.size() << " rigs declared and " << rigIdsReferenced.size() << " rigs used.");
        std::set<IndexT> undefinedRigIds;
        // undefinedRigIds = rigIdsReferenced - rigIdsDeclared
        std::set_difference(rigIdsReferenced.begin(),
                            rigIdsReferenced.end(),
                            rigIdsDeclared.begin(),
                            rigIdsDeclared.end(),
                            std::inserter(undefinedRigIds, undefinedRigIds.begin()));
        // If undefinedRigIds is not empty,
        // some rigs are used in Views but never declared.
        // So the file structure is invalid and may create troubles.
        if (!undefinedRigIds.empty())
            bRet = false;  // error
    }

    return bRet;
}

bool load(aliceVision::sfmData::SfMData& sfmData, const std::string& filename, ESfMData partFlag)
{
    const std::string extension = fs::path(filename).extension().string();
    bool status = false;

    if (extension == ".sfm" || extension == ".json")  // JSON File
    {
        status = loadJSON(sfmData, filename, partFlag);
    }
    else if (extension == ".abc")  // Alembic
    {
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
        AlembicImporter(filename).populateSfM(sfmData, partFlag);
        status = true;
#else
        ALICEVISION_THROW_ERROR("Cannot load the ABC file: \"" << filename << "\", AliceVision is built without Alembic support.");
#endif
    }
    else if (fs::is_directory(filename))
    {
        status = readGt(filename, sfmData);
    }
    else  // It is not a folder or known format, return false
    {
        ALICEVISION_LOG_ERROR("Unknown input SfM data format: '" << extension << "'");
        return false;
    }

    if (status)
        sfmData.setAbsolutePath(filename);

    // Assert that loaded intrinsics are linked to valid view
    if (status && (partFlag & VIEWS) && (partFlag & INTRINSICS))
        return validIds(sfmData, partFlag);

    return status;
}

bool save(const aliceVision::sfmData::SfMData& sfmData, const std::string& filename, ESfMData partFlag)
{
    const fs::path bPath = fs::path(filename);
    const std::string extension = bPath.extension().string();
    const std::string tmpPath = (bPath.parent_path() / bPath.stem()).string() + "." + utils::generateUniqueFilename() + extension;
    bool status = false;

    if (extension == ".sfm" || extension == ".json")  // JSON File
    {
        status = saveJSON(sfmData, tmpPath, partFlag);
    }
    else if (extension == ".ply")  // Polygon File
    {
        status = savePLY(sfmData, tmpPath, partFlag);
    }
    else if (extension == ".baf")  // Bundle Adjustment File
    {
        status = saveBAF(sfmData, tmpPath, partFlag);
    }
    else if (extension == ".abc")  // Alembic
    {
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
        AlembicExporter(tmpPath).addSfM(sfmData, partFlag);
        status = true;
#else
        ALICEVISION_THROW_ERROR("Cannot save the ABC file: \"" << filename << "\", AliceVision is built without Alembic support.");
#endif
    }
    else
    {
        ALICEVISION_LOG_ERROR("Cannot save the SfM data file: '" << filename << "'." << std::endl << "The file extension is not recognized.");
        return false;
    }

    // rename temporary filename
    if (status)
        fs::rename(tmpPath, filename);

    return status;
}

}  // namespace sfmDataIO
}  // namespace aliceVision
