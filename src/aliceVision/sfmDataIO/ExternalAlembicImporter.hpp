// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <Alembic/AbcGeom/All.h>
#include <Alembic/AbcCoreFactory/All.h>
#include <Alembic/AbcCoreOgawa/All.h>


namespace aliceVision {
namespace sfmDataIO {

class ExternalAlembicImporter
{
public:
    explicit ExternalAlembicImporter(const std::string& filename);

    /**
     * @brief populate a SfMData from the alembic file
     * @param[out] sfmData The output SfMData
     * @param[in] files the input list of images files
     */
    void populateSfM(sfmData::SfMData& sfmdata, const std::vector<std::string> & files);

    void visitObject(Alembic::Abc::IObject iObj, const Alembic::Abc::M44d & mat, sfmData::SfMData& sfmdata, const std::vector<std::string> & files);

private: 
    Alembic::Abc::IObject _rootEntity;
    std::string _filename;
};

}  // namespace sfmDataIO
}  // namespace aliceVision
