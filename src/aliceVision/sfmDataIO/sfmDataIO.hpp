// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/version.hpp>

#define ALICEVISION_SFMDATAIO_VERSION_MAJOR 1
#define ALICEVISION_SFMDATAIO_VERSION_MINOR 2
#define ALICEVISION_SFMDATAIO_VERSION_REVISION 6

// AliceVision version as a string; for example "0.9.0".
#define ALICEVISION_SFMDATAIO_VERSION_STRING                                                                                                         \
    ALICEVISION_TO_STRING(ALICEVISION_SFMDATAIO_VERSION_MAJOR)                                                                                       \
    "." ALICEVISION_TO_STRING(ALICEVISION_SFMDATAIO_VERSION_MINOR) "." ALICEVISION_TO_STRING(ALICEVISION_SFMDATAIO_VERSION_REVISION)

namespace aliceVision {
namespace sfmDataIO {

enum ESfMData
{
    VIEWS = 1,
    EXTRINSICS = 2,
    INTRINSICS = 4,
    STRUCTURE = 8,
    OBSERVATIONS = 16,
    OBSERVATIONS_WITH_FEATURES = 32,
    LANDMARKS_UNCERTAINTY = 64,
    POSES_UNCERTAINTY = 128,
    CONSTRAINTS2D = 256,

    UNCERTAINTY = LANDMARKS_UNCERTAINTY | POSES_UNCERTAINTY,
    ALL_DENSE = VIEWS | EXTRINSICS | INTRINSICS | STRUCTURE | OBSERVATIONS | CONSTRAINTS2D,
    ALL = VIEWS | EXTRINSICS | INTRINSICS | STRUCTURE | OBSERVATIONS | OBSERVATIONS_WITH_FEATURES | UNCERTAINTY | CONSTRAINTS2D
};

/// check that each pose have a valid intrinsic and pose id in the existing View ids
bool ValidIds(const sfmData::SfMData& sfmData, ESfMData partFlag);

/// load SfMData SfM scene from a file
bool Load(sfmData::SfMData& sfmData, const std::string& filename, ESfMData partFlag);

/// save SfMData SfM scene to a file
bool Save(const sfmData::SfMData& sfmData, const std::string& filename, ESfMData partFlag);

}  // namespace sfmDataIO
}  // namespace aliceVision
