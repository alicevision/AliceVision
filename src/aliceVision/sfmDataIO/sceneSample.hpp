// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>

namespace aliceVision {
namespace sfmDataIO {

enum ESceneType
{
    SCENE_CUBE,
    SCENE_SPHERE,
};

ESceneType ESceneType_stringToEnum(const std::string& SScene);
std::string ESceneType_enumToString(const ESceneType EScene);
std::ostream& operator<<(std::ostream& os, ESceneType p);
std::istream& operator>>(std::istream& in, ESceneType& p);

/**
 * @brief Create an SfmData with some arbitrary content.
 * This is used in unit tests to validate read/write sfmData files.
 */
void generateSampleScene(sfmData::SfMData& output, ESceneType scene=ESceneType::SCENE_CUBE);

void generateCubeScene(sfmData::SfMData& output);

void generateSphereScene(sfmData::SfMData& output, int pointsNb, int posesNb);

}  // namespace sfmDataIO
}  // namespace aliceVision
