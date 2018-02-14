// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once
#include <aliceVision/structures/StaticVector.hpp>

class MultiViewParams;

StaticVector<StaticVector<int>*>* getPtsCamsFromInfoFile(const std::string& fileNameInfo);
StaticVector<int>* getUsedCamsFromInfoFile(const std::string& fileNameInfo, MultiViewParams* mp);
