// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>
#include <pxr/usd/usd/stage.h>

namespace aliceVision {
namespace sfmDataIO {

class UsdExporter
{
public:
    UsdExporter(const std::string & filename);

    void createNewCamera(const std::string & name, long startTimeCode, long endTimeCode, double frameRate);
    
    void addFrame(const sfmData::View::sptr & view, const camera::IntrinsicBase::sptr);

    void terminate();

private:
    UsdStageRefPtr _stage;
}

}  // namespace sfmDataIO
}  // namespace aliceVision
