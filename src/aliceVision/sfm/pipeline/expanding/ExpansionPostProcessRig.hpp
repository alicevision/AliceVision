// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfm/pipeline/expanding/ExpansionPostProcess.hpp>

namespace aliceVision {
namespace sfm {

class ExpansionPostProcessRig : public ExpansionPostProcess
{
public:
    using uptr = std::unique_ptr<ExpansionPostProcessRig>;

public:

    /**
     * @brief Perform post process for an iteration
     * @param sfmData the scene to process
     * @param tracksHandler the tracks for this scene
     * @return true if the process succeeded
    */
    bool process(sfmData::SfMData & sfmData, track::TracksHandler & tracksHandler) override;

    /**
     * @brief update the required number of cameras for rig resection
     * param count the desired minimal value
    */
    void setMinimalNumberCameras(std::size_t count)
    {
        _minNumberCameras = count;
    }

private:
    std::size_t _minNumberCameras = 20;
};

}
}