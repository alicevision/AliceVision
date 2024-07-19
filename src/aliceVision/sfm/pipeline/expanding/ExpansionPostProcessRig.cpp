// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/pipeline/expanding/ExpansionPostProcessRig.hpp>
#include <aliceVision/sfm/pipeline/RigSequence.hpp>

namespace aliceVision {
namespace sfm {

bool ExpansionPostProcessRig::process(sfmData::SfMData & sfmData, track::TracksHandler & tracksHandler)
{
    RigParams params;
    params.useRigConstraint = true;
    params.minNbCamerasForCalibration = _minNumberCameras;
    _updatedViews.clear();

    if (sfmData.getRigs().empty())
    {
        return false;
    }

    /*Calibrate all rigs*/
    int countInit = 0;
    for (const auto & [rigId, rigObject] : sfmData.getRigs())
    {
        if (rigObject.isInitialized())
        {
            continue;
        }

        RigSequence sequence(sfmData, rigId, params);
        sequence.init(tracksHandler.getTracksPerView());
        sequence.updateSfM(_updatedViews);

        countInit++;
    }

    if (countInit == 0)
    {
        return false;
    }


    return true;
}

}
}