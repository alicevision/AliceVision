// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>

namespace aliceVision {
namespace mvsUtils {

class PreMatchCams
{
public:
    float minang;
    float maxang;

    explicit PreMatchCams(const MultiViewParams& mp);

    StaticVector<int> findCamsWhichIntersectsHexahedron(const Point3d hexah[8], const std::string& minMaxDepthsFileName);
    StaticVector<int> findCamsWhichIntersectsHexahedron(const Point3d hexah[8]);
    StaticVector<int> findNearestCamsFromLandmarks(int rc, int nbNearestCams);

private:
    const MultiViewParams& _mp;
};

} // namespace mvsUtils
} // namespace aliceVision
