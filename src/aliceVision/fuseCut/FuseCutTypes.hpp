// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/types.hpp>

#include <array>

namespace aliceVision {
namespace fuseCut {

struct CellInfo
{
    /// initialized to a large value if the tetrahedron is directly in front of one camera ELSE set to 0
    float cellSWeight = 0.0f;
    /// strong fullness: sum of weights for being the tetrahedron 2*sigma behind the point p
    float cellTWeight = 0.0f;
    // float gEdgePhotoWeight[4];
    /// score for emptiness along each egde/facet
    std::array<float, 4> gEdgeVisWeight{{0.0f, 0.0f, 0.0f, 0.0f}};
    /// emptiness score: sum of all weights for emptiness (before the point p)
    float emptinessScore = 0.0f;
    /// first full tetrahedron score: sum of weights for T1 (tetrahedron just after the point p)
    float on = 0.0f;
};

}  // namespace fuseCut
}  // namespace aliceVision
