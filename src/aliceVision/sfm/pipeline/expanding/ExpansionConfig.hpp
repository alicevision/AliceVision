// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

namespace aliceVision {
namespace sfm {

enum class EExpansionMode
{
    EXPANSION_STRICT,    // High thresholds — only stable, well-conditioned cameras
    EXPANSION_PERMISSIVE // Lower thresholds — accepts weaker cameras
};


}
}