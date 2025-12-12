// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Landmark.hpp"
#include "View.hpp"

namespace aliceVision {
namespace sfmData {

IndexT Landmark::getReferenceViewIndex() const
{
    if (!_referenceView)
    {
        return UndefinedIndexT;
    }

    return _referenceView->getViewId();
}

}  // namespace sfmData
}  // namespace aliceVision
