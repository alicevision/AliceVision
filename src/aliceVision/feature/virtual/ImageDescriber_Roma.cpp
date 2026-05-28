// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImageDescriber_Roma.hpp"
#include <aliceVision/system/Logger.hpp>

namespace aliceVision {
namespace feature {


void ImageDescriber_Roma::setConfigurationPreset(ConfigurationPreset preset) { }

void ImageDescriber_Roma::allocate(std::unique_ptr<Regions>& regions) const { regions.reset(new ROMA_Regions); }

bool ImageDescriber_Roma::describe(const image::Image<unsigned char>& image,
                                   std::unique_ptr<Regions>& regions,
                                   const image::Image<unsigned char>* mask)
{
    // Roma feature extraction is performed externally (e.g. via a Python pipeline).
    // This virtual describer only provides the AliceVision interface to load/save
    // pre-computed Roma regions. Direct extraction is therefore not supported here.
    throw std::logic_error("ImageDescriber_Roma::describe() is not implemented. "
                           "Roma features must be extracted externally and loaded from disk.");
    return false;
}

}  // namespace feature
}  // namespace aliceVision
