// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%module (package="pyalicevision") sfm

%include <aliceVision/sfm/bundle/BundleAdjustment.hpp>
%include <aliceVision/sfm/sfmFilters.hpp>

%{
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfm/bundle/BundleAdjustment.hpp>
#include <aliceVision/sfm/sfmFilters.hpp>

using namespace aliceVision;
using namespace aliceVision::sfmData;
using namespace aliceVision::sfm;
%}
