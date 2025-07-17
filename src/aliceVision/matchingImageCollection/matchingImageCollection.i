// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%module (package="pyalicevision") matchingImageCollection

%include <aliceVision/global.i>
%include <std_string.i>
%include <std_set.i>

%{    
#include <aliceVision/matchingImageCollection/ImagePairListIO.hpp>

using namespace aliceVision;

%} 

%include <aliceVision/matchingImageCollection/ImagePairListIO.hpp>
