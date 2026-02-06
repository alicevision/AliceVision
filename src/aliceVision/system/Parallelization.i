// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%include "typemaps.i"

%apply int& OUTPUT { int& rangeStart };
%apply int& OUTPUT { int& rangeEnd };

%{
#include <aliceVision/system/Parallelization.hpp>

using namespace aliceVision;
%}

%include <aliceVision/system/Parallelization.hpp>
