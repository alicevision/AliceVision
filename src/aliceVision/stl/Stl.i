// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%module (module="pyalicevision") stl

%include <aliceVision/global.i>

%include <aliceVision/stl/Bitmask.i>
%include <aliceVision/stl/DynamicBitset.i>
%include <aliceVision/stl/FlatMap.i>
%include <aliceVision/stl/Hash.i>
%include <aliceVision/stl/IndexedSort.i>
%include <aliceVision/stl/MapUtils.i>
%include <aliceVision/stl/Regex.i>

%include <aliceVision/stl/stl.hpp>

%{
#include <aliceVision/stl/stl.hpp>
using namespace aliceVision;
using namespace stl;
%}