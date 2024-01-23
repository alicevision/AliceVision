// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

// %include <aliceVision/global.i>
%include <std_shared_ptr.i>
%shared_ptr(aliceVision::sfmData::View);

%include <aliceVision/sfmData/ImageInfo.i>
%include <aliceVision/sfmData/View.hpp>

%{
#include <aliceVision/sfmData/View.hpp>
%}
