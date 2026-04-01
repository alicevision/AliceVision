// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.
%include <std_shared_ptr.i>
%shared_ptr(aliceVision::sfmData::ImageGroup);
%shared_ptr(aliceVision::sfmData::ImageSet);
%shared_ptr(aliceVision::sfmData::ImageSequence);

%include <aliceVision/global.i>

%include <aliceVision/sfmData/ImageGroup.hpp>
%include <aliceVision/sfmData/ImageSet.hpp>
%include <aliceVision/sfmData/ImageSequence.hpp>

%{
#include <aliceVision/sfmData/ImageGroup.hpp>
#include <aliceVision/sfmData/ImageSet.hpp>
#include <aliceVision/sfmData/ImageSequence.hpp>
%}
