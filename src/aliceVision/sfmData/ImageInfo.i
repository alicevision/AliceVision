// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%include <aliceVision/global.i>

// Handles all the std::shared_ptr<ImageInfo> objects 
// Side effect: will instantiate all the objects in the hierarchy as "std::shared_ptr"
%shared_ptr(aliceVision::sfmData::ImageInfo);

%include <aliceVision/sfmData/exif.hpp>
%include <aliceVision/sfmData/ExposureSetting.i>
%include <aliceVision/sfmData/ImageInfo.hpp>

%{
#include <aliceVision/sfmData/exif.hpp>
#include <aliceVision/sfmData/ImageInfo.hpp>
%}
