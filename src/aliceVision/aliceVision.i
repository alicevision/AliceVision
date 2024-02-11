// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%module aliceVision

%include <aliceVision/global.i>

%import <aliceVision/camera/Camera.i>
%import <aliceVision/hdr/Hdr.i>
%import <aliceVision/sensorDB/SensorDB.i>
%import <aliceVision/sfmDataIO/SfMDataIO.i>
%import <aliceVision/sfmData/SfMData.i>

%include <aliceVision/version.hpp>

%{
#include <aliceVision/version.hpp>
using namespace aliceVision;
%}
