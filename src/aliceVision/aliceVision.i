// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%module pyalicevision

%include <aliceVision/global.i>
%include <aliceVision/version.hpp>

%import <aliceVision/numeric/numeric.i>
%import <aliceVision/camera/Camera.i>
%import <aliceVision/geometry/Geometry.i>
%import <aliceVision/hdr/Hdr.i>
%import <aliceVision/sensorDB/SensorDB.i>
%import <aliceVision/sfmDataIO/SfMDataIO.i>
%import <aliceVision/sfmData/SfMData.i>
%import <aliceVision/stl/Stl.i>

%{
#include <aliceVision/version.hpp>

//For unknown reason, we need to declare cameras here too
#include <aliceVision/camera/IntrinsicBase.hpp>
#include <aliceVision/camera/Pinhole.hpp>
#include <aliceVision/camera/Equidistant.hpp>
#include <aliceVision/camera/Equirectangular.hpp>

using namespace aliceVision;

%}
