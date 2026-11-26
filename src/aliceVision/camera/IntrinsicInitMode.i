// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

// Stream operators are not usable from Python, and their __lshift__ clashes with other headers' (e.g. ExposureSetting)
// in modules that include this file through IntrinsicBase.i (e.g. sfmData).
%ignore aliceVision::camera::operator<<(std::ostream&, const EInitMode);
%ignore aliceVision::camera::operator>>(std::istream&, EInitMode&);

%include <aliceVision/camera/IntrinsicInitMode.hpp>

%{
#include <aliceVision/camera/IntrinsicInitMode.hpp>
%}