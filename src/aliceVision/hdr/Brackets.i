// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.


%include <aliceVision/hdr/brackets.hpp>

%{
#include <aliceVision/hdr/brackets.hpp>

using namespace aliceVision;
using namespace aliceVision::hdr;
%}


%include "std_vector.i"

%template(vectorli) std::vector<aliceVision::hdr::LuminanceInfo>;
%template(vvectori) std::vector<std::vector<aliceVision::IndexT>>;

