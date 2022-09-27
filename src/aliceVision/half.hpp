// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#if __has_include(<Imath/half.h>)
  // Try to use the modern header first: OpenEXR >= 3.0
  #include <Imath/half.h>
#elif __has_include(<OpenEXR/half.h>)
  // Try fallback for compatibility with OpenEXR < 3.0
  #include <OpenEXR/half.h>
#else
  // Generate an error using the modern header
  #include <Imath/half.h>
#endif


