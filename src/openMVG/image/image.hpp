// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef ALICEVISION_IMAGE_HPP
#define ALICEVISION_IMAGE_HPP

// Get rid of the specific MSVC compiler warnings.
#if defined(_MSC_VER) && !defined(_CRT_SECURE_NO_WARNINGS)
# define _CRT_SECURE_NO_WARNINGS
#endif

#include <iostream>
#include <vector>

#include "aliceVision/numeric/numeric.h"

#include "aliceVision/image/image_container.hpp"
#include "aliceVision/image/pixel_types.hpp"
#include "aliceVision/image/image_converter.hpp"
#include "aliceVision/image/image_drawing.hpp"
#include "aliceVision/image/image_concat.hpp"
#include "aliceVision/image/image_io.hpp"
#include "aliceVision/image/sample.hpp"

#include "aliceVision/image/image_convolution_base.hpp"
#include "aliceVision/image/image_convolution.hpp"
#include "aliceVision/image/image_filtering.hpp"
#include "aliceVision/image/image_resampling.hpp"
#include "aliceVision/image/image_diffusion.hpp"

#endif /* ALICEVISION_IMAGE_HPP */
