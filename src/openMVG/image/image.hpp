// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef OPENMVG_IMAGE_HPP
#define OPENMVG_IMAGE_HPP

// Get rid of the specific MSVC compiler warnings.
#if defined(_MSC_VER) && !defined(_CRT_SECURE_NO_WARNINGS)
# define _CRT_SECURE_NO_WARNINGS
#endif

#include <iostream>
#include <vector>

#include "openMVG/numeric/numeric.h"

#include "openMVG/image/image_container.hpp"
#include "openMVG/image/pixel_types.hpp"
#include "openMVG/image/image_converter.hpp"
#include "openMVG/image/image_drawing.hpp"
#include "openMVG/image/image_concat.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/image/sample.hpp"

#include "openMVG/image/image_convolution_base.hpp"
#include "openMVG/image/image_convolution.hpp"
#include "openMVG/image/image_filtering.hpp"
#include "openMVG/image/image_resampling.hpp"
#include "openMVG/image/image_diffusion.hpp"

#endif /* OPENMVG_IMAGE_HPP */
