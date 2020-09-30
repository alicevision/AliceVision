#pragma once

#include <aliceVision/image/all.hpp>

#include "coordinatesMap.hpp"

namespace aliceVision {

bool distanceToCenter(aliceVision::image::Image<float> & _weights, const CoordinatesMap & map);

}