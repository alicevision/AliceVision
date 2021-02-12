#pragma once

#include <aliceVision/image/all.hpp>

#include "coordinatesMap.hpp"

namespace aliceVision
{

bool distanceToCenter(aliceVision::image::Image<float>& _weights, const CoordinatesMap& map, int width, int height);
bool computeDistanceMap(image::Image<int>& distance, const image::Image<unsigned char>& mask);

} // namespace aliceVision