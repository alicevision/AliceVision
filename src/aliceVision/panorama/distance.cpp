#pragma once

#include "distance.hpp"

namespace aliceVision {

bool distanceToCenter(aliceVision::image::Image<float> & _weights, const CoordinatesMap & map, int width, int height) {

    const aliceVision::image::Image<Eigen::Vector2d> & coordinates = map.getCoordinates();
    const aliceVision::image::Image<unsigned char> & mask = map.getMask();

    float cx = width / 2.0f;
    float cy = height / 2.0f;

    _weights = aliceVision::image::Image<float>(coordinates.Width(), coordinates.Height());

    for (int i = 0; i < _weights.Height(); i++) {
        for (int j = 0; j < _weights.Width(); j++) {
        
            _weights(i, j) = 0.0f;

            bool valid = mask(i, j);
            if (!valid) {
                continue;
            }

            const Vec2 & coords = coordinates(i, j);

            float x = coords(0);
            float y = coords(1);

            float wx = 1.0f - std::abs((x - cx) / cx);
            float wy = 1.0f - std::abs((y - cy) / cy);
            
            _weights(i, j) = wx * wy;
        }
    }

    return true;
}

}