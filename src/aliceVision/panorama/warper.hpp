#pragma once

#include "coordinatesMap.hpp"
#include "gaussian.hpp"


namespace aliceVision
{

class Warper
{
public:
    virtual bool warp(const CoordinatesMap& map, const aliceVision::image::Image<image::RGBfColor>& source);

    const aliceVision::image::Image<image::RGBfColor>& getColor() const { return _color; }

    const aliceVision::image::Image<unsigned char>& getMask() const { return _mask; }

    size_t getOffsetX() const { return _offset_x; }

    size_t getOffsetY() const { return _offset_y; }

protected:
    size_t _offset_x = 0;
    size_t _offset_y = 0;

    aliceVision::image::Image<image::RGBfColor> _color;
    aliceVision::image::Image<unsigned char> _mask;
};

class GaussianWarper : public Warper
{
public:
    virtual bool warp(const CoordinatesMap& map, const GaussianPyramidNoMask& pyramid, bool clamp);
};

} // namespace aliceVision