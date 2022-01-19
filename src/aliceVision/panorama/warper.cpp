#include "warper.hpp"
#include <OpenEXR/half.h>

namespace aliceVision
{

bool Warper::warp(const CoordinatesMap& map, const aliceVision::image::Image<image::RGBfColor>& source)
{

    /**
     * Copy additional info from map
     */
    _offset_x = map.getOffsetX();
    _offset_y = map.getOffsetY();
    _mask = map.getMask();

    const image::Sampler2d<image::SamplerLinear> sampler;
    const aliceVision::image::Image<Eigen::Vector2d>& coordinates = map.getCoordinates();

    /**
     * Create buffer
     * No longer need to keep a 2**x size
     */
    _color = aliceVision::image::Image<image::RGBfColor>(coordinates.Width(), coordinates.Height());

    /**
     * Simple warp
     */
    for(size_t i = 0; i < _color.Height(); i++)
    {
        for(size_t j = 0; j < _color.Width(); j++)
        {

            bool valid = _mask(i, j);
            if(!valid)
            {
                continue;
            }

            const Eigen::Vector2d& coord = coordinates(i, j);
            const image::RGBfColor pixel = sampler(source, coord(1), coord(0));

            _color(i, j) = pixel;
        }
    }

    return true;
}

bool GaussianWarper::warp(const CoordinatesMap& map, const GaussianPyramidNoMask& pyramid, bool clamp)
{

    /**
     * Copy additional info from map
     */
    _offset_x = map.getOffsetX();
    _offset_y = map.getOffsetY();
    _mask = map.getMask();

    const image::Sampler2d<image::SamplerLinear> sampler;
    const aliceVision::image::Image<Eigen::Vector2d>& coordinates = map.getCoordinates();

    /**
     * Create a pyramid for input
     */
    const std::vector<image::Image<image::RGBfColor>>& mlsource = pyramid.getPyramidColor();
    size_t max_level = pyramid.getScalesCount() - 1;

    /**
     * Create buffer
     */
    _color = aliceVision::image::Image<image::RGBfColor>(coordinates.Width(), coordinates.Height(), true,
                                                         image::RGBfColor(1.0, 0.0, 0.0));

    /**
     * Multi level warp
     */
    for(size_t i = 0; i < _color.Height(); i++)
    {
        for(size_t j = 0; j < _color.Width(); j++)
        {

            bool valid = _mask(i, j);
            if(!valid)
            {
                continue;
            }

            int next_j = j + 1;
            int next_i = i + 1;

            if (j == _color.Width() - 1)
            {
                next_j = j - 1;
            }

            if (i == _color.Height() - 1)
            {
                next_i = i - 1;
            }

            if (!_mask(next_i, j) || !_mask(i, next_j))
            {
                const Eigen::Vector2d& coord = coordinates(i, j);
                const image::RGBfColor pixel = sampler(mlsource[0], coord(1), coord(0));
                _color(i, j) = pixel;

                continue;
            }

            const Eigen::Vector2d& coord_mm = coordinates(i, j);
            const Eigen::Vector2d& coord_mp = coordinates(i, next_j);
            const Eigen::Vector2d& coord_pm = coordinates(next_i, j);

            double dxx = coord_pm(0) - coord_mm(0);
            double dxy = coord_mp(0) - coord_mm(0);
            double dyx = coord_pm(1) - coord_mm(1);
            double dyy = coord_mp(1) - coord_mm(1);
            double det = std::abs(dxx * dyy - dxy * dyx);
            double scale = sqrt(det);

            double flevel = std::max(0.0, log2(scale));
            size_t blevel = std::min(max_level, size_t(floor(flevel)));

            double dscale, x, y;
            dscale = 1.0 / pow(2.0, blevel);
            x = coord_mm(0) * dscale;
            y = coord_mm(1) * dscale;
            /*Fallback to first level if outside*/
            if(x >= mlsource[blevel].Width() - 1 || y >= mlsource[blevel].Height() - 1)
            {
                _color(i, j) = sampler(mlsource[0], coord_mm(1), coord_mm(0));
                continue;
            }

            _color(i, j) = sampler(mlsource[blevel], y, x);
            if (clamp)
            {
                if (_color(i, j).r() > HALF_MAX) _color(i, j).r() = HALF_MAX;
                if (_color(i, j).g() > HALF_MAX) _color(i, j).g() = HALF_MAX;
                if (_color(i, j).b() > HALF_MAX) _color(i, j).b() = HALF_MAX;
            }
        }
    }

    return true;
}

} // namespace aliceVision