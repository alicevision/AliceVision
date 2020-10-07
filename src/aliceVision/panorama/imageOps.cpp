#include "imageOps.hpp"

namespace aliceVision
{

void removeNegativeValues(CachedImage<image::RGBfColor> & img)
{
    img.perPixelOperation(
        [](const image::RGBfColor & c) -> image::RGBfColor 
        {
            image::RGBfColor rpix;
            image::RGBfColor ret = c;

            rpix.r() = std::exp(c.r());
            rpix.g() = std::exp(c.g());
            rpix.b() = std::exp(c.b());

            if (rpix.r() < 0.0)
            {
                ret.r() = 0.0;
            }

            if(rpix.g() < 0.0)
            {
                ret.g() = 0.0;
            }

            if(rpix.b() < 0.0)
            {
                ret.b() = 0.0;
            }

            return ret;
        }
    );
}

} // namespace aliceVision