#include "imageOps.hpp"
#include "gaussian.hpp"
#include "feathering.hpp"

namespace aliceVision
{

void removeNegativeValues(image::Image<image::RGBfColor> & img)
{
    for (int i = 0; i < img.Height(); i++) 
    {
        for (int j = 0; j < img.Width(); j++)
        {
            image::RGBfColor rpix;
            image::RGBfColor ret = img(i, j);

            rpix.r() = std::exp(ret.r());
            rpix.g() = std::exp(ret.g());
            rpix.b() = std::exp(ret.b());

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

            img(i, j) = ret;
        }
    }
}

} // namespace aliceVision