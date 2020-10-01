#include "imageOps.hpp"

namespace aliceVision
{

void removeNegativeValues(aliceVision::image::Image<image::RGBfColor>& img)
{
    for(int i = 0; i < img.Height(); i++)
    {
        for(int j = 0; j < img.Width(); j++)
        {
            image::RGBfColor& pix = img(i, j);
            image::RGBfColor rpix;
            rpix.r() = std::exp(pix.r());
            rpix.g() = std::exp(pix.g());
            rpix.b() = std::exp(pix.b());

            if(rpix.r() < 0.0)
            {
                pix.r() = 0.0;
            }

            if(rpix.g() < 0.0)
            {
                pix.g() = 0.0;
            }

            if(rpix.b() < 0.0)
            {
                pix.b() = 0.0;
            }
        }
    }
}

} // namespace aliceVision