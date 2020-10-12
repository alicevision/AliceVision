#include "imageOps.hpp"
#include "gaussian.hpp"

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

bool downscaleByPowerOfTwo(image::Image<image::RGBfColor> & output, image::Image<unsigned char> & outputMask, const image::Image<image::RGBfColor> & input, const image::Image<unsigned char> & inputMask, const int timesDividedBy2) 
{
    image::Image<image::RGBfColor> currentColor = input;
    image::Image<unsigned char> currentMask = inputMask;

    for(int l = 1; l <= timesDividedBy2; l++)
    {

        aliceVision::image::Image<image::RGBfColor> buf(currentColor.Width(), currentColor.Height());
        aliceVision::image::Image<image::RGBfColor> nextColor(currentColor.Width() / 2, currentColor.Height() / 2);
        aliceVision::image::Image<unsigned char> nextMask(currentColor.Width() / 2, currentColor.Height() / 2);

        //Convolve + divide
        convolveGaussian5x5<image::RGBfColor>(buf, currentColor);
        downscale(nextColor, buf);

        //Just nearest neighboor divide for mask
        for(int i = 0; i < nextMask.Height(); i++)
        {
            int di = i * 2;

            for(int j = 0; j < nextMask.Width(); j++)
            {
                int dj = j * 2;

                if(currentMask(di, dj) && currentMask(di, dj + 1) 
                   && currentMask(di + 1, dj) && currentMask(di + 1, dj + 1))
                {
                    nextMask(i, j) = 255;
                }
                else
                {
                    nextMask(i, j) = 0;
                }
            }
        }

        currentColor = nextColor;
        currentMask = nextMask;
    }

    output = currentColor;
    outputMask = currentMask;

    return true;
}

} // namespace aliceVision