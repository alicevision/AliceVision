#include "feathering.hpp"

namespace aliceVision
{

bool feathering(aliceVision::image::Image<image::RGBfColor>& output,
                const aliceVision::image::Image<image::RGBfColor>& color,
                const aliceVision::image::Image<unsigned char>& inputMask)
{

    std::vector<image::Image<image::RGBfColor>> feathering;
    std::vector<image::Image<unsigned char>> feathering_mask;
    feathering.push_back(color);
    feathering_mask.push_back(inputMask);

    int lvl = 0;
    int width = color.Width();
    int height = color.Height();

    while(1)
    {
        const image::Image<image::RGBfColor>& src = feathering[lvl];
        const image::Image<unsigned char>& src_mask = feathering_mask[lvl];

        image::Image<image::RGBfColor> half(width / 2, height / 2);
        image::Image<unsigned char> half_mask(width / 2, height / 2);

        for(int i = 0; i < half.Height(); i++)
        {

            int di = i * 2;
            for(int j = 0; j < half.Width(); j++)
            {
                int dj = j * 2;

                int count = 0;
                half(i, j) = image::RGBfColor(0.0, 0.0, 0.0);

                if(src_mask(di, dj))
                {
                    half(i, j) += src(di, dj);
                    count++;
                }

                if(src_mask(di, dj + 1))
                {
                    half(i, j) += src(di, dj + 1);
                    count++;
                }

                if(src_mask(di + 1, dj))
                {
                    half(i, j) += src(di + 1, dj);
                    count++;
                }

                if(src_mask(di + 1, dj + 1))
                {
                    half(i, j) += src(di + 1, dj + 1);
                    count++;
                }

                if(count > 0)
                {
                    half(i, j) /= float(count);
                    half_mask(i, j) = 1;
                }
                else
                {
                    half_mask(i, j) = 0;
                }
            }
        }

        feathering.push_back(half);
        feathering_mask.push_back(half_mask);

        width = half.Width();
        height = half.Height();

        if(width < 2 || height < 2)
            break;

        lvl++;
    }

    for(int lvl = feathering.size() - 2; lvl >= 0; lvl--)
    {

        image::Image<image::RGBfColor>& src = feathering[lvl];
        image::Image<unsigned char>& src_mask = feathering_mask[lvl];
        image::Image<image::RGBfColor>& ref = feathering[lvl + 1];
        image::Image<unsigned char>& ref_mask = feathering_mask[lvl + 1];

        for(int i = 0; i < src_mask.Height(); i++)
        {
            for(int j = 0; j < src_mask.Width(); j++)
            {
                if(!src_mask(i, j))
                {
                    int mi = i / 2;
                    int mj = j / 2;

                    if(mi >= ref_mask.Height())
                    {
                        mi = ref_mask.Height() - 1;
                    }

                    if(mj >= ref_mask.Width())
                    {
                        mj = ref_mask.Width() - 1;
                    }

                    src_mask(i, j) = ref_mask(mi, mj);
                    src(i, j) = ref(mi, mj);
                }
            }
        }
    }

    output = feathering[0];

    return true;
}

} // namespace aliceVision