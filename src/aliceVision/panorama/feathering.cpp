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

    while (!(width < 2 || height < 2))
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

        lvl++;
    }

    //Now we want to make sure we have no masked pixel with undefined color
    //So we compute the mean of all valid pixels, and set the invalid pixels to this value
    image::Image<image::RGBfColor> & lastImage = feathering[feathering.size() - 1];
    image::Image<unsigned char> & lastMask = feathering_mask[feathering_mask.size() - 1];
    image::RGBfColor sum(0.0f);
    int count = 0;
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            if (lastMask(y, x))
            {
                sum.r() += lastImage(y, x).r();
                sum.g() += lastImage(y, x).g();
                sum.b() += lastImage(y, x).b();
                count++;
            }
        }
    }

    if (count > 0) 
    {
        image::RGBfColor mean;
        mean.r() = sum.r() / float(count);
        mean.g() = sum.g() / float(count);
        mean.b() = sum.b() / float(count);

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                if (!lastMask(y, x))
                {
                    lastImage(y, x) = mean;
                    lastMask(y, x) = 255;
                }
            }
        }
    }


    //Now, level by level, we fill masked pixel with the estimated value from
    //The lower level.
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


bool feathering(CachedImage<image::RGBfColor> & input_output, CachedImage<unsigned char> & inputMask) 
{
    if (input_output.getTileSize() < 2) 
    {
        return false;
    }

    if (input_output.getTileSize() != inputMask.getTileSize()) 
    {
        return false;
    }

    if (input_output.getWidth() != inputMask.getWidth()) 
    {
        return false;
    }

    if (input_output.getHeight() != inputMask.getHeight()) 
    {
        return false;
    }

    std::vector<std::vector<image::CachedTile::smart_pointer>> & tilesColor = input_output.getTiles();
    std::vector<std::vector<image::CachedTile::smart_pointer>> & tilesMask = inputMask.getTiles();

    if (tilesColor.empty()) 
    {
        return false;
    }

    int gridHeight = tilesColor.size();
    int gridWidth = tilesColor[0].size();
    int currentSize = input_output.getTileSize();

    //Make sure the grid has a pow2 size, and is square
    gridWidth = pow(2.0, std::ceil(std::log2(float(gridWidth))));
    gridHeight = pow(2.0, std::ceil(std::log2(float(gridHeight))));
    int gridSize = std::max(gridWidth, gridHeight);

    image::Image<image::RGBfColor> colorTile;
    image::Image<unsigned char> maskTile;

    image::Image<image::RGBfColor> featheredGrid(gridSize, gridSize);
    image::Image<image::RGBfColor> colorGrid(gridSize, gridSize);
    image::Image<unsigned char> maskGrid(gridSize, gridSize, true, 0);

    /*Build the grid color image */
    for (int i = 0; i < tilesColor.size(); i++)
    {   
        std::vector<image::CachedTile::smart_pointer> rowColor = tilesColor[i];
        std::vector<image::CachedTile::smart_pointer> rowMask = tilesMask[i];

        for (int j = 0; j < rowColor.size(); j++)
        {
            if (!CachedImage<image::RGBfColor>::getTileAsImage(colorTile, rowColor[j])) 
            {
                return false;
            }

            if (!CachedImage<unsigned char>::getTileAsImage(maskTile, rowMask[j])) 
            {
                return false;
            }

            while (1) 
            {
                image::Image<image::RGBfColor> smallerTile(colorTile.Width() / 2, colorTile.Height() / 2);
                image::Image<unsigned char> smallerMask(maskTile.Width() / 2, maskTile.Height() / 2);

                for(int y = 0; y < smallerTile.Height(); y++)
                {
                    int dy = y * 2;
                    for(int x = 0; x < smallerTile.Width(); x++)
                    {
                        int dx = x * 2;

                        int count = 0;

                        smallerTile(y, x) = image::RGBfColor(0.0, 0.0, 0.0);

                        if(maskTile(dy, dx))
                        {
                            smallerTile(y, x) += colorTile(dy, dx);
                            count++;
                        }

                        if(maskTile(dy, dx + 1))
                        {
                            smallerTile(y, x) += colorTile(dy, dx + 1);
                            count++;
                        }

                        if(maskTile(dy + 1, dx))
                        {
                            smallerTile(y, x) += colorTile(dy + 1, dx);
                            count++;
                        }

                        if(maskTile(dy + 1, dx + 1))
                        {
                            smallerTile(y, x) += colorTile(dy + 1, dx + 1);
                            count++;
                        }

                        if(count > 0)
                        {
                            smallerTile(y, x) /= float(count);
                            smallerMask(y, x) = 1;
                        }
                        else
                        {
                            smallerMask(y, x) = 0;
                        }
                    }
                }

                colorTile = smallerTile;
                maskTile = smallerMask;
                if (colorTile.Width() < 2 || colorTile.Height() < 2)
                {
                    break;
                }
            }

            maskGrid(i, j) = maskTile(0, 0);
            colorGrid(i, j) = colorTile(0, 0);
        }
    }


    if (!feathering(featheredGrid, colorGrid, maskGrid)) 
    {
        return false;
    }
    
    for (int i = 0; i < tilesColor.size(); i++)
    {   
        std::vector<image::CachedTile::smart_pointer> rowColor = tilesColor[i];
        std::vector<image::CachedTile::smart_pointer> rowMask = tilesMask[i];

        for (int j = 0; j < rowColor.size(); j++)
        {
            if (!CachedImage<image::RGBfColor>::getTileAsImage(colorTile, rowColor[j])) 
            {
                return false;
            }

            if (!CachedImage<unsigned char>::getTileAsImage(maskTile, rowMask[j])) 
            {
                return false;
            }

            std::vector<image::Image<image::RGBfColor>> pyramid_colors;
            std::vector<image::Image<unsigned char>> pyramid_masks;

            pyramid_colors.push_back(colorTile);
            pyramid_masks.push_back(maskTile);

            while (1) 
            {
                image::Image<image::RGBfColor> & largerTile = pyramid_colors[pyramid_colors.size() - 1];
                image::Image<unsigned char> & largerMask = pyramid_masks[pyramid_masks.size() - 1];

                image::Image<image::RGBfColor> smallerTile(largerTile.Width() / 2, largerTile.Height() / 2);
                image::Image<unsigned char> smallerMask(largerMask.Width() / 2, largerMask.Height() / 2);

                for(int y = 0; y < smallerTile.Height(); y++)
                {
                    int dy = y * 2;
                    for(int x = 0; x < smallerTile.Width(); x++)
                    {
                        int dx = x * 2;

                        int count = 0;

                        smallerTile(y, x) = image::RGBfColor(0.0, 0.0, 0.0);

                        if(largerMask(dy, dx))
                        {
                            smallerTile(y, x) += largerTile(dy, dx);
                            count++;
                        }

                        if(largerMask(dy, dx + 1))
                        {
                            smallerTile(y, x) += largerTile(dy, dx + 1);
                            count++;
                        }

                        if(largerMask(dy + 1, dx))
                        {
                            smallerTile(y, x) += largerTile(dy + 1, dx);
                            count++;
                        }

                        if(largerMask(dy + 1, dx + 1))
                        {
                            smallerTile(y, x) += largerTile(dy + 1, dx + 1);
                            count++;
                        }

                        if(count > 0)
                        {
                            smallerTile(y, x) /= float(count);
                            smallerMask(y, x) = 1;
                        }
                        else
                        {
                            smallerMask(y, x) = 0;
                        }
                    }
                }


                pyramid_colors.push_back(smallerTile);
                pyramid_masks.push_back(smallerMask);
                
                if (smallerTile.Width() < 2 || smallerTile.Height() < 2)
                {
                    break;
                }
            }

            image::Image<image::RGBfColor> & img = pyramid_colors[pyramid_colors.size() - 1];
            image::Image<unsigned char> & mask = pyramid_masks[pyramid_masks.size() - 1];

            if (!mask(0, 0)) 
            {
                mask(0, 0) = 255;
                img(0, 0) = featheredGrid(i, j);
            }
            

            for(int lvl = pyramid_colors.size() - 2; lvl >= 0; lvl--)
            {

                image::Image<image::RGBfColor> & src = pyramid_colors[lvl];
                image::Image<unsigned char> & src_mask = pyramid_masks[lvl];
                image::Image<image::RGBfColor> & ref = pyramid_colors[lvl + 1];
                image::Image<unsigned char> & ref_mask = pyramid_masks[lvl + 1];

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

            if (!CachedImage<image::RGBfColor>::setTileWithImage(rowColor[j], pyramid_colors[0])) 
            {
                return false;
            }
        }
    }

    return true;
}

} // namespace aliceVision