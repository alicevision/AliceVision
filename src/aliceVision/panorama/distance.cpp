#include "distance.hpp"

namespace aliceVision
{

bool distanceToCenter(aliceVision::image::Image<float>& _weights, const CoordinatesMap& map, int width, int height)
{

    const aliceVision::image::Image<Eigen::Vector2d>& coordinates = map.getCoordinates();
    const aliceVision::image::Image<unsigned char>& mask = map.getMask();

    float cx = width / 2.0f;
    float cy = height / 2.0f;

    _weights = aliceVision::image::Image<float>(coordinates.Width(), coordinates.Height());

    for(int i = 0; i < _weights.Height(); i++)
    {
        for(int j = 0; j < _weights.Width(); j++)
        {

            _weights(i, j) = 0.0f;

            bool valid = mask(i, j);
            if(!valid)
            {
                continue;
            }

            const Vec2& coords = coordinates(i, j);

            float x = coords(0);
            float y = coords(1);

            float wx = 1.0f - std::abs((x - cx) / cx);
            float wy = 1.0f - std::abs((y - cy) / cy);

            _weights(i, j) = wx * wy;
        }
    }

    return true;
}

static inline int f(int x_i, int gi) noexcept
{
    return (x_i * x_i) + gi * gi;
}

static inline int sep(int i, int u, int gi, int gu, int) noexcept
{
    return (u * u - i * i + gu * gu - gi * gi) / (2 * (u - i));
}

/// Code adapted from VFLib: https://github.com/vinniefalco/VFLib (Licence MIT)
bool computeDistanceMap(image::Image<int>& distance, const image::Image<unsigned char>& mask)
{

    int width = mask.Width();
    int height = mask.Height();

    int maxval = width + height;
    image::Image<int> buf(width, height);

    /* Per column distance 1D calculation */
    for(int j = 0; j < width; j++)
    {
        buf(0, j) = mask(0, j) ? 0 : maxval;

        /*Top to bottom accumulation */
        for(int i = 1; i < height; i++)
        {

            buf(i, j) = mask(i, j) ? 0 : 1 + buf(i - 1, j);
        }

        /*Bottom to top correction */
        for(int i = height - 2; i >= 0; i--)
        {

            if(buf(i + 1, j) < buf(i, j))
            {
                buf(i, j) = 1 + buf(i + 1, j);
            }
        }
    }

    std::vector<int> s(std::max(width, height));
    std::vector<int> t(std::max(width, height));

    /*Per row scan*/
    for(int i = 0; i < height; i++)
    {
        int q = 0;
        s[0] = 0;
        t[0] = 0;

        // scan 3
        for(int j = 1; j < width; j++)
        {
            while(q >= 0 && f(t[q] - s[q], buf(i, s[q])) > f(t[q] - j, buf(i, j)))
                q--;

            if(q < 0)
            {
                q = 0;
                s[0] = j;
            }
            else
            {
                int const w = 1 + sep(s[q], j, buf(i, s[q]), buf(i, j), maxval);

                if(w < width)
                {
                    ++q;
                    s[q] = j;
                    t[q] = w;
                }
            }
        }

        // scan 4
        for(int j = width - 1; j >= 0; --j)
        {
            int const d = f(j - s[q], buf(i, s[q]));

            distance(i, j) = d;
            if(j == t[q])
                --q;
        }
    }

    return true;
}

} // namespace aliceVision