#pragma once

#include <aliceVision/structures/mv_multiview_params.hpp>
#include <aliceVision/structures/mv_color.hpp>

class mv_images_cache
{
public:
    const multiviewParams* mp;

    int N_PRELOADED_IMAGES;
    rgb** imgs;

    staticVector<int>* camIdMapId;
    staticVector<int>* mapIdCamId;
    staticVector<long>* mapIdClock;
    std::vector<std::string> imagesNames;

    int bandType;
    bool transposed;

    mv_images_cache(const multiviewParams* _mp, int _bandType, bool _transposed = false);
    mv_images_cache(const multiviewParams* _mp, int _bandType, std::vector<std::string>& _imagesNames,
                    bool _transposed = false);
    void initIC(int _bandType, std::vector<std::string>& _imagesNames, bool _transposed);
    ~mv_images_cache();

    int getPixelId(int x, int y, int imgid);
    void refreshData(int camId);
    Color getPixelValueInterpolated(const point2d* pix, int camId);
    rgb getPixelValue(const pixel* pix, int camId);
};
