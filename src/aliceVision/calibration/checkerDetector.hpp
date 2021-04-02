#pragma once

#include <aliceVision/system/system.hpp>
#include <aliceVision/image/all.hpp>

namespace aliceVision{
namespace calibration{

class CheckerDetector
{
public:
    bool process(const image::Image<image::RGBColor> & source);

private:
    bool processLevel(const image::Image<float> & input);
    bool normalizeImage(image::Image<float> & output, const image::Image<float> & input);
    bool computeHessianResponse(image::Image<float> & output, const image::Image<float> & input);
    bool extractCorners(std::vector<Vec2> & raw_corners, const image::Image<float> & hessianResponse);
    bool refineCorners(std::vector<Vec2> & refined_corners, const std::vector<Vec2> & raw_corners, const image::Image<float> & input);
    bool fitCorners(std::vector<Vec2> & refined_corners, const std::vector<Vec2> & raw_corners, const image::Image<float> & input);
    void getMinMax(float &min, float &max, const image::Image<float> & input);
};

}//namespace calibration
}//namespace aliceVision
