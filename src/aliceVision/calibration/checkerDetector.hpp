#pragma once

#include <aliceVision/system/system.hpp>
#include <aliceVision/image/all.hpp>
#include <aliceVision/types.hpp>

namespace aliceVision{
namespace calibration{

class CheckerDetector
{
public:
    using CheckerBoard = Eigen::Matrix<IndexT, -1, -1>;

    struct NewPoint
    {
        IndexT row;
        IndexT col;
        double score;
    };

    struct CheckerBoardCorner
    {
        Vec2 center;
        Vec2 dir1;
        Vec2 dir2;
    };

    enum Direction
    {
        LEFT,
        RIGHT,
        UP,
        DOWN
    };

public:
    bool process(const image::Image<image::RGBColor> & source);

    std::vector<CheckerBoard> getBoards()
    {
        return _boards;
    }

    std::vector<CheckerBoardCorner> getCorners()
    {
        return _corners;
    }

    void drawCheckerBoard(image::Image<image::RGBColor> & img) const;
    
private:
    bool processLevel(std::vector<Vec2> & corners, const image::Image<float> & input, double scale);
    bool normalizeImage(image::Image<float> & output, const image::Image<float> & input);
    bool computeHessianResponse(image::Image<float> & output, const image::Image<float> & input);
    bool extractCorners(std::vector<Vec2> & raw_corners, const image::Image<float> & hessianResponse);
    bool refineCorners(std::vector<Vec2> & refined_corners, const std::vector<Vec2> & raw_corners, const image::Image<float> & input);
    bool pruneCorners(std::vector<Vec2> & pruned_corners, const std::vector<Vec2> & raw_corners, const image::Image<float> & input);
    bool fitCorners(std::vector<CheckerBoardCorner> & refined_corners, const std::vector<Vec2> & raw_corners, const image::Image<float> & input);
    void getMinMax(float &min, float &max, const image::Image<float> & input);
    bool buildCheckerboards(std::vector<CheckerBoard> & boards, const std::vector<CheckerBoardCorner> & refined_corners, const image::Image<float> & input);
    IndexT findClosestCorner(const Vec2 & center, const Vec2 & dir, const std::vector<CheckerBoardCorner> & refined_corners);
    bool getSeedCheckerboard(Eigen::Matrix<IndexT, -1, -1> & board, IndexT seed, const std::vector<CheckerBoardCorner> & refined_corners);
    double computeEnergy(Eigen::Matrix<IndexT, -1, -1> & board, const std::vector<CheckerBoardCorner> & refined_corners);
    bool getCandidates(std::vector<NewPoint> & candidates, Eigen::Matrix<IndexT, -1, -1> & board);
    bool growIteration(Eigen::Matrix<IndexT, -1, -1> & board, const std::vector<CheckerBoardCorner> & refined_corners);
    bool growIterationUp(Eigen::Matrix<IndexT, -1, -1> & board, const std::vector<CheckerBoardCorner> & refined_corners);
    
    bool mergeCheckerboards();

    
private:
    std::vector<CheckerBoard> _boards;
    std::vector<CheckerBoardCorner> _corners;
};

}//namespace calibration
}//namespace aliceVision
