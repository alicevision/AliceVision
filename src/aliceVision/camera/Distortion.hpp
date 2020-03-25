#pragma once

#include <vector>
#include <aliceVision/numeric/numeric.hpp>

namespace aliceVision{
namespace camera{

class Distortion
{
public:

    Distortion() = default;

    virtual Distortion* clone() const = 0;

    std::vector<double>& getParameters()
    {
        return _distortionParams;
    }

    size_t getDistortionParametersCount()
    {
        return _distortionParams.size();
    }

    /// Add distortion to the point p (assume p is in the camera frame [normalized coordinates])
    virtual Vec2 addDistortion(const Vec2& p) const
    {
        return p;
    }

    /// Remove distortion (return p' such that disto(p') = p)
    virtual Vec2 removeDistortion(const Vec2& p) const
    {
        return p;
    }

    virtual double getUndistortedRadius(double r) const
    {
        return r;
    }

    virtual Eigen::Matrix2d getDerivativeAddDistoWrtPt(const Vec2& p) const
    {
        return Eigen::Matrix2d::Identity();
    }

    virtual Eigen::MatrixXd getDerivativeAddDistoWrtDisto(const Vec2& p) const
    {
        return Eigen::MatrixXd(0, 0);
    }

    virtual Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(const Vec2& p) const
    {
        return Eigen::Matrix2d::Identity();
    }

    virtual Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(const Vec2& p) const
    {
        return Eigen::MatrixXd(0, 0);
    }

    virtual ~Distortion() = default;

protected:
    std::vector<double> _distortionParams{};
};

} // namespace camera
} // namespace aliceVision
