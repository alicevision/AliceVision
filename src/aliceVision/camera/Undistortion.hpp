#pragma once

#include <vector>
#include <aliceVision/numeric/numeric.hpp>

namespace aliceVision{
namespace camera{

class Undistortion
{
public:

    Undistortion(int width, int height)
    {
        _diagonal =  0.5  * sqrt(width * width + height * height);
        _center = {width / 2, height / 2};
    }

    virtual Undistortion* clone() const = 0;

    // not virtual as child classes do not hold any data
    bool operator==(const Undistortion& other) const
    {
        return _undistortionParams == other._undistortionParams;
    }

    void setOffset(const Vec2& offset)
    {
        _offset = offset;
    }

    inline Vec2 getOffset() const 
    { 
        return _offset; 
    }


    const std::vector<double>& getParameters()
    {
        return _undistortionParams;
    }

    void setParameters(const std::vector<double>& params)
    {
        for (int i = 0; i < _undistortionParams.size(); i++)
        {
            _undistortionParams[i] = params[i];
        }
    }

    size_t getUndistortionParametersCount()
    {
        return _undistortionParams.size();
    }

    
    Vec2 undistort(const Vec2& p) const
    {
        Vec2 centered = p - _center - _offset;
        Vec2 normalized = centered / _diagonal;
        Vec2 undistorted = undistortNormalized(normalized);
        Vec2 unnormalized = undistorted * _diagonal + _center;

        return p;
    }

    Eigen::Matrix<double, 2, Eigen::Dynamic> getDerivativeUndistortWrtParameters(const Vec2 &p)
    {
        Vec2 centered = p - _center - _offset;
        Vec2 normalized = centered / _diagonal;
        Vec2 undistorted = undistortNormalized(normalized);
        Vec2 unnormalized = undistorted * _diagonal + _center;

        Eigen::Matrix<double, 2, 2> d_unnormalized_d_undistorted;
        d_unnormalized_d_undistorted(0, 0) = _diagonal;
        d_unnormalized_d_undistorted(0, 1) = 0;
        d_unnormalized_d_undistorted(1, 0) = 0;
        d_unnormalized_d_undistorted(1, 1) = _diagonal;

        return d_unnormalized_d_undistorted * getDerivativeUndistortNormalizedwrtParameters(normalized);
    }

    Eigen::Matrix<double, 2, 2> getDerivativeUndistortWrtOffset(const Vec2 &p)
    {
        Vec2 centered = p - _center - _offset;
        Vec2 normalized = centered / _diagonal;
        Vec2 undistorted = undistortNormalized(normalized);
        Vec2 unnormalized = undistorted * _diagonal + _center;

        Eigen::Matrix<double, 2, 2> d_unnormalized_d_undistorted;
        d_unnormalized_d_undistorted(0, 0) = _diagonal;
        d_unnormalized_d_undistorted(0, 1) = 0;
        d_unnormalized_d_undistorted(1, 0) = 0;
        d_unnormalized_d_undistorted(1, 1) = _diagonal;

        Eigen::Matrix<double, 2, 2> d_normalized_d_centered;
        d_normalized_d_centered(0, 0) = 1.0 / _diagonal;
        d_normalized_d_centered(0, 1) = 0;
        d_normalized_d_centered(1, 0) = 0;
        d_normalized_d_centered(1, 1) = 1.0 / _diagonal;

        return  d_unnormalized_d_undistorted * getDerivativeUndistortNormalizedwrtPoint(normalized) * d_normalized_d_centered * -1.0;
    }
    
    virtual Vec2 undistortNormalized(const Vec2& p) const = 0;
    virtual Eigen::Matrix<double, 2, Eigen::Dynamic> getDerivativeUndistortNormalizedwrtParameters(const Vec2& p) = 0;
    virtual Eigen::Matrix<double, 2, 2> getDerivativeUndistortNormalizedwrtPoint(const Vec2& p) = 0;

    virtual ~Undistortion() = default;

protected:
    Vec2 _center;
    Vec2 _offset;
    double _diagonal;
    
    std::vector<double> _undistortionParams{};
};

} // namespace camera
} // namespace aliceVision
