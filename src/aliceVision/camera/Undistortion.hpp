#pragma once

#include <aliceVision/numeric/numeric.hpp>

#include <vector>
#include <string>

namespace aliceVision{
namespace camera{

class Undistortion
{
public:
    enum Type
    {
        NONE,
        ANAMORPHIC4
    };

    static std::string enumToString(Type type)
    {
        switch (type)
        {
        case ANAMORPHIC4: 
            return "anamorphic4";
        default: 
            return "none";
        }
    }

    static Type stringToEnum(const std::string & typestr)
    {
        std::string type = typestr;
        std::transform(type.begin(), type.end(), type.begin(), ::tolower); //tolower

        if (type == "anamorphic4") return Type::ANAMORPHIC4;
        
        return Type::NONE;
    }

    static std::shared_ptr<Undistortion> create(const Type& type, int width, int height);

    Undistortion(int width, int height)
    {
        _size = { width, height };
        _diagonal = sqrt(width * width + height * height);
        _center = {width / 2, height / 2};
        _offset = { 0.0, 0.0 };
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

    Vec2 getSize() const
    {
        return _size;
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

    virtual Type getType() const = 0 ;
    
    Vec2 undistort(const Vec2& p) const
    {
        Vec2 centered;
        
        centered(0) = p(0) - _center(0) -_offset(0);
        centered(1) = p(1) - _center(1) -_offset(1);

        Vec2 normalized;
        normalized(0) = centered(0) / _diagonal;
        normalized(1) = centered(1) / _diagonal;

        Vec2 undistorted = undistortNormalized(normalized);
        Vec2 unnormalized;
        
        unnormalized(0) = undistorted(0) * _diagonal + _center(0) + _offset(0);
        unnormalized(1) = undistorted(1) * _diagonal + _center(1) + _offset(1);

        return unnormalized;
    }

    Eigen::Matrix<double, 2, Eigen::Dynamic> getDerivativeUndistortWrtParameters(const Vec2 &p)
    {
        Vec2 centered = p - _center - _offset;
        Vec2 normalized = centered / _diagonal;
        Vec2 undistorted = undistortNormalized(normalized);
        Vec2 unnormalized = undistorted * _diagonal + _center + _offset;

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
        Vec2 unnormalized = undistorted * _diagonal + _center + _offset;

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

        return  Eigen::Matrix2d::Identity() + d_unnormalized_d_undistorted * getDerivativeUndistortNormalizedwrtPoint(normalized) * d_normalized_d_centered * -1.0;
    }

    /// add distortion (return p' such that undisto(p') = p)
    Vec2 inverse(const Vec2& p) const
    {
        Vec2 centered;

        centered(0) = p(0) - _center(0) - _offset(0);
        centered(1) = p(1) - _center(1) - _offset(1);

        Vec2 normalized;
        normalized(0) = centered(0) / _diagonal;
        normalized(1) = centered(1) / _diagonal;

        Vec2 distorted = inverseNormalized(normalized);
        Vec2 unnormalized;

        unnormalized(0) = distorted(0) * _diagonal + _center(0) + _offset(0);
        unnormalized(1) = distorted(1) * _diagonal + _center(1) + _offset(1);

        return unnormalized;
    }
    
    virtual Vec2 inverseNormalized(const Vec2& p) const = 0;
    virtual Vec2 undistortNormalized(const Vec2& p) const = 0;
    virtual Eigen::Matrix<double, 2, Eigen::Dynamic> getDerivativeUndistortNormalizedwrtParameters(const Vec2& p) const = 0;
    virtual Eigen::Matrix<double, 2, 2> getDerivativeUndistortNormalizedwrtPoint(const Vec2& p) const = 0;

    virtual ~Undistortion() = default;

protected:
    Vec2 _size;
    Vec2 _center;
    Vec2 _offset;
    double _diagonal;
    std::vector<double> _undistortionParams{};
};

} // namespace camera
} // namespace aliceVision
