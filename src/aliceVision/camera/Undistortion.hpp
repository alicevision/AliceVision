// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/camera/cameraCommon.hpp>

#include <aliceVision/numeric/numeric.hpp>

#include <vector>
#include <string>
#include <memory>

namespace aliceVision {
namespace camera {

/**
 * @brief Abstract class to represent the inverse operation of a distortion model.
 *
 * Contrary to distortion models, the undistortion process is a pixel-to-pixel operation
 * and thus it does not depend on the camera's focal length.
 *
 * Its main purpose is to undistort images before they are used in other calculations
 * so that we can virtually "remove" distortion parameters from the camera.
 */
class Undistortion
{
  public:
    Undistortion(int width, int height)
    {
        setSize(width, height);
        setOffset({0.0, 0.0});
        _pixelAspectRatio = 1.0;
    }

    virtual EUNDISTORTION getType() const = 0;

    virtual Undistortion* clone() const = 0;

    // not virtual as child classes do not hold any data
    bool operator==(const Undistortion& other) const { return _undistortionParams == other._undistortionParams; }

    void setOffset(const Vec2& offset) { _offset = offset; }

    void setSize(int width, int height)
    {
        _size = {width, height};
        double hh = height / _pixelAspectRatio;
        _diagonal = sqrt(width * width + hh * hh) * 0.5;
        _center = {width / 2, height / 2};
    }

    void setDiagonal(double diagonal)
    {
        //May be used for plates with a different size than lens grid
        _diagonal = diagonal;
    }

    void setPixelAspectRatio(double pixelAspectRatio) 
    { 
        _pixelAspectRatio = pixelAspectRatio; 
        double hh = _size.y() / _pixelAspectRatio;
        _diagonal = sqrt(_size.x() * _size.x() + hh * hh) * 0.5;
    }

    inline Vec2 getOffset() const { return _offset; }

    inline Vec2 getScaledOffset() const { return _offset / _diagonal; }

    Vec2 getSize() const { return _size; }

    inline double getDiagonal() const { return _diagonal; }

    double getPixelAspectRatio() const { return _pixelAspectRatio; }
    
    const std::vector<double>& getParameters() const { return _undistortionParams; }

    void setParameters(const std::vector<double>& params)
    {
        if (_undistortionParams.size() != params.size())
        {
            return;
        }

        for (int i = 0; i < _undistortionParams.size(); i++)
        {
            _undistortionParams[i] = params[i];
        }
    }

    std::size_t getUndistortionParametersCount() const { return _undistortionParams.size(); }

    Vec2 undistort(const Vec2& p) const;

    Eigen::Matrix<double, 2, Eigen::Dynamic> getDerivativeUndistortWrtParameters(const Vec2& p);

    Eigen::Matrix<double, 2, 2> getDerivativeUndistortWrtOffset(const Vec2& p);

    /// add distortion (return p' such that undisto(p') = p)
    Vec2 inverse(const Vec2& p) const;

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
    double _pixelAspectRatio;
    std::vector<double> _undistortionParams{};
};

}  // namespace camera
}  // namespace aliceVision
