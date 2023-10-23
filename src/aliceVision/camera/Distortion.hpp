// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/camera/cameraCommon.hpp>

#include <aliceVision/numeric/numeric.hpp>

#include <vector>

namespace aliceVision {
namespace camera {

/**
 * @brief Abstract class to model the distortion of a camera-lense couple using a set of parameters.
 * @note Distortion models are expressed in terms of the camera's focal length.
 */
class Distortion
{
  public:
    Distortion() = default;

    virtual EDISTORTION getType() const = 0;

    virtual Distortion* clone() const = 0;

    // not virtual as child classes do not hold any data
    bool operator==(const Distortion& other) const { return _distortionParams == other._distortionParams; }

    void setParameters(const std::vector<double>& params)
    {
        if (_distortionParams.size() != params.size())
        {
            return;
        }

        for (int i = 0; i < _distortionParams.size(); i++)
        {
            _distortionParams[i] = params[i];
        }
    }

    inline std::vector<double>& getParameters() { return _distortionParams; }

    inline const std::vector<double>& getParameters() const { return _distortionParams; }

    inline std::size_t getDistortionParametersCount() const { return _distortionParams.size(); }

    /// Add distortion to the point p (assume p is in the camera frame [normalized coordinates])
    virtual Vec2 addDistortion(const Vec2& p) const { return p; }

    /// Remove distortion (return p' such that disto(p') = p)
    virtual Vec2 removeDistortion(const Vec2& p) const { return p; }

    virtual double getUndistortedRadius(double r) const { return r; }

    virtual Eigen::Matrix2d getDerivativeAddDistoWrtPt(const Vec2& p) const { return Eigen::Matrix2d::Identity(); }

    virtual Eigen::MatrixXd getDerivativeAddDistoWrtDisto(const Vec2& p) const { return Eigen::MatrixXd(0, 0); }

    virtual Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(const Vec2& p) const { return Eigen::Matrix2d::Identity(); }

    virtual Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(const Vec2& p) const { return Eigen::MatrixXd(0, 0); }

    virtual ~Distortion() = default;

  protected:
    std::vector<double> _distortionParams{};
};

}  // namespace camera
}  // namespace aliceVision
