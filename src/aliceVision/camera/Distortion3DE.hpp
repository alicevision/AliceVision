// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/camera/Distortion.hpp>
#include <aliceVision/system/Logger.hpp>

#include <cmath>

namespace aliceVision {
namespace camera {

/**
 * @class DistortionRadial3DE
 * @brief 3DE radial distortion
 */
class Distortion3DERadial4 : public Distortion
{
  public:
    /**
     * @brief Default constructor, no distortion.
     */
    Distortion3DERadial4() { _distortionParams = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; }

    /**
     * @brief Constructor with the three coefficients
     */
    explicit Distortion3DERadial4(double c2, double c4, double u1, double v1, double u3, double v3) { _distortionParams = {c2, c4, u1, v1, u3, v3}; }

    EDISTORTION getType() const override { return EDISTORTION::DISTORTION_3DERADIAL4; }

    Distortion3DERadial4* clone() const override { return new Distortion3DERadial4(*this); }

    /// Add distortion to the point p (assume p is in the camera frame [normalized coordinates])
    Vec2 addDistortion(const Vec2& p) const override;

    Eigen::Matrix2d getDerivativeAddDistoWrtPt(const Vec2& p) const override;

    Eigen::MatrixXd getDerivativeAddDistoWrtDisto(const Vec2& p) const override;

    /// Remove distortion (return p' such that disto(p') = p)
    Vec2 removeDistortion(const Vec2& p) const override;

    Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(const Vec2& p) const override
    {
        ALICEVISION_THROW_ERROR("Invalid class for getDerivativeRemoveDistoWrtPt");
    }

    Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(const Vec2& p) const override
    {
        ALICEVISION_THROW_ERROR("Invalid class for getDerivativeRemoveDistoWrtDisto");
    }

    double getUndistortedRadius(double r) const override { ALICEVISION_THROW_ERROR("Invalid class for getUndistortedRadius"); }

    ~Distortion3DERadial4() override = default;
};

/**
 * @class DistortionRadialAnamorphic4
 * @brief Anamorphic radial distortion
 */
class Distortion3DEAnamorphic4 : public Distortion
{
  public:
    /**
     * @brief Default constructor, no distortion.
     */
    Distortion3DEAnamorphic4() { _distortionParams = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0}; }

    /**
     * @brief Constructor with the three coefficients
     */
    explicit Distortion3DEAnamorphic4(double cx02,
                                      double cy02,
                                      double cx22,
                                      double cy22,
                                      double cx04,
                                      double cy04,
                                      double cx24,
                                      double cy24,
                                      double cx44,
                                      double cy44,
                                      double phi,
                                      double sqx,
                                      double sqy,
                                      double ps)
    {
        _distortionParams = {cx02, cy02, cx22, cy22, cx04, cy04, cx24, cy24, cx44, cy44, phi, sqx, sqy, ps};
    }

    EDISTORTION getType() const override { return EDISTORTION::DISTORTION_3DEANAMORPHIC4; }

    Distortion3DEAnamorphic4* clone() const override { return new Distortion3DEAnamorphic4(*this); }

    /// Add distortion to the point p (assume p is in the camera frame [normalized coordinates])
    Vec2 addDistortion(const Vec2& p) const override;

    Eigen::Matrix2d getDerivativeAddDistoWrtPt(const Vec2& p) const override;

    Eigen::MatrixXd getDerivativeAddDistoWrtDisto(const Vec2& p) const override;

    /// Remove distortion (return p' such that disto(p') = p)
    Vec2 removeDistortion(const Vec2& p) const override;

    Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(const Vec2& p) const override
    {
        ALICEVISION_THROW_ERROR("Invalid class for getDerivativeRemoveDistoWrtPt");
    }

    Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(const Vec2& p) const override
    {
        ALICEVISION_THROW_ERROR("Invalid class for getDerivativeRemoveDistoWrtDisto");
    }

    double getUndistortedRadius(double r) const override { ALICEVISION_THROW_ERROR("Invalid class for getUndistortedRadius"); }

    ~Distortion3DEAnamorphic4() override = default;
};

/**
 * @class Distortion3DEClassicLD
 * @brief Anamorphic radial distortion
 */
class Distortion3DEClassicLD : public Distortion
{
  public:
    /**
     * @brief Default constructor, no distortion.
     */
    Distortion3DEClassicLD() { _distortionParams = {0.0, 1.0, 0.0, 0.0, 0.0}; }

    /**
     * @brief Constructor with the three coefficients
     */
    explicit Distortion3DEClassicLD(double delta, double epsilon, double mux, double muy, double q)
    {
        _distortionParams = {delta, 1.0 / epsilon, mux, muy, q};
    }

    EDISTORTION getType() const override { return EDISTORTION::DISTORTION_3DECLASSICLD; }

    Distortion3DEClassicLD* clone() const override { return new Distortion3DEClassicLD(*this); }

    /// Add distortion to the point p (assume p is in the camera frame [normalized coordinates])
    Vec2 addDistortion(const Vec2& p) const override;

    Eigen::Matrix2d getDerivativeAddDistoWrtPt(const Vec2& p) const override;

    Eigen::MatrixXd getDerivativeAddDistoWrtDisto(const Vec2& p) const override;

    /// Remove distortion (return p' such that disto(p') = p)
    Vec2 removeDistortion(const Vec2& p) const override;

    Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(const Vec2& p) const override
    {
        ALICEVISION_THROW_ERROR("Invalid class for getDerivativeRemoveDistoWrtPt");
    }

    Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(const Vec2& p) const override
    {
        ALICEVISION_THROW_ERROR("Invalid class for getDerivativeRemoveDistoWrtDisto");
    }

    double getUndistortedRadius(double r) const override { ALICEVISION_THROW_ERROR("Invalid class for getUndistortedRadius"); }

    ~Distortion3DEClassicLD() override = default;
};

}  // namespace camera
}  // namespace aliceVision
