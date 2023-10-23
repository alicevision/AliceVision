// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/camera/Distortion.hpp>

namespace aliceVision {
namespace camera {

namespace radial_distortion {

/**
 * @brief Solve by bisection the p' radius such that Square(disto(radius(p'))) = r^2
 * @param[in] params radial distortion parameters.
 * @param[in] r2 targeted radius
 * @param[in] functor functor to solve Square(disto(radius(p'))) = r^2
 * @param epsilon minimum error to stop the bisection
 * @return optimal radius
 */
template<class Disto_Functor>
double bisection_Radius_Solve(const std::vector<double>& params, double r2, Disto_Functor& functor, double epsilon = 1e-8)
{
    // Guess plausible upper and lower bound
    double lowerbound = r2, upbound = r2;
    while (functor(params, lowerbound) > r2)
        lowerbound /= 1.05;
    while (functor(params, upbound) < r2)
        upbound *= 1.05;

    // Perform a bisection until epsilon accuracy is not reached
    while (epsilon < (upbound - lowerbound))
    {
        const double mid = .5 * (lowerbound + upbound);
        if (functor(params, mid) > r2)
            upbound = mid;
        else
            lowerbound = mid;
    }

    return .5 * (lowerbound + upbound);
}

}  // namespace radial_distortion

/**
 * @class DistortionRadialK1
 * @brief Radial distortion with a single distortion coefficient.
 * \f$ x_d = x_u (1 + K_1 r^2 ) \f$
 */
class DistortionRadialK1 : public Distortion
{
  public:
    /**
     * @brief Default contructor, no distortion
     */
    DistortionRadialK1() { _distortionParams = {0.0}; }

    /**
     * @brief Constructor with the single distortion coefficient.
     * @param[in] k1 the distortion coefficient
     */
    explicit DistortionRadialK1(double k1) { _distortionParams = {k1}; }

    EDISTORTION getType() const override { return EDISTORTION::DISTORTION_RADIALK1; }

    DistortionRadialK1* clone() const override { return new DistortionRadialK1(*this); }

    /// Add distortion to the point p (assume p is in the camera frame [normalized coordinates])
    Vec2 addDistortion(const Vec2& p) const override;

    Eigen::Matrix2d getDerivativeAddDistoWrtPt(const Vec2& p) const override;

    Eigen::MatrixXd getDerivativeAddDistoWrtDisto(const Vec2& p) const override;

    /// Remove distortion (return p' such that disto(p') = p)
    Vec2 removeDistortion(const Vec2& p) const override;

    Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(const Vec2& p) const override;

    Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(const Vec2& p) const override;

    double getUndistortedRadius(double r) const override;

    /// Functor to solve Square(disto(radius(p'))) = r^2
    static double distoFunctor(const std::vector<double>& params, double r2);

    ~DistortionRadialK1() override = default;
};

/**
 * @class DistortionRadialK3
 * @brief Radial distortion modeled with a 6th degree polynomial with three distortion coefficients.
 * \f$ x_d = x_u (1 + K_1 r^2 + K_2 r^4 + K_3 r^6) \f$
 */
class DistortionRadialK3 : public Distortion
{
  public:
    /**
     * @brief Default constructor, no distortion.
     */
    DistortionRadialK3() { _distortionParams = {0.0, 0.0, 0.0}; }

    /**
     * @brief Constructor with the three coefficients
     * @param[in] k1 first coefficient
     * @param[in] k2 second coefficient
     * @param[in] k3 third coefficient
     */
    explicit DistortionRadialK3(double k1, double k2, double k3) { _distortionParams = {k1, k2, k3}; }

    EDISTORTION getType() const override { return EDISTORTION::DISTORTION_RADIALK3; }

    DistortionRadialK3* clone() const override { return new DistortionRadialK3(*this); }

    /// Add distortion to the point p (assume p is in the camera frame [normalized coordinates])
    Vec2 addDistortion(const Vec2& p) const override;

    Eigen::Matrix2d getDerivativeAddDistoWrtPt(const Vec2& p) const override;

    Eigen::MatrixXd getDerivativeAddDistoWrtDisto(const Vec2& p) const override;

    /// Remove distortion (return p' such that disto(p') = p)
    Vec2 removeDistortion(const Vec2& p) const override;

    Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(const Vec2& p) const override;

    Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(const Vec2& p) const override;

    double getUndistortedRadius(double r) const override;

    /// Functor to solve Square(disto(radius(p'))) = r^2
    static double distoFunctor(const std::vector<double>& params, double r2);

    ~DistortionRadialK3() override = default;
};

class DistortionRadialK3PT : public Distortion
{
  public:
    DistortionRadialK3PT() { _distortionParams = {0.0, 0.0, 0.0}; }

    explicit DistortionRadialK3PT(double k1, double k2, double k3) { _distortionParams = {k1, k2, k3}; }

    EDISTORTION getType() const override { return EDISTORTION::DISTORTION_RADIALK3PT; }

    DistortionRadialK3PT* clone() const override { return new DistortionRadialK3PT(*this); }

    /// Add distortion to the point p (assume p is in the camera frame [normalized coordinates])
    Vec2 addDistortion(const Vec2& p) const override;

    Eigen::Matrix2d getDerivativeAddDistoWrtPt(const Vec2& p) const override;

    Eigen::MatrixXd getDerivativeAddDistoWrtDisto(const Vec2& p) const override;

    Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(const Vec2& p) const override;

    Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(const Vec2& p) const override;

    /// Remove distortion (return p' such that disto(p') = p)
    Vec2 removeDistortion(const Vec2& p) const override;

    double getUndistortedRadius(double r) const override;

    /// Functor to solve Square(disto(radius(p'))) = r^2
    static double distoFunctor(const std::vector<double>& params, double r2);

    ~DistortionRadialK3PT() override = default;
};

}  // namespace camera
}  // namespace aliceVision
