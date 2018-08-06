// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/camera/cameraCommon.hpp>

#include <vector>

namespace aliceVision {
namespace camera {

/// Implement a Pinhole camera with a 3 radial distortion coefficients and 2 tangential distortion coefficients.
/// x_d = x_u (1 + K_1 r^2 + K_2 r^4 + K_3 r^6) + (T_2 (r^2 + 2 x_u^2) + 2 T_1 x_u y_u)
/// y_d = y_u (1 + K_1 r^2 + K_2 r^4 + K_3 r^6) + (T_1 (r^2 + 2 y_u^2) + 2 T_2 x_u y_u)
class PinholeBrownT2 : public Pinhole
{
    public:

    PinholeBrownT2(
        int w = 0, int h = 0,
        double focal = 0.0, double ppx = 0, double ppy = 0,
        double k1 = 0.0, double k2 = 0.0, double k3 = 0.0,
        double t1 = 0.0, double t2 = 0.0)
            :Pinhole(w, h, focal, ppx, ppy, {k1, k2, k3, t1, t2})
    {
    }

    PinholeBrownT2* clone() const { return new PinholeBrownT2(*this); }
    void assign(const IntrinsicBase& other) { *this = dynamic_cast<const PinholeBrownT2&>(other); }
  
    EINTRINSIC getType() const { return PINHOLE_CAMERA_BROWN; }

    virtual bool have_disto() const { return true;}

    virtual Vec2 add_disto(const Vec2 & p) const{
        return (p + distoFunction(_distortionParams, p));
    }

    // numerical approximation based on
    // Heikkila J (2000) Geometric Camera Calibration Using Circular Control Points.
    // IEEE Trans. Pattern Anal. Mach. Intell., 22:1066-1077

    virtual Vec2 remove_disto(const Vec2 & p) const{
        const double epsilon = 1e-8; //criteria to stop the iteration
        Vec2 p_u = p;

        while((add_disto(p_u)-p).lpNorm<1>() > epsilon)//manhattan distance between the two points
        {
            p_u = p - distoFunction(_distortionParams, p_u);
        }

        return p_u;
    }

    /// Return the un-distorted pixel (with removed distortion)
    virtual Vec2 get_ud_pixel(const Vec2& p) const
    {
      return cam2ima( remove_disto(ima2cam(p)) );
    }

    /// Return the distorted pixel (with added distortion)
    virtual Vec2 get_d_pixel(const Vec2& p) const
    {
      return cam2ima( add_disto(ima2cam(p)) );
    }

    private:

    /// Functor to calculate distortion offset accounting for both radial and tangential distortion
    static Vec2 distoFunction(const std::vector<double> & params, const Vec2 & p)
    {
        const double k1 = params[0], k2 = params[1], k3 = params[2], t1 = params[3], t2 = params[4];
        const double r2 = p(0)*p(0) + p(1)*p(1);
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;
        const double k_diff = (k1*r2 + k2*r4 + k3*r6);
        const double t_x = t2 * (r2 + 2 * p(0)*p(0)) + 2 * t1 * p(0) * p(1);
        const double t_y = t1 * (r2 + 2 * p(1)*p(1)) + 2 * t2 * p(0) * p(1);
        Vec2 d(p(0) * k_diff + t_x, p(1) * k_diff + t_y);
        return d;
    }
};

} // namespace camera
} // namespace aliceVision
