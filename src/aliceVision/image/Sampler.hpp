// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/image/Image.hpp"
#include "aliceVision/image/pixelTypes.hpp"

namespace aliceVision {
namespace image {

/**
** Sampling functors
** These functors computes weight associated to each pixels

  For a (relative) sampling position x (\in [0,1]) between two (consecutives) points :

    A  .... x ......... B

  w[0] is the weight associated to A
  w[1] is the weight associated to B

  Note: The following functors generalize the sampling to more than two neighbors
  ** They all contains the neighbor_width variable that specify the number of neighbors used for sampling
*/

/**
 ** Nearest sampling (ie: find the nearest pixel to a specified position)
 **/
struct SamplerNearest
{
  public:
    // Nearest sampling is only between two pixels
    static const int neighborWidth = 2;

    /**
     ** @brief Computes weight associated to neighboring pixels
     ** @author Romuald Perrot <perrot.romuald_AT_gmail.com>
     ** @param x Sampling position
     ** @param[out] weigth Sampling factors associated to the neighboring
     ** @note weight must be at least neighborWidth length
     **/
    void operator()(const double x, double* const weight) const
    {
        weight[0] = (x < 0.5) ? 1.0 : 0.0;
        weight[1] = (x >= 0.5) ? 1.0 : 0.0;
    }
};

/**
 ** Linear sampling (ie: linear interpolation between two pixels)
 **/
struct SamplerLinear
{
  public:
    // Linear sampling is between two pixels
    static const int neighborWidth = 2;

    /**
     ** @brief Computes weight associated to neighboring pixels
     ** @author Romuald Perrot <perrot.romuald_AT_gmail.com>
     ** @param x Sampling position
     ** @param[out] weight Sampling factors associated to the neighboring
     ** @note weight must be at least neighborWidth length
     **/
    void operator()(const double x, double* const weight) const
    {
        weight[0] = 1.0 - x;
        weight[1] = x;
    }
};

/**
 ** Cubic interpolation between 4 pixels
 **
 ** Interpolation weight is for A,B,C and D pixels given a x position as illustrated as follow :
 **
 ** A      B    x C      D
 **
 ** @ref : Cubic Convolution Interpolation for Digital Image Processing , R. Keys, eq(4)
 **/
struct SamplerCubic
{
  public:
    // Cubic interpolation is between 4 pixels
    static const int neighborWidth = 4;

    /**
     ** @brief Constructor
     ** @param sharpnessCoef Sharpness coefficient used to control sharpness of the cubic curve
     ** @note sharpnessCoef must be between -0.75 to -0.5
     ** -0.5 gives better mathematically result (ie: approximation at 3 order precision)
     **/
    SamplerCubic(const double sharpnessCoef = -0.5)
      : _sharpness(sharpnessCoef)
    {}

    /**
     ** @brief Computes weight associated to neighboring pixels
     ** @author Romuald Perrot <perrot.romuald_AT_gmail.com>
     ** @param x Sampling position
     ** @param[out] weight Sampling factors associated to the neighboring
     ** @note weight must be at least neighborWidth length
     **/
    void operator()(const double x, double* const weight) const
    {
        // remember :
        // A      B    x  C       D

        // weigth[0] -> weight for A
        // weight[1] -> weight for B
        // weight[2] -> weight for C
        // weight[3] -> weigth for D

        weight[0] = CubicInter12(_sharpness, x + 1.0);
        weight[1] = CubicInter01(_sharpness, x);
        weight[2] = CubicInter01(_sharpness, 1.0 - x);
        weight[3] = CubicInter12(_sharpness, 2.0 - x);
    }

  private:
    // Cubic interpolation for x \in [0 ; 1 [
    static double CubicInter01(const double sharpness, const double x)
    {
        // A = sharpness
        //  f(x) = ( A + 2 ) * x ^ 3 - ( A + 3 ) * x ^ 2 + 1
        return ((sharpness + 2.0) * x - (sharpness + 3.0)) * x * x + 1.0;
    }

    // Cubic interpolation for x \in [1 ; 2 [
    static double CubicInter12(const double sharpness, const double x)
    {
        // A = sharpness
        // f(x) = A * x^3 - 5 * A * x^2 + 8 * A * x - 4 * a

        return ((sharpness * x - 5.0 * sharpness) * x + 8.0 * sharpness) * x - 4.0 * sharpness;
    }

    // Sharpness coefficient
    double _sharpness;
};

/**
 ** Sampler spline16 -> Interpolation on 4 points used for 2d resampling (16 = 4x4 sampling)
 ** Cubic interpolation with 0-derivative at edges (ie at A and D points)
 ** See Helmut Dersch for more details
 **
 ** Some refs :
 **  -   http://forum.doom9.org/archive/index.php/t-147117.html
 **  -   http://avisynth.nl/index.php/Resampling
 **  -   http://www.ipol.im/pub/art/2011/g_lmii/
 **
 ** The idea is to consider 3 cubic splines (f1,f2,f3) in the sampling interval :
 **
 ** A   f1    B    f2    C     f3     D
 **
 ** with curves defined as follow :
 ** f1(x) = a1 x^3 + b1 x^2 + c1 x + d1
 ** f2(x) = a2 x^3 + b2 x^2 + c2 x + d2
 ** f3(x) = a3 x^3 + b2 x^2 + c3 x + d3
 **
 ** We want to compute spline coefs for A,B,C,D assuming that:
 ** y0 = coef[A] = f1(-1)
 ** y1 = coef[B] = f1(0) = f2(0)
 ** y2 = coef[C] = f2(1) = f3(1)
 ** y3 = coef[D] = f3(2)
 **
 ** coef are computed using the following constraints :
 ** Curve is continuous, ie:
 ** f1(0)  = f2(0)
 ** f2(1)  = f3(1)
 ** First derivative are equals, ie:
 ** f1'(0) = f2'(0)
 ** f2'(1) = f3'(1)
 ** Second derivative are equals, ie:
 ** f1''(0) = f2''(0)
 ** f2''(1) = f3''(0)
 ** Curve is, at boundary, with second derivative set to zero (it's a constraint introduced by Dersch), ie:
 ** f1''(-1) = 0
 ** f3''(2) = 0
 **
 ** Then, you can solve for (a1,a2,a3,b1,b2,b3,c1,c2,c3,d1,d2,d3)
 **
 ** for ex, for curve f2 you find :
 **
 ** d2 = y1                                      // easy since y1 = f2(0)
 ** c2 = - 7/15 y0 - 1/5 y1 + 4/5 y2 - 2/15 y3
 ** b2 = 4/5 y0 - 9/5 y1 + 6/5 y2 - 1/5 y3
 ** a2 = - 1/3 y0 + y1 - y2 + 1/3 y3
 **
 **
 ** When you have coefs, you just have to express your curve as a linear combinaison of the control points, fort ex
 ** with f2 :
 **
 **
 ** f2(x) = w0(x) * y0 + w1(x) + y1 + w2(x) * y2 + w3(x) * y3
 **
 ** with :
 **
 ** w0(x) = - 1/3 * x^3 + 4/5 * x^2 - 7/15 * x
 ** w1(x) = x^3 - 9/5 * x^2 - 1/5 * x + 1
 ** w2(x) = -x^3 + 6/5 * x^2 + 4/5 * x
 ** w3(x) = 1/3 * x^3 - 1/5 * x^2 - 2/15 * x
 **
 ** substituing boundary conditions gives the correct coeficients for y0,y1,y2,y3 giving the final sampling scheme
 **/
struct SamplerSpline16
{
  public:
    static const int neighborWidth = 4;

    /**
     ** @brief Computes weight associated to neighboring pixels
     ** @author Romuald Perrot <perrot.romuald_AT_gmail.com>
     ** @param x Sampling position
     ** @param[out] weight Sampling factors associated to the neighboring
     ** @note weight must be at least neighborWidth length
     **/
    void operator()(const double x, double* const weight) const
    {
        weight[0] = ((-1.0 / 3.0 * x + 4.0 / 5.0) * x - 7.0 / 15.0) * x;
        weight[1] = ((x - 9.0 / 5.0) * x - 1.0 / 5.0) * x + 1.0;
        weight[2] = ((6.0 / 5.0 - x) * x + 4.0 / 5.0) * x;
        weight[3] = ((1.0 / 3.0 * x - 1.0 / 5.0) * x - 2.0 / 15.0) * x;
    }
};

/**
 ** Sampler spline 36
 ** Same as spline 16 but on 6 neighbors (used for 6x6 frame)
 **/
struct SamplerSpline36
{
  public:
    static const int neighborWidth = 6;

    /**
     ** @brief Computes weight associated to neighboring pixels
     ** @author Romuald Perrot <perrot.romuald_AT_gmail.com>
     ** @param x Sampling position
     ** @param[out] weight Sampling factors associated to the neighboring
     ** @note weight must be at least neighborWidth length
     **/
    void operator()(const double x, double* const weight) const
    {
        weight[0] = ((1.0 / 11.0 * x - 45.0 / 209.0) * x + 26.0 / 209.0) * x;
        weight[1] = ((-6.0 / 11.0 * x + 270.0 / 209.0) * x - 156.0 / 209.0) * x;
        weight[2] = ((13.0 / 11.0 * x - 453.0 / 209.0) * x - 3.0 / 209.0) * x + 1.0;
        weight[3] = ((-13.0 / 11.0 * x + 288.0 / 209.0) * x + 168.0 / 209.0) * x;
        weight[4] = ((6.0 / 11.0 * x - 72.0 / 209.0) * x - 42.0 / 209.0) * x;
        weight[5] = ((-1.0 / 11.0 * x + 12.0 / 209.0) * x + 7.0 / 209.0) * x;
    }
};

/**
 ** Sampler spline 64
 ** Same as spline 16 but on 8 neighbors (used for 8x8 frame)
 **/
struct SamplerSpline64
{
  public:
    static const int neighborWidth = 8;

    /**
     ** @brief Computes weight associated to neighboring pixels
     ** @author Romuald Perrot <perrot.romuald_AT_gmail.com>
     ** @param x Sampling position
     ** @param[out] weight Sampling factors associated to the neighboring
     ** @note weight must be at least neighborWidth length
     **/
    void operator()(const double x, double* const weight) const
    {
        weight[0] = ((-1.0 / 41.0 * x + 168.0 / 2911.0) * x - 97.0 / 2911.0) * x;
        weight[1] = ((6.0 / 41.0 * x - 1008.0 / 2911.0) * x + 582.0 / 2911.0) * x;
        weight[2] = ((-24.0 / 41.0 * x + 4032.0 / 2911.0) * x - 2328.0 / 2911.0) * x;
        weight[3] = ((49.0 / 41.0 * x - 6387.0 / 2911.0) * x - 3.0 / 2911.0) * x + 1.0;
        weight[4] = ((-49.0 / 41.0 * x + 4050.0 / 2911.0) * x + 2340.0 / 2911.0) * x;
        weight[5] = ((24.0 / 41.0 * x - 1080.0 / 2911.0) * x - 624.0 / 2911.0) * x;
        weight[6] = ((-6.0 / 41.0 * x + 270.0 / 2911.0) * x + 156.0 / 2911.0) * x;
        weight[7] = ((1.0 / 41.0 * x - 45.0 / 2911.0) * x - 26.0 / 2911.0) * x;
    }
};

template<typename T>
struct RealPixel
{
    typedef T base_type;
    typedef double real_type;

    static real_type convert_to_real(const base_type& val) { return static_cast<real_type>(val); }

    static base_type convert_from_real(const real_type& val) { return static_cast<base_type>(val); }

    static real_type zero() { return real_type(0); }
};

// overloading for unsigned char
template<>
struct RealPixel<unsigned char>
{
    typedef unsigned char base_type;
    typedef double real_type;

    static real_type convert_to_real(const base_type& val) { return static_cast<real_type>(val); }

    static base_type convert_from_real(const real_type& val)
    {
        // handle out of range values.
        return (val < 0.0) ? 0
                           : (val > static_cast<real_type>(std::numeric_limits<base_type>::max()) ? std::numeric_limits<base_type>::max()
                                                                                                  : static_cast<base_type>(val + 0.5));
    }

    static real_type zero() { return 0.; }
};

// overloading for float
template<>
struct RealPixel<float>
{
    typedef float base_type;
    typedef double real_type;

    static real_type convert_to_real(const base_type& val) { return static_cast<real_type>(val); }

    static base_type convert_from_real(const real_type& val) { return static_cast<base_type>(val); }

    static real_type zero() { return real_type(0); }
};

// overloading for Rgb
template<typename T>
struct RealPixel<Rgb<T>>
{
    typedef Rgb<T> base_type;
    typedef Rgb<double> real_type;

    static real_type convert_to_real(const base_type& val) { return real_type(val.template cast<double>()); }

    static base_type convert_from_real(const real_type& val)
    {
        return base_type(
          RealPixel<T>::convert_from_real(val.r()), RealPixel<T>::convert_from_real(val.g()), RealPixel<T>::convert_from_real(val.b()));
    }

    static real_type zero() { return real_type(real_type::Zero()); }
};

// overloading for rgba
template<typename T>
struct RealPixel<Rgba<T>>
{
    typedef Rgba<T> base_type;
    typedef Rgba<double> real_type;

    static real_type convert_to_real(const base_type& val) { return real_type(val.template cast<double>()); }

    static base_type convert_from_real(const real_type& val)
    {
        return base_type(RealPixel<T>::convert_from_real(val.r()),
                         RealPixel<T>::convert_from_real(val.g()),
                         RealPixel<T>::convert_from_real(val.b()),
                         RealPixel<T>::convert_from_real(val.a()));
    }

    static real_type zero() { return real_type(real_type::Zero()); }
};

/**
 ** Generic sampling of image using a sampling function
 **/
template<typename SamplerFunc>
struct Sampler2d
{
    Sampler2d(const SamplerFunc& sampler = SamplerFunc())
      : _sampler(sampler),
        _halfWidth(SamplerFunc::neighborWidth / 2)
    {}

    /**
     ** Sample image at a specified position
     ** @param src Input image
     ** @param y Y-coordinate of sampling
     ** @param x X-coordinate of sampling
     ** @return Sampled value
     **/
    template<typename T>
    T operator()(const Image<T>& src, const float y, const float x) const
    {
        const int imWidth = src.width();
        const int imHeight = src.height();

        // Get sampler coefficients
        double coefsX[SamplerFunc::neighborWidth];
        double coefsY[SamplerFunc::neighborWidth];

        // Compute difference between exact pixel location and sample
        const double dx = static_cast<double>(x) - floor(x);
        const double dy = static_cast<double>(y) - floor(y);

        // Get sampler weights
        _sampler(dx, coefsX);
        _sampler(dy, coefsY);

        auto res = RealPixel<T>::zero();

        // integer position of sample (x,y)
        const int gridX = static_cast<int>(floor(x));
        const int gridY = static_cast<int>(floor(y));

        // Sample a grid around specified grid point
        double totalWeight = 0.0;
        for (int i = 0; i < SamplerFunc::neighborWidth; ++i)
        {
            // Get current i value
            // +1 for correct scheme (draw it to be conviced)
            const int iCurrent = gridY + 1 + i - _halfWidth;

            // handle out of range
            if (iCurrent < 0 || iCurrent >= imHeight)
            {
                continue;
            }

            for (int j = 0; j < SamplerFunc::neighborWidth; ++j)
            {
                // Get current j value
                // +1 for the same reason
                const int jCurrent = gridX + 1 + j - _halfWidth;

                // handle out of range
                if (jCurrent < 0 || jCurrent >= imWidth)
                {
                    continue;
                }

                // sample input image and weight according to sampler
                const double w = coefsX[j] * coefsY[i];
                const typename RealPixel<T>::real_type pix = RealPixel<T>::convert_to_real(src(iCurrent, jCurrent));
                const typename RealPixel<T>::real_type wp = pix * w;
                res += wp;

                totalWeight += w;
            }
        }

        // If value too small, it should be so instable, so return the sampled value
        if (totalWeight <= 0.2)
        {
            int row = floor(y);
            int col = floor(x);

            if (row < 0)
                row = 0;
            if (col < 0)
                col = 0;
            if (row >= imHeight)
                row = imHeight - 1;
            if (col >= imWidth)
                col = imWidth - 1;

            return src(row, col);
        }

        if (totalWeight != 1.0)
        {
            res /= totalWeight;
        }

        return RealPixel<T>::convert_from_real(res);
    }

  private:
    SamplerFunc _sampler;
    const int _halfWidth;
};

}  // namespace image
}  // namespace aliceVision
