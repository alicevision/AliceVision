// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/image/Image.hpp"
#include "aliceVision/image/pixelTypes.hpp"

namespace aliceVision{
namespace image {

// The factor comes from http://www.easyrgb.com/
// RGB to XYZ : Y is the luminance channel
// var_R * 0.2126 + var_G * 0.7152 + var_B * 0.0722
inline float Rgb2GrayLinear(const float r, const float g, const float b)
{
  return r * 0.2126f + g * 0.7152f + b * 0.0722f;
}

inline float Rgb2Gray(const float r, const float g, const float b)
{
  return r * 0.299f + g * 0.587f + b * 0.114f;
}

template <typename Tin, typename Tout>
inline void Convert(const Tin& valin, Tout& out)
{
  out = static_cast<Tout>(valin);
}

template<>
inline void Convert<unsigned char, RGBColor>(const unsigned char& valin, RGBColor& valOut)
{
  valOut = RGBColor(valin);
}

template<>
inline void Convert<RGBColor, unsigned char>(const RGBColor& valin, unsigned char& valOut)
{
  valOut = static_cast<unsigned char>(Rgb2GrayLinear(valin.r(), valin.g(), valin.b()));
}

template<>
inline void Convert<unsigned char, RGBAColor>(const unsigned char& valin, RGBAColor& valOut)
{
  valOut = RGBAColor(valin, valin, valin, 255);
}

template<>
inline void Convert<RGBAColor, unsigned char>(const RGBAColor& valin, unsigned char& valOut)
{
  valOut = static_cast<unsigned char>((valin.a() / 255.f) * Rgb2GrayLinear(valin.r(), valin.g(), valin.b()));
}

template<>
inline void Convert<RGBAColor, RGBColor>(const RGBAColor& valin, RGBColor& valOut)
{
  valOut = RGBColor(static_cast<unsigned char>((valin.a() / 255.f) * valin.r()),
                    static_cast<unsigned char>((valin.a() / 255.f) * valin.g()),
                    static_cast<unsigned char>((valin.a() / 255.f) * valin.b()));
}

template<>
inline void Convert<RGBColor, RGBAColor>(const RGBColor& valin, RGBAColor& valOut)
{
  valOut = RGBAColor(valin.r(), valin.g(), valin.b(), static_cast<unsigned char>(255));
}

template<>
inline void Convert<float, RGBfColor>(const float& valin, RGBfColor& valOut)
{
  valOut = RGBfColor(valin);
}

template<>
inline void Convert<RGBfColor, float>(const RGBfColor& valin, float& valOut)
{
  valOut = Rgb2GrayLinear(valin.r(), valin.g(), valin.b());
}

template<>
inline void Convert<float, RGBAfColor>(const float& valin, RGBAfColor& valOut)
{
  valOut = RGBAfColor(valin);
}

template<>
inline void Convert<RGBAfColor, float>(const RGBAfColor& valin, float& valOut)
{
  valOut = Rgb2GrayLinear(valin.a() * valin.r(), valin.a() * valin.g(), valin.a() * valin.b());
}

template<>
inline void Convert<RGBAfColor, RGBfColor>(const RGBAfColor& valin, RGBfColor& valOut)
{
  valOut = RGBfColor(valin.a() * valin.r(), valin.a() * valin.g(), valin.a() * valin.b());
}

template<>
inline void Convert<RGBfColor, RGBAfColor>(const RGBfColor& valin, RGBAfColor& valOut)
{
  // alpha 1 by default
  valOut = RGBAfColor(valin.r(), valin.g(), valin.b());
}

template<typename ImageIn, typename ImageOut>
void ConvertPixelType(const ImageIn& imaIn, ImageOut *imaOut)
{
  (*imaOut) = ImageOut(imaIn.Width(), imaIn.Height());
  // Convert each input pixel to destination pixel
  for(int j = 0; j < imaIn.Height(); ++j)
    for(int i = 0; i < imaIn.Width(); ++i)
      Convert(imaIn(j,i), (*imaOut)(j,i));
}

//--------------------------------------------------------------------------
// RGB ( unsigned char or int ) to Float
//--------------------------------------------------------------------------

template< typename Tin, typename Tout >
inline void convertRGB2Float(
    const Tin& valIn,
    Tout& valOut,
    float factor = 1.0f / 255.f)
{
  for( int channel = 0; channel < 3; ++channel )
    valOut(channel) = (float)((int)(valIn(channel)) * factor);
}

template< typename ImageIn >
void rgb2Float( const ImageIn& imaIn,
                Image< RGBfColor > *imaOut, float factor = 1.0f / 255.f )
{
  assert( imaIn.Depth() == 3 );
  (*imaOut).resize(imaIn.Width(), imaIn.Height());
  // Convert each int RGB to float RGB values
  for( int j = 0; j < imaIn.Height(); ++j )
    for( int i = 0; i < imaIn.Width(); ++i )
      convertRGB2Float( imaIn( j, i ), ( *imaOut )( j, i ), factor );
}

//--------------------------------------------------------------------------
// Float to RGB ( unsigned char or int )
//--------------------------------------------------------------------------

inline void convertFloatToInt(
  const RGBfColor& valIn,
  RGBColor& valOut,
  float factor = 255.f)
{
  for( int channel = 0; channel < 3; ++channel )
    valOut(channel) = (int)(valIn(channel) * factor);
}

inline void rgbFloat2rgbInt(
        const Image< RGBfColor >& imaIn,
        Image< RGBColor > *imaOut,
        float factor = 255.f)
{
  assert( imaIn.Depth() == 3 );
  (*imaOut).resize(imaIn.Width(), imaIn.Height());
  // Convert each int RGB to float RGB values
  for( int j = 0; j < imaIn.Height(); ++j )
    for( int i = 0; i < imaIn.Width(); ++i )
      convertFloatToInt( imaIn( j, i ), (*imaOut)( j, i ), factor  );
}

} // namespace image
} // namespace aliceVision
