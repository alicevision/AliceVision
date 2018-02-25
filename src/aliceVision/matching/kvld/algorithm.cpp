// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2011-12 Zhe Liu and Pierre Moulon.
// This file was initially part of the KVLD library under the terms of the BSD license (see the COPYING file).
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/** @basic structures implementation
 ** @author Zhe Liu
 **/

#include "algorithm.h"

IntegralImages::IntegralImages(const aliceVision::image::Image< float >& I)
{
  map.resize( I.Width() + 1, I.Height() + 1 );
  map.fill( 0 );
  for( int y = 0; y < I.Height(); y++ )
  {
    for( int x = 0; x < I.Width(); x++ )
    {
      map( y + 1, x + 1 ) = double( I( y, x ) ) + map( y, x + 1 ) + map( y + 1, x ) - map( y, x );
    }
  }
}

float getRange(
  const aliceVision::image::Image< float >& I,
  int a,
  const float p )
{
  float range = sqrt( float( 3.f * I.Height() * I.Width() ) / ( p * a * PI_ ) );
  return range;
}


//=============================IO interface======================//

std::ofstream& writeDetector( std::ofstream& out, const aliceVision::feature::SIOPointFeature& feature )
{
  out << feature.x() << " "
    << feature.y() << " "
    << feature.scale() << " "
    << feature.orientation() << std::endl;
  return out;
}

std::ifstream& readDetector( std::ifstream& in, aliceVision::feature::SIOPointFeature& point )
{
  in >> point.x()
    >> point.y()
    >> point.scale()
    >> point.orientation();
  return in;
}
