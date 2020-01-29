// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/image/all.hpp"
#include "aliceVision/feature/PointFeature.hpp"
#include <vector>

namespace aliceVision {

//-- A slow but accurate way to draw K-VLD lines
void getKVLDMask(
  image::Image< unsigned char > *maskL,
  image::Image< unsigned char > *maskR,
  const std::vector< feature::PointFeature > &vec_F1,
  const std::vector< feature::PointFeature > &vec_F2,
  const std::vector< Pair >& vec_matches,
  const std::vector< bool >& vec_valide,
  const aliceVision::Mat& mat_E)
{
  for( int it1 = 0; it1 < vec_matches.size() - 1; it1++ )
  {
    for( int it2 = it1 + 1; it2 < vec_matches.size(); it2++ )
    {
      if( vec_valide[ it1 ] && vec_valide[ it2 ] && mat_E( it1, it2 ) >= 0 )
      {
        const feature::PointFeature & l1 = vec_F1[ vec_matches[ it1 ].first ];
        const feature::PointFeature & l2 = vec_F1[ vec_matches[ it2 ].first ];
        float l = ( l1.coords() - l2.coords() ).norm();
        int widthL = std::max( 1.f, l / ( dimension + 1.f ) );

        image::DrawLineThickness(l1.x(), l1.y(), l2.x(), l2.y(), 255, widthL, maskL);

        const feature::PointFeature & r1 = vec_F2[ vec_matches[ it1 ].second ];
        const feature::PointFeature & r2 = vec_F2[ vec_matches[ it2 ].second ];
        float r = ( r1.coords() - r2.coords() ).norm();
        int widthR = std::max( 1.f, r / ( dimension + 1.f ) );

        image::DrawLineThickness(r1.x(), r1.y(), r2.x(), r2.y(), 255, widthR, maskR);
      }
    }
  }
}

}; // namespace aliceVision
