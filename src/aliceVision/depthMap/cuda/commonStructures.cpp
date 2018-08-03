// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "commonStructures.hpp"

#include <aliceVision/depthMap/cuda/planeSweeping/plane_sweeping_cuda.hpp>

namespace aliceVision {
namespace depthMap {

cameraStruct::cameraStruct()
    : cam( nullptr ),
      tex_rgba_hmh( nullptr ),
      camId( -1 ),
      rc( -1 ),
      scale( -1 )
{ }

cameraStruct::cameraStruct( const cameraStruct& orig )
    : cam( orig.cam ),
      tex_rgba_hmh( orig.tex_rgba_hmh ),
      camId( orig.camId ),
      rc( orig.rc ),
      scale( orig.scale )
{ }

cameraStruct& cameraStruct::operator=( const cameraStruct& orig )
{
    cam = orig.cam;
    tex_rgba_hmh = orig.tex_rgba_hmh;
    camId = orig.camId;
    rc = orig.rc;
    scale = orig.scale;

    return *this;
}

cameraStruct::~cameraStruct()
{
}

void cameraStruct::makeTexRGBA( int w, int h )
{
    tex_rgba_hmh.reset( new CudaHostMemoryHeap<uchar4, 2>(CudaSize<2>( w, h ) ) );
}

void cameraStruct::setTexRGBA( int x, int y, const uchar4& val )
{
    tex_rgba_hmh->operator()(x,y) = val;
}

} // namespace depthMap
} // namespace aliceVision

