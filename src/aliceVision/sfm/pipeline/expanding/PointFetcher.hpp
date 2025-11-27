// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/camera/IntrinsicBase.hpp>

namespace aliceVision 
{
namespace sfm
{

class PointFetcher
{
public:
    using uptr = std::unique_ptr<PointFetcher>;
    using sptr = std::shared_ptr<PointFetcher>;

public:
    /**
     * Set the pose of the camera
     * @param pose the pose of the camera wrt some global coordinates frame
    */
    virtual void setPose(const geometry::Pose3 & pose) = 0;

    /**
     * @brief virtual method to get coordinates and normals of a pixel of an image
     * @param point result point in some global coordinates frame
     * @param normal result normal in some global coordinates frame
     * @param intrinsic the camera intrinsic object
     * @param imageCoords the input image pixel coordinates in 2D.
     * @return false on error 
    */
    virtual bool pickPointAndNormal(Vec3 & point, 
                                Vec3 & normal, 
                                const camera::IntrinsicBase & intrinsic, 
                                const Vec2 & imageCoords) = 0;
};

}
}