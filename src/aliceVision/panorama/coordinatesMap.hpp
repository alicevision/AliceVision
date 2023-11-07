// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/image/all.hpp>
#include <aliceVision/camera/camera.hpp>

#include "boundingBox.hpp"

namespace aliceVision {

class CoordinatesMap
{
  public:
    /**
     * Build coordinates map given camera properties
     * @param panoramaSize desired output panoramaSize
     * @param pose the camera pose wrt an arbitrary reference frame
     * @param intrinsics the camera intrinsics
     */
    bool build(const std::pair<int, int>& panoramaSize,
               const geometry::Pose3& pose,
               const aliceVision::camera::IntrinsicBase& intrinsics,
               const BoundingBox& coarseBbox);

    bool computeScale(double& result, float ratioUpscale);

    size_t getOffsetX() const { return _offset_x; }

    size_t getOffsetY() const { return _offset_y; }

    BoundingBox getBoundingBox() const { return _boundingBox; }

    const aliceVision::image::Image<Eigen::Vector2f>& getCoordinates() const { return _coordinates; }

    const aliceVision::image::Image<unsigned char>& getMask() const { return _mask; }

  private:
    size_t _offset_x = 0;
    size_t _offset_y = 0;

    aliceVision::image::Image<Eigen::Vector2f> _coordinates;
    aliceVision::image::Image<unsigned char> _mask;
    BoundingBox _boundingBox;
};

}  // namespace aliceVision