// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/geometry/lie.hpp>
#include <aliceVision/camera/IntrinsicScaleOffsetDisto.hpp>
#include <Eigen/Core>
#include <ceres/ceres.h>
#pragma once

namespace aliceVision {
namespace sfm {

/**
 * @brief Back-projection of an observation to a unit ray in the camera frame.
 *        Cameras with an Undistortion lift the undistorted pixel, computed once, in undistorted space.
 */
class CostIntrinsicsLift : public ceres::CostFunction
{
  public:
    CostIntrinsicsLift(const Vec2& pt2d, const std::shared_ptr<camera::IntrinsicBase>& intrinsics)
      : _intrinsics(intrinsics),
        _pt2d(pt2d)
    {
        set_num_residuals(3);

        size_t size_disto = 1;
        _isod = camera::IntrinsicScaleOffsetDisto::cast(intrinsics);
        if (_isod)
        {
            _distortion = _isod->getDistortion();
            if (_distortion)
            {
                size_disto = _distortion->getParameters().size();
            }

            // The undistortion is constant: the undistorted pixel is computed once.
            const auto undistortion = _isod->getUndistortion();
            if (undistortion)
            {
                _pt2d = undistortion->undistort(pt2d);
            }
        }

        mutable_parameter_block_sizes()->push_back(intrinsics->getParametersSize());
        mutable_parameter_block_sizes()->push_back(size_disto);
    }

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override
    {
        // Here we assume the intrinsics object has been externally updated
        const Vec3 pt_est = _isod ? _isod->backProjectUnitDistortionOnly(_pt2d) : _intrinsics->backProjectUnit(_pt2d);

        residuals[0] = pt_est(0);
        residuals[1] = pt_est(1);
        residuals[2] = pt_est(2);

        if (jacobians == nullptr)
        {
            return true;
        }

        if (jacobians[0] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobians[0], 3, _intrinsics->getParametersSize());

            J = _intrinsics->getDerivativeBackProjectUnitWrtParams(_pt2d);
        }

        if (jacobians[1] != nullptr && _distortion)
        {
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobians[1], 3, _distortion->getParameters().size());

            J = _isod->getDerivativeBackProjectUnitWrtDistortion(_pt2d);
        }

        return true;
    }

  private:
    const std::shared_ptr<camera::IntrinsicBase> _intrinsics;
    std::shared_ptr<camera::IntrinsicScaleOffsetDisto> _isod;
    std::shared_ptr<camera::Distortion> _distortion;
    /// Pixel lifted: undistorted for cameras with an Undistortion.
    Vec2 _pt2d;
};

}  // namespace sfm
}  // namespace aliceVision