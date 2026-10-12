// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/geometry/lie.hpp>
#include <aliceVision/camera/Pinhole.hpp>
#include <Eigen/Core>
#include <ceres/ceres.h>
#pragma once

namespace aliceVision {
namespace sfm {

class CostIntrinsicsProject : public ceres::CostFunction
{
  public:
    CostIntrinsicsProject(const sfmData::Observation& measured, const std::shared_ptr<camera::IntrinsicBase>& intrinsics)
      : _measured(measured),
        _intrinsics(intrinsics)
    {
        set_num_residuals(2);

        size_t sizeDisto = 1;
        _isod = camera::IntrinsicScaleOffsetDisto::cast(intrinsics);
        if (_isod)
        {
            _distortion = _isod->getDistortion();
            if (_distortion)
            {
                sizeDisto = _distortion->getParameters().size();
            }

            // The undistortion is constant: the undistorted observation and the weight are computed once.
            const auto undistortion = _isod->getUndistortion();
            if (undistortion)
            {
                _undistortedSpace = true;
                _target = undistortion->undistort(measured.getCoordinates());

                // Where the undistortion folds (bad calibration, observation outside its valid domain),
                // its Jacobian is singular and the weight would explode: keep the undistorted pixel error.
                const Eigen::Matrix2d J = undistortion->getDerivativeUndistortWrtPoint(measured.getCoordinates());
                const double minDeterminant = 1e-3;
                if (J.allFinite() && J.determinant() > minDeterminant)
                {
                    _weight = J.inverse();
                }
            }
        }

        if (!_undistortedSpace)
        {
            _target = measured.getCoordinates();
        }

        mutable_parameter_block_sizes()->push_back(intrinsics->getParametersSize());
        mutable_parameter_block_sizes()->push_back(sizeDisto);
        mutable_parameter_block_sizes()->push_back(3);
    }

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override
    {
        const double* parameter_point = parameters[2];

        const Eigen::Map<const Vec3> pt(parameter_point);
        const Vec4 pth = pt.homogeneous();

        const Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        // Here we assume the intrinsics object has been externally updated
        const Vec2 pt_est = _undistortedSpace ? _isod->transformProjectDistortionOnly(T, pth) : _intrinsics->project(pth, true);
        const double scale = (_measured.getScale() > 1e-12) ? _measured.getScale() : 1.0;

        const Vec2 residual = _weight * (pt_est - _target) / scale;
        residuals[0] = residual(0);
        residuals[1] = residual(1);

        if (jacobians == nullptr)
        {
            return true;
        }

        const Eigen::Matrix2d d_res_d_pt_est = _weight / scale;
        const size_t params_size = _intrinsics->getParametersSize();

        if (jacobians[0] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobians[0], 2, params_size);

            J = d_res_d_pt_est * _intrinsics->getDerivativeTransformProjectWrtParams(T, pth);
        }

        if (jacobians[1] != nullptr && _distortion)
        {
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobians[1], 2, _distortion->getParameters().size());

            J = d_res_d_pt_est * _isod->getDerivativeTransformProjectWrtDistortion(T, pth);
        }

        if (jacobians[2] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J(jacobians[2]);

            J = d_res_d_pt_est * _intrinsics->getDerivativeTransformProjectWrtPoint3(T, pth);
        }

        return true;
    }

  private:
    const sfmData::Observation _measured;
    const std::shared_ptr<camera::IntrinsicBase> _intrinsics;
    std::shared_ptr<camera::IntrinsicScaleOffsetDisto> _isod;
    std::shared_ptr<camera::Distortion> _distortion;
    bool _undistortedSpace{false};
    /// Observation compared with the projection: undistorted for cameras with an Undistortion.
    Vec2 _target;
    /// Maps an undistorted pixel error to raw pixels (identity without Undistortion).
    Eigen::Matrix2d _weight{Eigen::Matrix2d::Identity()};
};

}  // namespace sfm
}  // namespace aliceVision