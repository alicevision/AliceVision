// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/geometry/lie.hpp>
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

        size_t size_disto = 1;
        auto isod = camera::IntrinsicScaleOffsetDisto::cast(intrinsics);
        if (isod)
        {
            _distortion = isod->getDistortion();
            if (_distortion)
            {
                size_disto = _distortion->getParameters().size();
            }
        }
        
        mutable_parameter_block_sizes()->push_back(intrinsics->getParametersSize());
        mutable_parameter_block_sizes()->push_back(size_disto);
        mutable_parameter_block_sizes()->push_back(3);
    }

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override
    {
        const double* parameter_intrinsics = parameters[0];
        const double* parameter_distortion = parameters[1];
        const double* parameter_point = parameters[2];

        const Eigen::Map<const Vec3> pt(parameter_point);
        const Vec4 pth = pt.homogeneous();

        const Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        //Here we assume the intrinsics object has been externally updated
        Vec2 pt_est = _intrinsics->project(pth, true);
        const double scale = (_measured.getScale() > 1e-12) ? _measured.getScale() : 1.0;

        auto test = std::dynamic_pointer_cast<camera::Pinhole>(_intrinsics);

        residuals[0] = (pt_est(0) - _measured.getX()) / scale;
        residuals[1] = (pt_est(1) - _measured.getY()) / scale;

        if (jacobians == nullptr)
        {
            return true;
        }

        size_t params_size = _intrinsics->getParametersSize();
        double d_res_d_pt_est = 1.0 / scale;

        if (jacobians[0] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobians[0], 2, params_size);

            J = d_res_d_pt_est * _intrinsics->getDerivativeTransformProjectWrtParams(T, pth);
        }
        
        if (jacobians[1] != nullptr && _distortion)
        {
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobians[1], 2, _distortion->getParameters().size());

            auto isod = camera::IntrinsicScaleOffsetDisto::cast(_intrinsics);
            J = d_res_d_pt_est * isod->getDerivativeTransformProjectWrtDistortion(T, pth);
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
    std::shared_ptr<camera::Distortion> _distortion;
};

}  // namespace sfm
}  // namespace aliceVision