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

class CostIntrinsicsLift : public ceres::CostFunction
{
  public:
    CostIntrinsicsLift(const Vec2 & pt2d, const std::shared_ptr<camera::IntrinsicBase>& intrinsics)
      : _pt2d(pt2d),
        _intrinsics(intrinsics)
    {
        set_num_residuals(3);

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
    }

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override
    {
        const double* parameter_intrinsics = parameters[0];
        const double* parameter_distortion = parameters[1];

        const Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        //Here we assume the intrinsics object has been externally updated
        Vec3 pt_est = _intrinsics->backProjectUnit(_pt2d);

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
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobians[1], 3,  _distortion->getParameters().size());

            auto isod = camera::IntrinsicScaleOffsetDisto::cast(_intrinsics);
            J = isod->getDerivativeBackProjectUnitWrtDistortion(_pt2d);
        }

        return true;
    }

  private:
    const std::shared_ptr<camera::IntrinsicBase> _intrinsics;
    std::shared_ptr<camera::Distortion> _distortion;
    const Vec2 _pt2d;
};

}  // namespace sfm
}  // namespace aliceVision