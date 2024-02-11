// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/geometry/lie.hpp>
#include <Eigen/Core>
#include <ceres/ceres.h>

namespace aliceVision {
namespace sfm {

class CostProjectionSimple : public ceres::CostFunction
{
  public:
    CostProjectionSimple(const sfmData::Observation& measured, const std::shared_ptr<camera::IntrinsicBase>& intrinsics)
      : _measured(measured),
        _intrinsics(intrinsics)
    {
        set_num_residuals(2);

        mutable_parameter_block_sizes()->push_back(16);
        mutable_parameter_block_sizes()->push_back(intrinsics->getParams().size());
        mutable_parameter_block_sizes()->push_back(3);
    }

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override
    {
        const double* parameter_pose = parameters[0];
        const double* parameter_intrinsics = parameters[1];
        const double* parameter_landmark = parameters[2];

        const Eigen::Map<const SE3::Matrix> T(parameter_pose);
        const Eigen::Map<const Vec3> pt(parameter_landmark);

        const Vec4 pth = pt.homogeneous();

        const Vec4 cpt = T * pth;

        Vec2 pt_est = _intrinsics->project(T, pth, true);
        const double scale = (_measured.getScale() > 1e-12) ? _measured.getScale() : 1.0;

        residuals[0] = (pt_est(0) - _measured.getX()) / scale;
        residuals[1] = (pt_est(1) - _measured.getY()) / scale;

        if (jacobians == nullptr)
        {
            return true;
        }

        const geometry::Pose3 T_pose3(T);
        size_t params_size = _intrinsics->getParams().size();

        double d_res_d_pt_est = 1.0 / scale;

        if (jacobians[0] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 2, 16, Eigen::RowMajor>> J(jacobians[0]);

            J = d_res_d_pt_est * _intrinsics->getDerivativeProjectWrtPoseLeft(T, pth);
        }

        if (jacobians[1] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobians[1], 2, params_size);

            J = d_res_d_pt_est * _intrinsics->getDerivativeProjectWrtParams(T, pth);
        }

        if (jacobians[2] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J(jacobians[2]);

            J = d_res_d_pt_est * _intrinsics->getDerivativeProjectWrtPoint3(T, pth);
        }

        return true;
    }

  private:
    const sfmData::Observation& _measured;
    const std::shared_ptr<camera::IntrinsicBase> _intrinsics;
};

}  // namespace sfm
}  // namespace aliceVision