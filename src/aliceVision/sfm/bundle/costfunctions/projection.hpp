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

class CostProjection : public ceres::CostFunction
{
  public:
    CostProjection(const sfmData::Observation& measured, const std::shared_ptr<camera::IntrinsicBase>& intrinsics, bool withRig)
      : _measured(measured),
        _intrinsics(intrinsics),
        _withRig(withRig)
    {
        set_num_residuals(2);

        mutable_parameter_block_sizes()->push_back(16);
        mutable_parameter_block_sizes()->push_back(16);
        mutable_parameter_block_sizes()->push_back(intrinsics->getParams().size());
        mutable_parameter_block_sizes()->push_back(3);
    }

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override
    {
        const double* parameter_pose = parameters[0];
        const double* parameter_rig = parameters[1];
        const double* parameter_intrinsics = parameters[2];
        const double* parameter_landmark = parameters[3];

        const Eigen::Map<const SE3::Matrix> rTo(parameter_pose);
        const Eigen::Map<const SE3::Matrix> cTr(parameter_rig);
        const Eigen::Map<const Vec3> pt(parameter_landmark);

        /*Update intrinsics object with estimated parameters*/
        size_t params_size = _intrinsics->getParams().size();
        std::vector<double> params;
        for (size_t param_id = 0; param_id < params_size; param_id++)
        {
            params.push_back(parameter_intrinsics[param_id]);
        }
        _intrinsics->updateFromParams(params);

        const SE3::Matrix T = cTr * rTo;
        const geometry::Pose3 T_pose3(T);

        const Vec4 pth = pt.homogeneous();

        const Vec2 pt_est = _intrinsics->project(T_pose3, pth, true);
        const double scale = (_measured.getScale() > 1e-12) ? _measured.getScale() : 1.0;

        residuals[0] = (pt_est(0) - _measured.getX()) / scale;
        residuals[1] = (pt_est(1) - _measured.getY()) / scale;

        if (jacobians == nullptr)
        {
            return true;
        }

        Eigen::Matrix2d d_res_d_pt_est = Eigen::Matrix2d::Identity() / scale;

        if (jacobians[0] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 2, 16, Eigen::RowMajor>> J(jacobians[0]);

            J = d_res_d_pt_est * _intrinsics->getDerivativeProjectWrtPose(T, pth) * getJacobian_AB_wrt_B<4, 4, 4>(cTr, rTo) *
                getJacobian_AB_wrt_A<4, 4, 4>(Eigen::Matrix4d::Identity(), rTo);
        }

        if (jacobians[1] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 2, 16, Eigen::RowMajor>> J(jacobians[1]);

            J = d_res_d_pt_est * _intrinsics->getDerivativeProjectWrtPose(T, pth) * getJacobian_AB_wrt_A<4, 4, 4>(cTr, rTo) *
                getJacobian_AB_wrt_A<4, 4, 4>(Eigen::Matrix4d::Identity(), cTr);
        }

        if (jacobians[2] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobians[2], 2, params_size);

            J = d_res_d_pt_est * _intrinsics->getDerivativeProjectWrtParams(T, pth);
        }

        if (jacobians[3] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J(jacobians[3]);

            J = d_res_d_pt_est * _intrinsics->getDerivativeProjectWrtPoint(T, pth) * Eigen::Matrix<double, 4, 3>::Identity();
        }

        return true;
    }

  private:
    const sfmData::Observation& _measured;
    const std::shared_ptr<camera::IntrinsicBase> _intrinsics;
    bool _withRig;
};

}  // namespace sfm
}  // namespace aliceVision