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
    

class CostRotationPrior : public ceres::SizedCostFunction<3, 9, 9> {
public:
  explicit CostRotationPrior(const Eigen::Matrix3d & two_R_one) : _two_R_one(two_R_one) {

  }

  bool Evaluate(double const * const * parameters, double * residuals, double ** jacobians) const override {

    const double * parameter_rotation_one = parameters[0];
    const double * parameter_rotation_two = parameters[1];

    const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> oneRo(parameter_rotation_one);
    const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> twoRo(parameter_rotation_two);

    Eigen::Matrix3d two_R_one_est = twoRo * oneRo.transpose();
    Eigen::Matrix3d error_R = two_R_one_est * _two_R_one.transpose();
    Eigen::Vector3d error_r = SO3::logm(error_R);

    residuals[0] = error_r(0);
    residuals[1] = error_r(1);
    residuals[2] = error_r(2);

    if (jacobians == nullptr) {
      return true;
    }

    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 3, 9, Eigen::RowMajor>> J(jacobians[0]);

      J = SO3::dlogmdr(error_R) * getJacobian_AB_wrt_A<3, 3, 3>(two_R_one_est, _two_R_one.transpose()) * getJacobian_AB_wrt_B<3, 3, 3>(twoRo, oneRo.transpose()) * getJacobian_At_wrt_A<3, 3>() * getJacobian_AB_wrt_A<3, 3, 3>(Eigen::Matrix3d::Identity(), oneRo);
    }

    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 3, 9, Eigen::RowMajor>> J(jacobians[1]);

      J = SO3::dlogmdr(error_R) * getJacobian_AB_wrt_A<3, 3, 3>(two_R_one_est, _two_R_one.transpose()) * getJacobian_AB_wrt_A<3, 3, 3>(twoRo, oneRo.transpose()) * getJacobian_AB_wrt_A<3, 3, 3>(Eigen::Matrix3d::Identity(), twoRo);
    }

    return true;
  }

private:
  Eigen::Matrix3d _two_R_one;
};

}
}