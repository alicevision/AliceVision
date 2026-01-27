// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmData/PointFetcher.hpp>

#include <Eigen/Core>
#include <ceres/ceres.h>

namespace aliceVision {
namespace sfm {

class CostMeshIntersector : public ceres::CostFunction
{
  public:
    CostMeshIntersector(sfmData::PointFetcher::sptr fetcher)
    : _fetcher(fetcher)
    {
      set_num_residuals(3);

      mutable_parameter_block_sizes()->push_back(3);
      mutable_parameter_block_sizes()->push_back(3);
    }

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override
    {
      Eigen::Map<const Eigen::Vector3d> origin(parameters[0]); 
      Eigen::Map<const Eigen::Vector3d> direction(parameters[1]);

      Vec3 point, normal;
      if (!_fetcher->getPointAndNormal(point, normal, origin, direction))
      {
        return false;
      }

      residuals[0] = point[0];
      residuals[1] = point[1];
      residuals[2] = point[2];

      if (jacobians == nullptr)
      {
        return true;
      }

      // We assume the mesh is locally a plane of normal "normal" and distance to plane "d"
      double d = point.x() * normal.x() + point.y() * normal.y() + point.z() * normal.z();

      // If we know this plane parameters, and the ray ...
      // We can act as if we only had this plane to estimate the world 3D point
      // 
      // pt = origin + lambda * direction
      //
      // So we can compute lambda using :
      //
      // normal.transpose() * (origin + lambda * direction) = d
      // normal.transpose() * lambda * direction = d - normal.transpose() * origin
      // lambda = (d - normal.transpose() * origin) / (normal.transpose() * direction)

      double num = d - normal.dot(origin);
      double denum = normal.dot(direction);
      if (std::abs(denum) < 1e-12)
      {
        return false;
      }

      double lambda = num / denum;

      // d_lambda_d_num =  1 / denum;
      // d_lambda_d_denum = - num / (denum * denum)
      // d_num_d_origin = - normal.transpose()
      // d_denum_d_direction = normal.transpose()
      // d_point_d_origin = I
      // d_point_d_direction = lambda * I
      // d_point_d_lambda = direction

      // J_origin = d_point_d_origin + d_point_d_lambda * d_lambda_d_num * d_num_d_origin
      // J_origin = I + direction * 1 / denum * (- normal.transpose())
      // J_origin = I - (1/denum) * direction * normal.transpose()
      
      // J_direction = d_point_d_direction + d_point_d_lambda * d_lambda_d_denum * d_denum_d_direction
      // J_direction = lambda * I + direction * (- num / (denum * denum)) * normal.transpose()
      // J_direction = lambda * I - (num / (denum * denum)) * direction * normal.transpose()

      if (jacobians[0])
      {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J(jacobians[0]);
        J = Eigen::Matrix3d::Identity() - (1.0 / denum) * direction * normal.transpose();
      }

      if (jacobians[1])
      {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J(jacobians[1]);
        J = lambda * Eigen::Matrix3d::Identity() - (num / (denum * denum)) * direction * normal.transpose();
      }

      return true;
    }

  private:
    sfmData::PointFetcher::sptr _fetcher;
};

}  // namespace sfm
}  // namespace aliceVision