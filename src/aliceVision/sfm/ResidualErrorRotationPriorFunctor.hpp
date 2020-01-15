#pragma once

#include <aliceVision/camera/camera.hpp>
#include <ceres/rotation.h>



namespace aliceVision {
namespace sfm {

/**
 * @brief Ceres functor to use a pair of pinhole on a pure rotation 2D constraint.
 *
 *  Data parameter blocks are the following <3,6,6>
 *  - 3 => dimension of the residuals,
 *  - 6 => the camera extrinsic data block for the first view 
 *  - 6 => the camera extrinsic data block for the second view 
 */
struct ResidualErrorRotationPriorFunctor
{
  explicit ResidualErrorRotationPriorFunctor(const Eigen::Matrix3d & two_R_one) 
  : m_two_R_one(two_R_one)
  {
  }

  /**
   * @param[in] cam_Rt: Camera parameterized using one block of 6 parameters [R;t]:
   *   - 3 for rotation(angle axis), 3 for translation
   * @param[in] pos_3dpoint
   * @param[out] out_residuals
   */
  template <typename T>
  bool operator()(
    const T* const cam_R1,
    const T* const cam_R2,
    T* out_residuals) const
  {
    Eigen::Matrix<T, 3, 3> oneRo, twoRo, twoRone, R_error;
  
    ceres::AngleAxisToRotationMatrix(cam_R1, oneRo.data());
    ceres::AngleAxisToRotationMatrix(cam_R2, twoRo.data());
    twoRone = twoRo * oneRo.transpose();
    R_error = twoRone * m_two_R_one.transpose();

    ceres::RotationMatrixToAngleAxis(R_error.data(), out_residuals);

    out_residuals[0] *= 180.0 / M_PI;
    out_residuals[1] *= 180.0 / M_PI;
    out_residuals[2] *= 180.0 / M_PI;
    

    return true;
  }

  Eigen::Matrix3d m_two_R_one;
};

} // namespace sfm
} // namespace aliceVision
