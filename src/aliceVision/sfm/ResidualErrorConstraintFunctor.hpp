#pragma once

#include <aliceVision/camera/camera.hpp>

#include <ceres/rotation.h>

namespace aliceVision {
namespace sfm {

/**
 * @brief Ceres functor to use a pair of pinhole on a pure rotation 2D constraint.
 *
 *  Data parameter blocks are the following <2,3,3,6,6>
 *  - 2 => dimension of the residuals,
 *  - 3 => the intrinsic data block for the first view [focal, principal point x, principal point y],
 *  - 3 => the intrinsic data block for the second view [focal, principal point x, principal point y],
 *  - 3 => the camera extrinsic data block for the first view (camera orientation only as angle axis)
 *  - 3 => the camera extrinsic data block for the second view (camera orientation only as angle axis)
 *
 */
struct ResidualErrorConstraintFunctor_Pinhole
{
  ResidualErrorConstraintFunctor_Pinhole(const double* const pos_2dpoint_first, const double* const pos_2dpoint_second)
  {
    m_pos_2dpoint_first[0] = pos_2dpoint_first[0];
    m_pos_2dpoint_first[1] = pos_2dpoint_first[1];
    m_pos_2dpoint_second[0] = pos_2dpoint_second[0];
    m_pos_2dpoint_second[1] = pos_2dpoint_second[1];
  }

  /**
   * @param[in] cam_K: Camera intrinsics( focal, principal point [x,y] )
   * @param[in] cam_Rt: Camera parameterized using one block of 6 parameters [R;t]:
   *   - 3 for rotation(angle axis), 3 for translation
   * @param[in] pos_3dpoint
   * @param[out] out_residuals
   */
  template <typename T>
  bool operator()(
    const T* const cam_K1,
    const T* const cam_K2,
    const T* const cam_R1,
    const T* const cam_R2,
    T* out_residuals) const
  {
    //Eigen::Matrix<T, 3, 3, Eigen::RowMajor> R1;
    
    

    return true;
  }

  double m_pos_2dpoint_first[2]; // The 2D observation in first view
  double m_pos_2dpoint_second[2]; // The 2D observation in second view
};


} // namespace sfm
} // namespace aliceVision
