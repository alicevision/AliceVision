#pragma once

#include <aliceVision/camera/camera.hpp>

#include <ceres/rotation.h>

namespace aliceVision {
namespace sfm {

/**
 * @brief Ceres functor to use a pair of pinhole on a pure rotation 2D constraint.
 *
 *  Data parameter blocks are the following <2,3,6,6>
 *  - 2 => dimension of the residuals,
 *  - 3 => the intrinsic data block for the first view [focal, principal point x, principal point y],
 *  - 3 => the camera extrinsic data block for the first view 
 *  - 3 => the camera extrinsic data block for the second view 
 *
 */
struct ResidualErrorConstraintFunctor_Pinhole
{
  ResidualErrorConstraintFunctor_Pinhole(const Vec3 & pos_2dpoint_first, const Vec3 & pos_2dpoint_second) 
  : m_pos_2dpoint_first(pos_2dpoint_first), m_pos_2dpoint_second(pos_2dpoint_second)
  {
  }

  // Enum to map intrinsics parameters between aliceVision & ceres camera data parameter block.
  enum {
    OFFSET_FOCAL_LENGTH = 0,
    OFFSET_PRINCIPAL_POINT_X = 1,
    OFFSET_PRINCIPAL_POINT_Y = 2
  };

  template <typename T>
  void lift(const T* const cam_K, const Vec3 pt, Eigen::Vector<T, 3> & out) const
  {
    const T& focal = cam_K[OFFSET_FOCAL_LENGTH];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X];
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y];


    out(0) = (pt(0) - principal_point_x) / focal;
    out(1) = (pt(1) - principal_point_y) / focal;
    out(2) = static_cast<T>(1.0);
  }

  template <typename T>
  void unlift(const T* const cam_K, const Eigen::Vector<T, 3> & pt, Eigen::Vector<T, 3> & out) const
  {
    const T& focal = cam_K[OFFSET_FOCAL_LENGTH];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X];
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y];

    Eigen::Vector<T, 3> proj_pt = pt / pt(2);

    out(0) = proj_pt(0) * focal + principal_point_x;
    out(1) = proj_pt(1) * focal + principal_point_y;
    out(2) = static_cast<T>(1.0);
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
    const T* const cam_K,
    const T* const cam_R1,
    const T* const cam_R2,
    T* out_residuals) const
  {
    Eigen::Matrix<T, 3, 3> oneRo, twoRo, twoRone;
    
    Eigen::Vector<T, 3> pt3d_1;

    lift(cam_K, m_pos_2dpoint_first, pt3d_1);

    ceres::AngleAxisToRotationMatrix(cam_R1, oneRo.data());
    ceres::AngleAxisToRotationMatrix(cam_R2, twoRo.data());

    twoRone = twoRo * oneRo.transpose();

    Eigen::Vector<T, 3> pt3d_2_est = twoRone * pt3d_1;

    Eigen::Vector<T, 3> pt2d_2_est;
    unlift(cam_K, pt3d_2_est, pt2d_2_est);

    Eigen::Vector<T, 3> residual = pt2d_2_est - m_pos_2dpoint_second;

    out_residuals[0] = residual(0);
    out_residuals[1] = residual(1);

    return true;
  }

  Vec3 m_pos_2dpoint_first; // The 2D observation in first view
  Vec3 m_pos_2dpoint_second; // The 2D observation in second view
};

/**
 * @brief Ceres functor to use a pair of pinhole on a pure rotation 2D constraint.
 *
 *  Data parameter blocks are the following <2,4,6,6>
 *  - 2 => dimension of the residuals,
 *  - 4 => the intrinsic data block for the first view [focal, principal point x, principal point y],
 *  - 3 => the camera extrinsic data block for the first view 
 *  - 3 => the camera extrinsic data block for the second view 
 *
 */
struct ResidualErrorConstraintFunctor_PinholeRadialK1
{
  ResidualErrorConstraintFunctor_PinholeRadialK1(const Vec3 & pos_2dpoint_first, const Vec3 & pos_2dpoint_second) 
  : m_pos_2dpoint_first(pos_2dpoint_first), m_pos_2dpoint_second(pos_2dpoint_second)
  {
  }

  // Enum to map intrinsics parameters between aliceVision & ceres camera data parameter block.
  enum {
    OFFSET_FOCAL_LENGTH = 0,
    OFFSET_PRINCIPAL_POINT_X = 1,
    OFFSET_PRINCIPAL_POINT_Y = 2,
    OFFSET_DISTO_K1 = 3
  };

  template <typename T>
  void lift(const T* const cam_K, const Vec3 pt, Eigen::Vector<T, 3> & out) const
  {
    /*const T& focal = cam_K[OFFSET_FOCAL_LENGTH];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X];
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y];
    const T& k1 = cam_K[OFFSET_DISTO_K1];

    // Apply distortion (xd,yd) = disto(x_u,y_u)
    const T r2 = x_u*x_u + y_u*y_u;
    const T r_coeff = (T(1) + k1*r2);
    const T x_d = x_u * r_coeff;
    const T y_d = y_u * r_coeff;

    out(0) = (pt(0) - principal_point_x) / focal;
    out(1) = (pt(1) - principal_point_y) / focal;
    out(2) = static_cast<T>(1.0);*/
  }

  template <typename T>
  void unlift(const T* const cam_K, const Eigen::Vector<T, 3> & pt, Eigen::Vector<T, 3> & out) const
  {
    const T& focal = cam_K[OFFSET_FOCAL_LENGTH];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X];
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y];

    Eigen::Vector<T, 3> proj_pt = pt / pt(2);

    out(0) = proj_pt(0) * focal + principal_point_x;
    out(1) = proj_pt(1) * focal + principal_point_y;
    out(2) = static_cast<T>(1.0);
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
    const T* const cam_K,
    const T* const cam_R1,
    const T* const cam_R2,
    T* out_residuals) const
  {
    Eigen::Matrix<T, 3, 3> oneRo, twoRo, twoRone;
    
    Eigen::Vector<T, 3> pt3d_1;

    lift(cam_K, m_pos_2dpoint_first, pt3d_1);

    ceres::AngleAxisToRotationMatrix(cam_R1, oneRo.data());
    ceres::AngleAxisToRotationMatrix(cam_R2, twoRo.data());

    twoRone = twoRo * oneRo.transpose();

    Eigen::Vector<T, 3> pt3d_2_est = twoRone * pt3d_1;

    Eigen::Vector<T, 3> pt2d_2_est;
    unlift(cam_K, pt3d_2_est, pt2d_2_est);

    Eigen::Vector<T, 3> residual = pt2d_2_est - m_pos_2dpoint_second;

    out_residuals[0] = residual(0);
    out_residuals[1] = residual(1);

    return true;
  }

  Vec3 m_pos_2dpoint_first; // The 2D observation in first view
  Vec3 m_pos_2dpoint_second; // The 2D observation in second view
};


} // namespace sfm
} // namespace aliceVision
