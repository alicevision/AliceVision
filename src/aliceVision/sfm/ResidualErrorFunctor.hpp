// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/camera/camera.hpp>

#include <ceres/rotation.h>

// Define ceres Cost_functor for each AliceVision camera model

namespace aliceVision {
namespace sfm {

/**
 * @brief Ceres functor to use a Pinhole (pinhole camera model K[R[t]) and a 3D point.
 *
 *  Data parameter blocks are the following <2,3,6,3>
 *  - 2 => dimension of the residuals,
 *  - 3 => the intrinsic data block [focal, principal point x, principal point y],
 *  - 6 => the camera extrinsic data block (camera orientation and position) [R;t],
 *         - rotation(angle axis), and translation [rX,rY,rZ,tx,ty,tz].
 *  - 3 => a 3D point data block.
 *
 */
struct ResidualErrorFunctor_Pinhole
{
  ResidualErrorFunctor_Pinhole(const double* const pos_2dpoint)
  {
    m_pos_2dpoint[0] = pos_2dpoint[0];
    m_pos_2dpoint[1] = pos_2dpoint[1];
  }

  // Enum to map intrinsics parameters between aliceVision & ceres camera data parameter block.
  enum {
    OFFSET_FOCAL_LENGTH = 0,
    OFFSET_PRINCIPAL_POINT_X = 1,
    OFFSET_PRINCIPAL_POINT_Y = 2
  };

  template <typename T>
  void applyIntrinsicParameters(const T* const cam_K,
                                const T x_u,
                                const T y_u,
                                T* out_residuals) const
  {
    const T& focal = cam_K[OFFSET_FOCAL_LENGTH];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X];
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y];

    // Apply focal length and principal point to get the final image coordinates
    const T projected_x = principal_point_x + focal * x_u;
    const T projected_y = principal_point_y + focal * y_u;

    // Compute and return the error is the difference between the predicted
    //  and observed position
    out_residuals[0] = projected_x - T(m_pos_2dpoint[0]);
    out_residuals[1] = projected_y - T(m_pos_2dpoint[1]);
  }

  template <typename T>
  bool operator()(
    const T* const cam_K,
    const T* const cam_Rt,
    const T* const subpose_Rt,
    const T* const pos_3dpoint,
    T* out_residuals) const
  {
    //--
    // Apply external parameters (Pose)
    //--

    T pos_proj[3];

    {
      const T* cam_R = cam_Rt;
      const T* cam_t = &cam_Rt[3];

      // Rotate the point according the camera rotation
      ceres::AngleAxisRotatePoint(cam_R, pos_3dpoint, pos_proj);

      // Apply the camera translation
      pos_proj[0] += cam_t[0];
      pos_proj[1] += cam_t[1];
      pos_proj[2] += cam_t[2];
    }

    {
      const T* cam_R = subpose_Rt;
      const T* cam_t = &subpose_Rt[3];

      // Rotate the point according to the camera rotation
      ceres::AngleAxisRotatePoint(cam_R, pos_proj, pos_proj);

      // Apply the camera translation
      pos_proj[0] += cam_t[0];
      pos_proj[1] += cam_t[1];
      pos_proj[2] += cam_t[2];
    }

    // Transform the point from homogeneous to euclidean (undistorted point)
    const T x_u = pos_proj[0] / pos_proj[2];
    const T y_u = pos_proj[1] / pos_proj[2];

    //--
    // Apply intrinsic parameters
    //--

    applyIntrinsicParameters(cam_K, x_u, y_u, out_residuals);

    return true;
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
    const T* const cam_Rt,
    const T* const pos_3dpoint,
    T* out_residuals) const
  {
    //--
    // Apply external parameters (Pose)
    //--

    const T * cam_R = cam_Rt;
    const T * cam_t = &cam_Rt[3];

    //T absCam_R[3];
    //T absCam_t[3];

    T pos_proj[3];
    // Rotate the point according the camera rotation
    ceres::AngleAxisRotatePoint(cam_R, pos_3dpoint, pos_proj);

    // Apply the camera translation
    pos_proj[0] += cam_t[0];
    pos_proj[1] += cam_t[1];
    pos_proj[2] += cam_t[2];

    // Transform the point from homogeneous to euclidean (undistorted point)
    const T x_u = pos_proj[0] / pos_proj[2];
    const T y_u = pos_proj[1] / pos_proj[2];

    //--
    // Apply intrinsic parameters
    //--

    applyIntrinsicParameters(cam_K, x_u, y_u, out_residuals);

    return true;
  }

  double m_pos_2dpoint[2]; // The 2D observation
};

/**
 * @brief Ceres functor to use a PinholeRadialK1
 *
 *  Data parameter blocks are the following <2,4,6,3>
 *  - 2 => dimension of the residuals,
 *  - 4 => the intrinsic data block [focal, principal point x, principal point y, K1],
 *  - 6 => the camera extrinsic data block (camera orientation and position) [R;t],
 *         - rotation(angle axis), and translation [rX,rY,rZ,tx,ty,tz].
 *  - 3 => a 3D point data block.
 *
 */
struct ResidualErrorFunctor_PinholeRadialK1
{
  ResidualErrorFunctor_PinholeRadialK1(const double* const pos_2dpoint)
  {
    m_pos_2dpoint[0] = pos_2dpoint[0];
    m_pos_2dpoint[1] = pos_2dpoint[1];
  }

  // Enum to map intrinsics parameters between aliceVision & ceres camera data parameter block.
  enum {
    OFFSET_FOCAL_LENGTH = 0,
    OFFSET_PRINCIPAL_POINT_X = 1,
    OFFSET_PRINCIPAL_POINT_Y = 2,
    OFFSET_DISTO_K1 = 3
  };


  template <typename T>
  void applyIntrinsicParameters(const T* const cam_K,
                                const T x_u,
                                const T y_u,
                                T* out_residuals) const
  {
    const T& focal = cam_K[OFFSET_FOCAL_LENGTH];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X];
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y];
    const T& k1 = cam_K[OFFSET_DISTO_K1];

    // Apply distortion (xd,yd) = disto(x_u,y_u)
    const T r2 = x_u*x_u + y_u*y_u;
    const T r_coeff = (T(1) + k1*r2);
    const T x_d = x_u * r_coeff;
    const T y_d = y_u * r_coeff;

    // Apply focal length and principal point to get the final image coordinates
    const T projected_x = principal_point_x + focal * x_d;
    const T projected_y = principal_point_y + focal * y_d;

    // Compute and return the error is the difference between the predicted
    //  and observed position
    out_residuals[0] = projected_x - T(m_pos_2dpoint[0]);
    out_residuals[1] = projected_y - T(m_pos_2dpoint[1]);
  }

  template <typename T>
  bool operator()(
    const T* const cam_K,
    const T* const cam_Rt,
    const T* const subpose_Rt,
    const T* const pos_3dpoint,
    T* out_residuals) const
  {
    //--
    // Apply external parameters (Pose)
    //--

    T pos_proj[3];

    {
      const T * cam_R = cam_Rt;
      const T * cam_t = &cam_Rt[3];

      // Rotate the point according the camera rotation
      ceres::AngleAxisRotatePoint(cam_R, pos_3dpoint, pos_proj);

      // Apply the camera translation
      pos_proj[0] += cam_t[0];
      pos_proj[1] += cam_t[1];
      pos_proj[2] += cam_t[2];
    }

    {
      const T* cam_R = subpose_Rt;
      const T* cam_t = &subpose_Rt[3];

      // Rotate the point according to the camera rotation
      ceres::AngleAxisRotatePoint(cam_R, pos_proj, pos_proj);

      // Apply the camera translation
      pos_proj[0] += cam_t[0];
      pos_proj[1] += cam_t[1];
      pos_proj[2] += cam_t[2];
    }

    // Transform the point from homogeneous to euclidean (undistorted point)
    const T x_u = pos_proj[0] / pos_proj[2];
    const T y_u = pos_proj[1] / pos_proj[2];

    //--
    // Apply intrinsic parameters
    //--

    applyIntrinsicParameters(cam_K, x_u, y_u, out_residuals);

    return true;
  }

  /**
   * @param[in] cam_K: Camera intrinsics( focal, principal point [x,y], K1 )
   * @param[in] cam_Rt: Camera parameterized using one block of 6 parameters [R;t]:
   *   - 3 for rotation(angle axis), 3 for translation
   * @param[in] pos_3dpoint
   * @param[out] out_residuals
   */
  template <typename T>
  bool operator()(
    const T* const cam_K,
    const T* const cam_Rt,
    const T* const pos_3dpoint,
    T* out_residuals) const
  {
    //--
    // Apply external parameters (Pose)
    //--

    const T * cam_R = cam_Rt;
    const T * cam_t = &cam_Rt[3];

    T pos_proj[3];
    // Rotate the point according the camera rotation
    ceres::AngleAxisRotatePoint(cam_R, pos_3dpoint, pos_proj);

    // Apply the camera translation
    pos_proj[0] += cam_t[0];
    pos_proj[1] += cam_t[1];
    pos_proj[2] += cam_t[2];

    // Transform the point from homogeneous to euclidean (undistorted point)
    const T x_u = pos_proj[0] / pos_proj[2];
    const T y_u = pos_proj[1] / pos_proj[2];

    //--
    // Apply intrinsic parameters
    //--

    applyIntrinsicParameters(cam_K, x_u, y_u, out_residuals);

    return true;
  }

  double m_pos_2dpoint[2]; // The 2D observation
};

/**
 * @brief Ceres functor to use a PinholeRadialK3
 *
 *  Data parameter blocks are the following <2,6,6,3>
 *  - 2 => dimension of the residuals,
 *  - 6 => the intrinsic data block [focal, principal point x, principal point y, K1, K2, K3],
 *  - 6 => the camera extrinsic data block (camera orientation and position) [R;t],
 *         - rotation(angle axis), and translation [rX,rY,rZ,tx,ty,tz].
 *  - 3 => a 3D point data block.
 *
 */
struct ResidualErrorFunctor_PinholeRadialK3
{
  ResidualErrorFunctor_PinholeRadialK3(const double* const pos_2dpoint)
  {
    m_pos_2dpoint[0] = pos_2dpoint[0];
    m_pos_2dpoint[1] = pos_2dpoint[1];
  }

  // Enum to map intrinsics parameters between aliceVision & ceres camera data parameter block.
  enum {
    OFFSET_FOCAL_LENGTH = 0,
    OFFSET_PRINCIPAL_POINT_X = 1,
    OFFSET_PRINCIPAL_POINT_Y = 2,
    OFFSET_DISTO_K1 = 3,
    OFFSET_DISTO_K2 = 4,
    OFFSET_DISTO_K3 = 5,
  };

  template <typename T>
  void applyIntrinsicParameters(const T* const cam_K,
                                const T x_u,
                                const T y_u,
                                T* out_residuals) const
  {
    const T& focal = cam_K[OFFSET_FOCAL_LENGTH];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X];
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y];
    const T& k1 = cam_K[OFFSET_DISTO_K1];
    const T& k2 = cam_K[OFFSET_DISTO_K2];
    const T& k3 = cam_K[OFFSET_DISTO_K3];

    // Apply distortion (xd,yd) = disto(x_u,y_u)
    const T r2 = x_u*x_u + y_u*y_u;
    const T r4 = r2 * r2;
    const T r6 = r4 * r2;
    const T r_coeff = (T(1) + k1*r2 + k2*r4 + k3*r6);
    const T x_d = x_u * r_coeff;
    const T y_d = y_u * r_coeff;

    // Apply focal length and principal point to get the final image coordinates
    const T projected_x = principal_point_x + focal * x_d;
    const T projected_y = principal_point_y + focal * y_d;

    // Compute and return the error is the difference between the predicted
    //  and observed position
    out_residuals[0] = projected_x - T(m_pos_2dpoint[0]);
    out_residuals[1] = projected_y - T(m_pos_2dpoint[1]);
  }

  template <typename T>
  bool operator()(
    const T* const cam_K,
    const T* const cam_Rt,
    const T* const subpose_Rt,
    const T* const pos_3dpoint,
    T* out_residuals) const
  {
    // Apply external parameters (RIG pose and RIG sub-pose)
    T pos_proj[3];

    // Apply RIG pose
    {
      const T * cam_R = cam_Rt;
      const T * cam_t = &cam_Rt[3];
      // Rotate the point according to the camera rotation
      ceres::AngleAxisRotatePoint(cam_R, pos_3dpoint, pos_proj); // TODO DELI cam_Rt <-> subposeRt

      // Apply the camera translation
      pos_proj[0] += cam_t[0];
      pos_proj[1] += cam_t[1];
      pos_proj[2] += cam_t[2];
    }

    // Apply RIG sub-pose
    {
      const T * cam_R = subpose_Rt;
      const T * cam_t = &subpose_Rt[3];
      // Rotate the point according to the camera rotation
      ceres::AngleAxisRotatePoint(cam_R, pos_proj, pos_proj);

      // Apply the camera translation
      pos_proj[0] += cam_t[0];
      pos_proj[1] += cam_t[1];
      pos_proj[2] += cam_t[2];
    }

    // Transform the point from homogeneous to euclidean (undistorted point)
    const T x_u = pos_proj[0] / pos_proj[2];
    const T y_u = pos_proj[1] / pos_proj[2];

    // Apply intrinsic parameters
    applyIntrinsicParameters(cam_K, x_u, y_u, out_residuals);

    return true;
  }

  /**
   * @param[in] cam_K: Camera intrinsics( focal, principal point [x,y], k1, k2, k3 )
   * @param[in] cam_Rt: Camera parameterized using one block of 6 parameters [R;t]:
   *   - 3 for rotation(angle axis), 3 for translation
   * @param[in] pos_3dpoint
   * @param[out] out_residuals
   */
  template <typename T>
  bool operator()(
    const T* const cam_K,
    const T* const cam_Rt,
    const T* const pos_3dpoint,
    T* out_residuals) const
  {
    //--
    // Apply external parameters (Pose)
    //--

    const T * cam_R = cam_Rt;
    const T * cam_t = &cam_Rt[3];

    T pos_proj[3];
    // Rotate the point according the camera rotation
    ceres::AngleAxisRotatePoint(cam_R, pos_3dpoint, pos_proj);

    // Apply the camera translation
    pos_proj[0] += cam_t[0];
    pos_proj[1] += cam_t[1];
    pos_proj[2] += cam_t[2];

    // Transform the point from homogeneous to euclidean (undistorted point)
    const T x_u = pos_proj[0] / pos_proj[2];
    const T y_u = pos_proj[1] / pos_proj[2];

    //--
    // Apply intrinsic parameters
    //--

    applyIntrinsicParameters(cam_K, x_u, y_u, out_residuals);

    return true;
  }

  double m_pos_2dpoint[2]; // The 2D observation
};

/**
 * @brief Ceres functor with constrained 3D points to use a PinholeBrownT2
 *
 *  Data parameter blocks are the following <2,8,6,3>
 *  - 2 => dimension of the residuals,
 *  - 8 => the intrinsic data block [focal, principal point x, principal point y, K1, K2, K3, T1, T2],
 *  - 6 => the camera extrinsic data block (camera orientation and position) [R;t],
 *         - rotation(angle axis), and translation [rX,rY,rZ,tx,ty,tz].
 *  - 3 => a 3D point data block.
 *
 */
struct ResidualErrorFunctor_PinholeBrownT2
{
  ResidualErrorFunctor_PinholeBrownT2(const double* const pos_2dpoint)
  {
    m_pos_2dpoint[0] = pos_2dpoint[0];
    m_pos_2dpoint[1] = pos_2dpoint[1];
  }

  // Enum to map intrinsics parameters between aliceVision & ceres camera data parameter block.
  enum {
    OFFSET_FOCAL_LENGTH = 0,
    OFFSET_PRINCIPAL_POINT_X = 1,
    OFFSET_PRINCIPAL_POINT_Y = 2,
    OFFSET_DISTO_K1 = 3,
    OFFSET_DISTO_K2 = 4,
    OFFSET_DISTO_K3 = 5,
    OFFSET_DISTO_T1 = 6,
    OFFSET_DISTO_T2 = 7,
  };

  template <typename T>
  void applyIntrinsicParameters(const T* const cam_K,
                                const T x_u,
                                const T y_u,
                                T* out_residuals) const
  {
    const T& focal = cam_K[OFFSET_FOCAL_LENGTH];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X];
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y];
    const T& k1 = cam_K[OFFSET_DISTO_K1];
    const T& k2 = cam_K[OFFSET_DISTO_K2];
    const T& k3 = cam_K[OFFSET_DISTO_K3];
    const T& t1 = cam_K[OFFSET_DISTO_T1];
    const T& t2 = cam_K[OFFSET_DISTO_T2];

    // Apply distortion (xd,yd) = disto(x_u,y_u)
    const T r2 = x_u*x_u + y_u*y_u;
    const T r4 = r2 * r2;
    const T r6 = r4 * r2;
    const T r_coeff = (T(1) + k1*r2 + k2*r4 + k3*r6);
    const T t_x = t2 * (r2 + T(2) * x_u*x_u) + T(2) * t1 * x_u * y_u;
    const T t_y = t1 * (r2 + T(2) * y_u*y_u) + T(2) * t2 * x_u * y_u;
    const T x_d = x_u * r_coeff + t_x;
    const T y_d = y_u * r_coeff + t_y;

    // Apply focal length and principal point to get the final image coordinates
    const T projected_x = principal_point_x + focal * x_d;
    const T projected_y = principal_point_y + focal * y_d;

    // Compute and return the error is the difference between the predicted
    //  and observed position
    out_residuals[0] = projected_x - T(m_pos_2dpoint[0]);
    out_residuals[1] = projected_y - T(m_pos_2dpoint[1]);
  }

  template <typename T>
  bool operator()(
    const T* const cam_K,
    const T* const cam_Rt,
    const T* const subpose_Rt,
    const T* const pos_3dpoint,
    T* out_residuals) const
  {
    //--
    // Apply external parameters (Pose)
    //--

    T pos_proj[3];

    {
      const T * cam_R = cam_Rt;
      const T * cam_t = &cam_Rt[3];


      // Rotate the point according the camera rotation
      ceres::AngleAxisRotatePoint(cam_R, pos_3dpoint, pos_proj);

      // Apply the camera translation
      pos_proj[0] += cam_t[0];
      pos_proj[1] += cam_t[1];
      pos_proj[2] += cam_t[2];
    }

    {
      const T * cam_R = subpose_Rt;
      const T * cam_t = &subpose_Rt[3];

      // Rotate the point according to the camera rotation
      ceres::AngleAxisRotatePoint(cam_R, pos_proj, pos_proj);

      // Apply the camera translation
      pos_proj[0] += cam_t[0];
      pos_proj[1] += cam_t[1];
      pos_proj[2] += cam_t[2];
    }

    // Transform the point from homogeneous to euclidean (undistorted point)
    const T x_u = pos_proj[0] / pos_proj[2];
    const T y_u = pos_proj[1] / pos_proj[2];

    //--
    // Apply intrinsic parameters
    //--

    applyIntrinsicParameters(cam_K, x_u, y_u, out_residuals);

    return true;
  }

  /**
   * @param[in] cam_K: Camera intrinsics( focal, principal point [x,y], k1, k2, k3, t1, t2 )
   * @param[in] cam_Rt: Camera parameterized using one block of 6 parameters [R;t]:
   *   - 3 for rotation(angle axis), 3 for translation
   * @param[in] pos_3dpoint
   * @param[out] out_residuals
   */
  template <typename T>
  bool operator()(
    const T* const cam_K,
    const T* const cam_Rt,
    const T* const pos_3dpoint,
    T* out_residuals) const
  {
    //--
    // Apply external parameters (Pose)
    //--

    const T * cam_R = cam_Rt;
    const T * cam_t = &cam_Rt[3];

    T pos_proj[3];
    // Rotate the point according the camera rotation
    ceres::AngleAxisRotatePoint(cam_R, pos_3dpoint, pos_proj);

    // Apply the camera translation
    pos_proj[0] += cam_t[0];
    pos_proj[1] += cam_t[1];
    pos_proj[2] += cam_t[2];

    // Transform the point from homogeneous to euclidean (undistorted point)
    const T x_u = pos_proj[0] / pos_proj[2];
    const T y_u = pos_proj[1] / pos_proj[2];

    //--
    // Apply intrinsic parameters
    //--

    applyIntrinsicParameters(cam_K, x_u, y_u, out_residuals);

    return true;
  }

  double m_pos_2dpoint[2]; // The 2D observation
};


/**
 * @brief Ceres functor with constrained 3D points to use a PinholeFisheye
 *
 *  Data parameter blocks are the following <2,8,6,3>
 *  - 2 => dimension of the residuals,
 *  - 7 => the intrinsic data block [focal, principal point x, principal point y, K1, K2, K3, K4],
 *  - 6 => the camera extrinsic data block (camera orientation and position) [R;t],
 *         - rotation(angle axis), and translation [rX,rY,rZ,tx,ty,tz].
 *  - 3 => a 3D point data block.
 *
 */
struct ResidualErrorFunctor_PinholeFisheye
{
  ResidualErrorFunctor_PinholeFisheye(const double* const pos_2dpoint)
  {
    m_pos_2dpoint[0] = pos_2dpoint[0];
    m_pos_2dpoint[1] = pos_2dpoint[1];
  }

  // Enum to map intrinsics parameters between aliceVision & ceres camera data parameter block.
  enum {
    OFFSET_FOCAL_LENGTH = 0,
    OFFSET_PRINCIPAL_POINT_X = 1,
    OFFSET_PRINCIPAL_POINT_Y = 2,
    OFFSET_DISTO_K1 = 3,
    OFFSET_DISTO_K2 = 4,
    OFFSET_DISTO_K3 = 5,
    OFFSET_DISTO_K4 = 6,
  };

  template <typename T>
  void applyIntrinsicParameters(const T* const cam_K,
                                const T x_u,
                                const T y_u,
                                T* out_residuals) const
  {
    const T& focal = cam_K[OFFSET_FOCAL_LENGTH];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X];
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y];
    const T& k1 = cam_K[OFFSET_DISTO_K1];
    const T& k2 = cam_K[OFFSET_DISTO_K2];
    const T& k3 = cam_K[OFFSET_DISTO_K3];
    const T& k4 = cam_K[OFFSET_DISTO_K4];

    // Apply distortion (xd,yd) = disto(x_u,y_u)
    const T r2 = x_u*x_u + y_u*y_u;
    const T r = sqrt(r2);
    const T theta = atan(r);
    const T theta2 = theta*theta, theta3 = theta2*theta, theta4 = theta2*theta2, theta5 = theta4*theta,
    theta6 = theta3*theta3, theta7 = theta6*theta, theta8 = theta4*theta4, theta9 = theta8*theta;
    const T theta_dist = theta + k1*theta3 + k2*theta5 + k3*theta7 + k4*theta9;
    const T inv_r = r > T(1e-8) ? T(1.0)/r : T(1.0);
    const T cdist = r > T(1e-8) ? theta_dist * inv_r : T(1);

    const T x_d = x_u * cdist;
    const T y_d = y_u * cdist;

    // Apply focal length and principal point to get the final image coordinates
    const T projected_x = principal_point_x + focal * x_d;
    const T projected_y = principal_point_y + focal * y_d;

    // Compute and return the error is the difference between the predicted
    //  and observed position
    out_residuals[0] = projected_x - T(m_pos_2dpoint[0]);
    out_residuals[1] = projected_y - T(m_pos_2dpoint[1]);

  }

  template <typename T>
  bool operator()(
    const T* const cam_K,
    const T* const cam_Rt,
    const T* const subpose_Rt,
    const T* const pos_3dpoint,
    T* out_residuals) const
  {
    //--
    // Apply external parameters (Pose)
    //--

    T pos_proj[3];

    {
      const T * cam_R = cam_Rt;
      const T * cam_t = &cam_Rt[3];

      // Rotate the point according the camera rotation
      ceres::AngleAxisRotatePoint(cam_R, pos_3dpoint, pos_proj);

      // Apply the camera translation
      pos_proj[0] += cam_t[0];
      pos_proj[1] += cam_t[1];
      pos_proj[2] += cam_t[2];
    }

    {
      const T * cam_R = subpose_Rt;
      const T * cam_t = &subpose_Rt[3];

      // Rotate the point according to the camera rotation
      ceres::AngleAxisRotatePoint(cam_R, pos_proj, pos_proj);

      // Apply the camera translation
      pos_proj[0] += cam_t[0];
      pos_proj[1] += cam_t[1];
      pos_proj[2] += cam_t[2];
    }

    // Transform the point from homogeneous to euclidean (undistorted point)
    const T x_u = pos_proj[0] / pos_proj[2];
    const T y_u = pos_proj[1] / pos_proj[2];

    //--
    // Apply intrinsic parameters
    //--

    applyIntrinsicParameters(cam_K, x_u, y_u, out_residuals);

    return true;
  }

  /**
   * @param[in] cam_K: Camera intrinsics( focal, principal point [x,y], k1, k2, k3, k4 )
   * @param[in] cam_Rt: Camera parameterized using one block of 6 parameters [R;t]:
   *   - 3 for rotation(angle axis), 3 for translation
   * @param[in] pos_3dpoint
   * @param[out] out_residuals
   */
  template <typename T>
  bool operator()(
    const T* const cam_K,
    const T* const cam_Rt,
    const T* const pos_3dpoint,
    T* out_residuals) const
  {
    //--
    // Apply external parameters (Pose)
    //--

    const T * cam_R = cam_Rt;
    const T * cam_t = &cam_Rt[3];

    T pos_proj[3];
    // Rotate the point according the camera rotation
    ceres::AngleAxisRotatePoint(cam_R, pos_3dpoint, pos_proj);

    // Apply the camera translation
    pos_proj[0] += cam_t[0];
    pos_proj[1] += cam_t[1];
    pos_proj[2] += cam_t[2];

    // Transform the point from homogeneous to euclidean (undistorted point)
    const T x_u = pos_proj[0] / pos_proj[2];
    const T y_u = pos_proj[1] / pos_proj[2];

    //--
    // Apply intrinsic parameters
    //--

    applyIntrinsicParameters(cam_K, x_u, y_u, out_residuals);

    return true;
  }

  double m_pos_2dpoint[2]; // The 2D observation
};

/**
 * @brief Ceres functor to use a ResidualErrorFunctor_PinholeFisheye1
 *
 *  Data parameter blocks are the following <2,4,6,3>
 *  - 2 => dimension of the residuals,
 *  - 4 => the intrinsic data block [focal, principal point x, principal point y, K1],
 *  - 6 => the camera extrinsic data block (camera orientation and position) [R;t],
 *         - rotation(angle axis), and translation [rX,rY,rZ,tx,ty,tz].
 *  - 3 => a 3D point data block.
 *
 */
struct ResidualErrorFunctor_PinholeFisheye1
{
  ResidualErrorFunctor_PinholeFisheye1(const double* const pos_2dpoint)
  {
    m_pos_2dpoint[0] = pos_2dpoint[0];
    m_pos_2dpoint[1] = pos_2dpoint[1];
  }

  // Enum to map intrinsics parameters between aliceVision & ceres camera data parameter block.
  enum {
    OFFSET_FOCAL_LENGTH = 0,
    OFFSET_PRINCIPAL_POINT_X = 1,
    OFFSET_PRINCIPAL_POINT_Y = 2,
    OFFSET_DISTO_K1 = 3
  };

  template <typename T>
  void applyIntrinsicParameters(const T* const cam_K,
                                const T x_u,
                                const T y_u,
                                T* out_residuals) const
  {
    const T& focal = cam_K[OFFSET_FOCAL_LENGTH];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X];
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y];
    const T& k1 = cam_K[OFFSET_DISTO_K1];

    // Apply distortion (xd,yd) = disto(x_u,y_u)
    const T r2 = x_u*x_u + y_u*y_u;
    const T r = sqrt(r2);
    const T r_coeff = (atan(2.0 * r * tan(0.5 * k1)) / k1) / r;
    const T x_d = x_u * r_coeff;
    const T y_d = y_u * r_coeff;

    // Apply focal length and principal point to get the final image coordinates
    const T projected_x = principal_point_x + focal * x_d;
    const T projected_y = principal_point_y + focal * y_d;

    // Compute and return the error is the difference between the predicted
    //  and observed position
    out_residuals[0] = projected_x - T(m_pos_2dpoint[0]);
    out_residuals[1] = projected_y - T(m_pos_2dpoint[1]);
  }

  template <typename T>
  bool operator()(
    const T* const cam_K,
    const T* const cam_Rt,
    const T* const subpose_Rt,
    const T* const pos_3dpoint,
    T* out_residuals) const
  {
    //--
    // Apply external parameters (Pose)
    //--

    T pos_proj[3];

    {
      const T * cam_R = cam_Rt;
      const T * cam_t = &cam_Rt[3];

      // Rotate the point according the camera rotation
      ceres::AngleAxisRotatePoint(cam_R, pos_3dpoint, pos_proj);

      // Apply the camera translation
      pos_proj[0] += cam_t[0];
      pos_proj[1] += cam_t[1];
      pos_proj[2] += cam_t[2];
    }

    {
      const T * cam_R = subpose_Rt;
      const T * cam_t = &subpose_Rt[3];

      // Rotate the point according to the camera rotation
      ceres::AngleAxisRotatePoint(cam_R, pos_proj, pos_proj);

      // Apply the camera translation
      pos_proj[0] += cam_t[0];
      pos_proj[1] += cam_t[1];
      pos_proj[2] += cam_t[2];
    }

    // Transform the point from homogeneous to euclidean (undistorted point)
    const T x_u = pos_proj[0] / pos_proj[2];
    const T y_u = pos_proj[1] / pos_proj[2];

    //--
    // Apply intrinsic parameters
    //--

    applyIntrinsicParameters(cam_K, x_u, y_u, out_residuals);

    return true;
  }

  /**
   * @param[in] cam_K: Camera intrinsics( focal, principal point [x,y], K1 )
   * @param[in] cam_Rt: Camera parameterized using one block of 6 parameters [R;t]:
   *   - 3 for rotation(angle axis), 3 for translation
   * @param[in] pos_3dpoint
   * @param[out] out_residuals
   */
  template <typename T>
  bool operator()(
    const T* const cam_K,
    const T* const cam_Rt,
    const T* const pos_3dpoint,
    T* out_residuals) const
  {
    //--
    // Apply external parameters (Pose)
    //--

    const T * cam_R = cam_Rt;
    const T * cam_t = &cam_Rt[3];

    T pos_proj[3];
    // Rotate the point according the camera rotation
    ceres::AngleAxisRotatePoint(cam_R, pos_3dpoint, pos_proj);

    // Apply the camera translation
    pos_proj[0] += cam_t[0];
    pos_proj[1] += cam_t[1];
    pos_proj[2] += cam_t[2];

    // Transform the point from homogeneous to euclidean (undistorted point)
    const T x_u = pos_proj[0] / pos_proj[2];
    const T y_u = pos_proj[1] / pos_proj[2];

    //--
    // Apply intrinsic parameters
    //--

    applyIntrinsicParameters(cam_K, x_u, y_u, out_residuals);

    return true;
  }

  double m_pos_2dpoint[2]; // The 2D observation
};


} // namespace sfm
} // namespace aliceVision
