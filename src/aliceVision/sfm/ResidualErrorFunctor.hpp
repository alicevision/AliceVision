// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/camera/camera.hpp>
#include <aliceVision/sfmData/SfMData.hpp>

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
  explicit ResidualErrorFunctor_Pinhole(int w, int h, const sfmData::Observation& obs)
      : _center(double(w) * 0.5, double(h) * 0.5),  _obs(obs)
  {
  }

  // Enum to map intrinsics parameters between aliceVision & ceres camera data parameter block.
  enum {
    OFFSET_FOCAL_LENGTH_X = 0,
    OFFSET_FOCAL_LENGTH_Y = 1,
    OFFSET_PRINCIPAL_POINT_X = 2,
    OFFSET_PRINCIPAL_POINT_Y = 3
  };

  template <typename T>
  void applyIntrinsicParameters(const T* const cam_K,
                                const T x_u,
                                const T y_u,
                                T* out_residuals) const
  {
    const T& focalX = cam_K[OFFSET_FOCAL_LENGTH_X];
    const T& focalY = cam_K[OFFSET_FOCAL_LENGTH_Y];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X] + _center(0);
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y] + _center(1);

    // Apply focal length and principal point to get the final image coordinates
    const T projected_x = principal_point_x + focalX * x_u;
    const T projected_y = principal_point_y + focalY * y_u;

    // Compute and return the error is the difference between the predicted
    //  and observed position
    const T scale(_obs.scale > 0.0 ? _obs.scale : 1.0);
    out_residuals[0] = (projected_x - T(_obs.x[0])) / scale;
    out_residuals[1] = (projected_y - T(_obs.x[1])) / scale;
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
      T pos_proj_tmp[3] = { pos_proj[0], pos_proj[1], pos_proj[2] };
      ceres::AngleAxisRotatePoint(cam_R, pos_proj_tmp, pos_proj);

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

  const sfmData::Observation& _obs; // The 2D observation
  const Vec2 _center;
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
  explicit ResidualErrorFunctor_PinholeRadialK1(int w, int h, const sfmData::Observation& obs)
      : _center(double(w) * 0.5, double(h) * 0.5),  _obs(obs)
  {
  }

  // Enum to map intrinsics parameters between aliceVision & ceres camera data parameter block.
  enum {
    OFFSET_FOCAL_LENGTH_X = 0,
    OFFSET_FOCAL_LENGTH_Y = 1,
    OFFSET_PRINCIPAL_POINT_X = 2,
    OFFSET_PRINCIPAL_POINT_Y = 3,
    OFFSET_DISTO_K1 = 4
  };


  template <typename T>
  void applyIntrinsicParameters(const T* const cam_K,
                                const T x_u,
                                const T y_u,
                                T* out_residuals) const
  {
    const T& focalX = cam_K[OFFSET_FOCAL_LENGTH_X];
    const T& focalY = cam_K[OFFSET_FOCAL_LENGTH_Y];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X] + _center(0);
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y] + _center(1);
    const T& k1 = cam_K[OFFSET_DISTO_K1];

    // Apply distortion (xd,yd) = disto(x_u,y_u)
    const T r2 = x_u*x_u + y_u*y_u;
    const T r_coeff = (T(1) + k1*r2);
    const T x_d = x_u * r_coeff;
    const T y_d = y_u * r_coeff;

    // Apply focal length and principal point to get the final image coordinates
    const T projected_x = principal_point_x + focalX * x_d;
    const T projected_y = principal_point_y + focalY * y_d;

    // Compute and return the error is the difference between the predicted
    //  and observed position
    const T scale(_obs.scale > 0.0 ? _obs.scale : 1.0);
    out_residuals[0] = (projected_x - T(_obs.x[0])) / scale;
    out_residuals[1] = (projected_y - T(_obs.x[1])) / scale;
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
      T pos_proj_tmp[3] = { pos_proj[0], pos_proj[1], pos_proj[2] };
      ceres::AngleAxisRotatePoint(cam_R, pos_proj_tmp, pos_proj);

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

  const sfmData::Observation& _obs; // The 2D observation
  const Vec2 _center;
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
  explicit ResidualErrorFunctor_PinholeRadialK3(int w, int h, const sfmData::Observation& obs)
      : _center(double(w) * 0.5, double(h) * 0.5),  _obs(obs)
  {
  }

  // Enum to map intrinsics parameters between aliceVision & ceres camera data parameter block.
  enum {
    OFFSET_FOCAL_LENGTH_X = 0,
    OFFSET_FOCAL_LENGTH_Y = 1,
    OFFSET_PRINCIPAL_POINT_X = 2,
    OFFSET_PRINCIPAL_POINT_Y = 3,
    OFFSET_DISTO_K1 = 4,
    OFFSET_DISTO_K2 = 5,
    OFFSET_DISTO_K3 = 6,
  };

  template <typename T>
  void applyIntrinsicParameters(const T* const cam_K,
                                const T x_u,
                                const T y_u,
                                T* out_residuals) const
  {
    const T& focalX = cam_K[OFFSET_FOCAL_LENGTH_X];
    const T& focalY = cam_K[OFFSET_FOCAL_LENGTH_Y];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X] + _center(0);
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y] + _center(1);
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
    const T projected_x = principal_point_x + focalX * x_d;
    const T projected_y = principal_point_y + focalY * y_d;

    // Compute and return the error is the difference between the predicted
    //  and observed position
    const T scale(_obs.scale > 0.0 ? _obs.scale : 1.0);
    out_residuals[0] = (projected_x - T(_obs.x[0])) / scale;
    out_residuals[1] = (projected_y - T(_obs.x[1])) / scale;
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
      // Rotate the point according to the camera rotation. In-place rotation not supported by Ceres
      T pos_proj_tmp[3];
      ceres::AngleAxisRotatePoint(cam_R, pos_proj, pos_proj_tmp);

      // Apply the camera translation
      pos_proj[0] = pos_proj_tmp[0] + cam_t[0];
      pos_proj[1] = pos_proj_tmp[1] + cam_t[1];
      pos_proj[2] = pos_proj_tmp[2] + cam_t[2];
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

  const sfmData::Observation& _obs; // The 2D observation
  const Vec2 _center;
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
  explicit ResidualErrorFunctor_PinholeBrownT2(int w, int h, const sfmData::Observation& obs)
      : _center(double(w) * 0.5, double(h) * 0.5),  _obs(obs)
  {
  }

  // Enum to map intrinsics parameters between aliceVision & ceres camera data parameter block.
  enum {
    OFFSET_FOCAL_LENGTH_X = 0,
    OFFSET_FOCAL_LENGTH_Y = 1,
    OFFSET_PRINCIPAL_POINT_X = 2,
    OFFSET_PRINCIPAL_POINT_Y = 3,
    OFFSET_DISTO_K1 = 4,
    OFFSET_DISTO_K2 = 5,
    OFFSET_DISTO_K3 = 6,
    OFFSET_DISTO_T1 = 7,
    OFFSET_DISTO_T2 = 8,
  };

  template <typename T>
  void applyIntrinsicParameters(const T* const cam_K,
                                const T x_u,
                                const T y_u,
                                T* out_residuals) const
  {
    const T& focalX = cam_K[OFFSET_FOCAL_LENGTH_X];
    const T& focalY = cam_K[OFFSET_FOCAL_LENGTH_Y];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X] + _center(0);
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y] + _center(1);
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
    const T projected_x = principal_point_x + focalX * x_d;
    const T projected_y = principal_point_y + focalY * y_d;

    // Compute and return the error is the difference between the predicted
    //  and observed position
    const T scale(_obs.scale > 0.0 ? _obs.scale : 1.0);
    out_residuals[0] = (projected_x - T(_obs.x[0])) / scale;
    out_residuals[1] = (projected_y - T(_obs.x[1])) / scale;
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
      T pos_proj_tmp[3] = { pos_proj[0], pos_proj[1], pos_proj[2] };
      ceres::AngleAxisRotatePoint(cam_R, pos_proj_tmp, pos_proj);

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

  const sfmData::Observation& _obs; // The 2D observation
  const Vec2 _center;
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
  explicit ResidualErrorFunctor_PinholeFisheye(int w, int h, const sfmData::Observation& obs)
      : _center(double(w) * 0.5, double(h) * 0.5),  _obs(obs)
  {
  }

  // Enum to map intrinsics parameters between aliceVision & ceres camera data parameter block.
  enum {
    OFFSET_FOCAL_LENGTH_X = 0,
    OFFSET_FOCAL_LENGTH_Y = 1,
    OFFSET_PRINCIPAL_POINT_X = 2,
    OFFSET_PRINCIPAL_POINT_Y = 3,
    OFFSET_DISTO_K1 = 4,
    OFFSET_DISTO_K2 = 5,
    OFFSET_DISTO_K3 = 6,
    OFFSET_DISTO_K4 = 7,
  };

  template <typename T>
  void applyIntrinsicParameters(const T* const cam_K,
                                const T x_u,
                                const T y_u,
                                T* out_residuals) const
  {
    const T& focalX = cam_K[OFFSET_FOCAL_LENGTH_X];
    const T& focalY = cam_K[OFFSET_FOCAL_LENGTH_Y];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X] + _center(0);
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y] + _center(1);
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
    const T projected_x = principal_point_x + focalX * x_d;
    const T projected_y = principal_point_y + focalY * y_d;

    // Compute and return the error is the difference between the predicted
    //  and observed position
    const T scale(_obs.scale > 0.0 ? _obs.scale : 1.0);
    out_residuals[0] = (projected_x - T(_obs.x[0])) / scale;
    out_residuals[1] = (projected_y - T(_obs.x[1])) / scale;

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
      T pos_proj_tmp[3] = { pos_proj[0], pos_proj[1], pos_proj[2] };
      ceres::AngleAxisRotatePoint(cam_R, pos_proj_tmp, pos_proj);

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

  const sfmData::Observation& _obs; // The 2D observation
  const Vec2 _center;
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
  explicit ResidualErrorFunctor_PinholeFisheye1(int w, int h, const sfmData::Observation& obs)
      : _center(double(w) * 0.5, double(h) * 0.5),  _obs(obs)
  {
  }

  // Enum to map intrinsics parameters between aliceVision & ceres camera data parameter block.
  enum {
    OFFSET_FOCAL_LENGTH_X = 0,
    OFFSET_FOCAL_LENGTH_Y = 1,
    OFFSET_PRINCIPAL_POINT_X = 2,
    OFFSET_PRINCIPAL_POINT_Y = 3,
    OFFSET_DISTO_K1 = 4
  };

  template <typename T>
  void applyIntrinsicParameters(const T* const cam_K,
                                const T x_u,
                                const T y_u,
                                T* out_residuals) const
  {
    const T& focalX = cam_K[OFFSET_FOCAL_LENGTH_X];
    const T& focalY = cam_K[OFFSET_FOCAL_LENGTH_Y];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X] + _center(0);
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y] + _center(1);
    const T& k1 = cam_K[OFFSET_DISTO_K1];

    // Apply distortion (xd,yd) = disto(x_u,y_u)
    const T r2 = x_u*x_u + y_u*y_u;
    const T r = sqrt(r2);
    const T r_coeff = (atan(2.0 * r * tan(0.5 * k1)) / k1) / r;
    const T x_d = x_u * r_coeff;
    const T y_d = y_u * r_coeff;

    // Apply focal length and principal point to get the final image coordinates
    const T projected_x = principal_point_x + focalX * x_d;
    const T projected_y = principal_point_y + focalY * y_d;

    // Compute and return the error is the difference between the predicted
    //  and observed position
    const T scale(_obs.scale > 0.0 ? _obs.scale : 1.0);
    out_residuals[0] = (projected_x - T(_obs.x[0])) / scale;
    out_residuals[1] = (projected_y - T(_obs.x[1])) / scale;
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
      T pos_proj_tmp[3] = { pos_proj[0], pos_proj[1], pos_proj[2] };
      ceres::AngleAxisRotatePoint(cam_R, pos_proj_tmp, pos_proj);

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

  const sfmData::Observation& _obs; // The 2D observation
  const Vec2 _center;
};


/**
 * @brief Ceres functor to use a Pinhole3DEClassicLD
 *
 *  Data parameter blocks are the following <2,9,6,3>
 *  - 2 => dimension of the residuals,
 *  - 6 => the intrinsic data block [focal_x, focal_y, principal point x, principal point y, K1, K2, K3],
 *  - 6 => the camera extrinsic data block (camera orientation and position) [R;t],
 *         - rotation(angle axis), and translation [rX,rY,rZ,tx,ty,tz].
 *  - 3 => a 3D point data block.
 *
 */
struct ResidualErrorFunctor_Pinhole3DEClassicLD
{
  explicit ResidualErrorFunctor_Pinhole3DEClassicLD(int w, int h, const sfmData::Observation& obs)
      : _center(double(w) * 0.5, double(h) * 0.5),  _obs(obs)
  {
  }

  // Enum to map intrinsics parameters between aliceVision & ceres camera data parameter block.
  enum {
    OFFSET_FOCAL_LENGTH_X = 0,
    OFFSET_FOCAL_LENGTH_Y = 1,
    OFFSET_PRINCIPAL_POINT_X = 2,
    OFFSET_PRINCIPAL_POINT_Y = 3,
    OFFSET_DISTO_DELTA = 4,
    OFFSET_DISTO_IEPS = 5,
    OFFSET_DISTO_MUX = 6,
    OFFSET_DISTO_MUY = 7,
    OFFSET_DISTO_Q = 8,
  };

  template <typename T>
  void applyIntrinsicParameters(const T* const cam_K,
                                const T x_u,
                                const T y_u,
                                T* out_residuals) const
  {
    const T& focalX = cam_K[OFFSET_FOCAL_LENGTH_X];
    const T& focalY = cam_K[OFFSET_FOCAL_LENGTH_Y];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X] + _center(0);
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y] + _center(1);
 
    // Apply distortion (xd,yd) = disto(x_u,y_u)
    const T& delta = cam_K[OFFSET_DISTO_DELTA];
    const T& invepsilon = cam_K[OFFSET_DISTO_IEPS];
    const T& mux = cam_K[OFFSET_DISTO_MUX];
    const T& muy = cam_K[OFFSET_DISTO_MUY];
    const T& q = cam_K[OFFSET_DISTO_Q];

    const T eps = 1.0 + cos(invepsilon);

    const T cxx = delta * eps;
    const T cxy = (delta + mux) * eps;
    const T cxxx = q * eps;
    const T cxxy = 2.0 * q * eps;
    const T cxyy = q * eps;
    const T cyx = delta + muy;
    const T cyy = delta;
    const T cyxx = q;
    const T cyxy = 2.0 * q;
    const T cyyy = q;

    T x = x_u;
    T y = y_u;
    T xx = x * x;
    T yy = y * y;
    T xxxx = xx * xx;
    T yyyy = yy * yy;
    T xxyy = xx * yy;
    
    T x_d = x * (1.0 + cxx * xx + cxy * yy + cxxx * xxxx + cxxy * xxyy + cxyy * yyyy);
    T y_d = y * (1.0 + cyx * xx + cyy * yy + cyxx * xxxx + cyxy * xxyy + cyyy * yyyy);

    // Apply focal length and principal point to get the final image coordinates
    const T projected_x = principal_point_x + focalX * x_d;
    const T projected_y = principal_point_y + focalY * y_d;

    // Compute and return the error is the difference between the predicted
    //  and observed position
    const T scale(_obs.scale > 0.0 ? _obs.scale : 1.0);
    out_residuals[0] = (projected_x - T(_obs.x[0])) / scale;
    out_residuals[1] = (projected_y - T(_obs.x[1])) / scale;
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
      // Rotate the point according to the camera rotation. In-place rotation not supported by Ceres
      T pos_proj_tmp[3];
      ceres::AngleAxisRotatePoint(cam_R, pos_proj, pos_proj_tmp);

      // Apply the camera translation
      pos_proj[0] = pos_proj_tmp[0] + cam_t[0];
      pos_proj[1] = pos_proj_tmp[1] + cam_t[1];
      pos_proj[2] = pos_proj_tmp[2] + cam_t[2];
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

  const sfmData::Observation& _obs; // The 2D observation
  const Vec2 _center;
};

/**
 * @brief Ceres functor to use a ResidualErrorFunctor_Pinhole3DERadial4
 *
 *  Data parameter blocks are the following <2,10,6,3>
 *  - 2 => dimension of the residuals,
 *  - 6 => the intrinsic data block [focal_x, focal_y, principal point x, principal point y, K1, K2, K3],
 *  - 6 => the camera extrinsic data block (camera orientation and position) [R;t],
 *         - rotation(angle axis), and translation [rX,rY,rZ,tx,ty,tz].
 *  - 3 => a 3D point data block.
 *
 */
struct ResidualErrorFunctor_Pinhole3DERadial4
{
  explicit ResidualErrorFunctor_Pinhole3DERadial4(int w, int h, const sfmData::Observation& obs)
      : _center(double(w) * 0.5, double(h) * 0.5),  _obs(obs)
  {
  }

  // Enum to map intrinsics parameters between aliceVision & ceres camera data parameter block.
  enum {
    OFFSET_FOCAL_LENGTH_X = 0,
    OFFSET_FOCAL_LENGTH_Y = 1,
    OFFSET_PRINCIPAL_POINT_X = 2,
    OFFSET_PRINCIPAL_POINT_Y = 3,
    OFFSET_DISTO_C2 = 4,
    OFFSET_DISTO_C4 = 5,
    OFFSET_DISTO_U1 = 6,
    OFFSET_DISTO_V1 = 7,
    OFFSET_DISTO_U3 = 8,
    OFFSET_DISTO_V3 = 9,
  };

  template <typename T>
  void applyIntrinsicParameters(const T* const cam_K,
                                const T x_u,
                                const T y_u,
                                T* out_residuals) const
  {
    const T& focalX = cam_K[OFFSET_FOCAL_LENGTH_X];
    const T& focalY = cam_K[OFFSET_FOCAL_LENGTH_Y];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X] + _center(0);
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y] + _center(1);
 
    // Apply distortion (xd,yd) = disto(x_u,y_u)
    const T& c2 = cam_K[OFFSET_DISTO_C2];
    const T& c4 = cam_K[OFFSET_DISTO_C4];
    const T& u1 = cam_K[OFFSET_DISTO_U1];
    const T& v1 = cam_K[OFFSET_DISTO_V1];
    const T& u3 = cam_K[OFFSET_DISTO_U3];
    const T& v3 = cam_K[OFFSET_DISTO_V3];

    T x = x_u;
    T y = y_u;
    T xx = x * x;
    T yy = y * y;
    T xy = x * y;
    T r2 = xx + yy;
    T r4 = r2 * r2;

    T p1 = 1.0 + c2 * r2 + c4 * r4;
    T p2 = r2 + 2.0 * xx;
    T p3 = r2 + 2.0 * yy;
    T p4 = u1 + u3 * r2;
    T p5 = v1 + v3 * r2;
    T p6 = 2.0 * xy;

    T x_d = x * p1 + p2 * p4 + p6 * p5;
    T y_d = y * p1 + p3 * p5 + p6 * p4;

    // Apply focal length and principal point to get the final image coordinates
    const T projected_x = principal_point_x + focalX * x_d;
    const T projected_y = principal_point_y + focalY * y_d;

    // Compute and return the error is the difference between the predicted
    //  and observed position
    const T scale(_obs.scale > 0.0 ? _obs.scale : 1.0);
    out_residuals[0] = (projected_x - T(_obs.x[0])) / scale;
    out_residuals[1] = (projected_y - T(_obs.x[1])) / scale;
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
      // Rotate the point according to the camera rotation. In-place rotation not supported by Ceres
      T pos_proj_tmp[3];
      ceres::AngleAxisRotatePoint(cam_R, pos_proj, pos_proj_tmp);

      // Apply the camera translation
      pos_proj[0] = pos_proj_tmp[0] + cam_t[0];
      pos_proj[1] = pos_proj_tmp[1] + cam_t[1];
      pos_proj[2] = pos_proj_tmp[2] + cam_t[2];
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

  const sfmData::Observation& _obs; // The 2D observation
  const Vec2 _center;
};


} // namespace sfm
} // namespace aliceVision
