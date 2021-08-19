#pragma once

#include <aliceVision/camera/camera.hpp>
#include <aliceVision/camera/PinholeRadial.hpp>

#include <ceres/rotation.h>



namespace aliceVision {
namespace sfm {

template <typename T>
double getJetValue(const T & val) {
  return val.a;
}

template <>
double getJetValue(const double & val) {
  return val;
}

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
  ResidualErrorConstraintFunctor_Pinhole(int w, int h, const Vec3 & pos_2dpoint_first, const Vec3 & pos_2dpoint_second) 
  : m_center(double(w) * 0.5, double(h) * 0.5), m_pos_2dpoint_first(pos_2dpoint_first), m_pos_2dpoint_second(pos_2dpoint_second)
  {
  }

  // Enum to map intrinsics parameters between aliceVision & ceres camera data parameter block.
  enum {
    OFFSET_FOCAL_LENGTH = 0,
    OFFSET_PRINCIPAL_POINT_X = 1,
    OFFSET_PRINCIPAL_POINT_Y = 2
  };

  template <typename T>
  void lift(const T* const cam_K, const Vec3 pt, Eigen::Matrix< T, 3, 1> & out) const
  {
    const T& focal = cam_K[OFFSET_FOCAL_LENGTH];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X] + m_center(0);
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y] + m_center(1);


    out(0) = (pt(0) - principal_point_x) / focal;
    out(1) = (pt(1) - principal_point_y) / focal;
    out(2) = static_cast<T>(1.0);
  }

  template <typename T>
  void unlift(const T* const cam_K, const Eigen::Matrix< T, 3, 1> & pt, Eigen::Matrix< T, 3, 1> & out) const
  {
    const T& focal = cam_K[OFFSET_FOCAL_LENGTH];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X] + m_center(0);
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y] + m_center(1);

    Eigen::Matrix< T, 3, 1> proj_pt = pt / pt(2);

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
    
    Eigen::Matrix< T, 3, 1> pt3d_1;

    lift(cam_K, m_pos_2dpoint_first, pt3d_1);

    ceres::AngleAxisToRotationMatrix(cam_R1, oneRo.data());
    ceres::AngleAxisToRotationMatrix(cam_R2, twoRo.data());

    twoRone = twoRo * oneRo.transpose();

    Eigen::Matrix< T, 3, 1> pt3d_2_est = twoRone * pt3d_1;

    Eigen::Matrix< T, 3, 1> pt2d_2_est;
    unlift(cam_K, pt3d_2_est, pt2d_2_est);

    Eigen::Matrix< T, 3, 1> residual = pt2d_2_est - m_pos_2dpoint_second;

    out_residuals[0] = residual(0);
    out_residuals[1] = residual(1);

    return true;
  }

  Vec3 m_pos_2dpoint_first; // The 2D observation in first view
  Vec3 m_pos_2dpoint_second; // The 2D observation in second view
  Vec2 m_center; // Image center
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
  ResidualErrorConstraintFunctor_PinholeRadialK1(int w, int h, const Vec3 & pos_2dpoint_first, const Vec3 & pos_2dpoint_second) 
  : m_center(double(w) * 0.5, double(h) * 0.5), m_pos_2dpoint_first(pos_2dpoint_first), m_pos_2dpoint_second(pos_2dpoint_second)
  {
  }

  // Enum to map intrinsics parameters between aliceVision & ceres camera data parameter block.
  enum {
    OFFSET_FOCAL_LENGTH = 0,
    OFFSET_PRINCIPAL_POINT_X = 1,
    OFFSET_PRINCIPAL_POINT_Y = 2,
    OFFSET_DISTO_K1 = 3
  };

  static double distoFunctor(const std::vector<double> & params, double r2)
  {
    const double k1 = params[0];
    return r2 * Square(1.+r2*k1);
  }

  template <typename T>
  void lift(const T* const cam_K, const Vec3 pt, Eigen::Matrix< T, 3, 1> & out) const
  {
    const T& focal = cam_K[OFFSET_FOCAL_LENGTH];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X] + m_center(0);
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y] + m_center(1);
    const T& k1 = cam_K[OFFSET_DISTO_K1];

    //Unshift then unscale back to meters
    T xd = (pt(0) - principal_point_x) / focal;
    T yd = (pt(1) - principal_point_y) / focal;
    T distorted_radius = sqrt(xd*xd + yd*yd);

    /*A hack to obtain undistorted point even if using automatic diff*/
    double xd_real, yd_real, k1_real; 
    xd_real = getJetValue<T>(xd);
    yd_real = getJetValue<T>(yd);
    k1_real = getJetValue<T>(k1);

    /*get rescaler*/
    std::vector<double> distortionParams = {k1_real};
    double distorted_radius_square = xd_real * xd_real + yd_real * yd_real;
    double rescaler = ::sqrt(camera::radial_distortion::bisection_Radius_Solve(distortionParams, distorted_radius_square, distoFunctor));;
    
    if (distorted_radius < 1e-12) {
      out(0) = xd;
      out(1) = yd;
      out(2) = static_cast<T>(1.0);
    }
    else {
      out(0) = (xd / distorted_radius) * static_cast<T>(rescaler);
      out(1) = (yd / distorted_radius) * static_cast<T>(rescaler);    
      out(2) = static_cast<T>(1.0);
    }
  }

  template <typename T>
  void unlift(const T* const cam_K, const Eigen::Matrix< T, 3, 1> & pt, Eigen::Matrix< T, 3, 1> & out) const
  {
    const T& focal = cam_K[OFFSET_FOCAL_LENGTH];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X] + m_center(0);
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y] + m_center(1);
    const T& k1 = cam_K[OFFSET_DISTO_K1];

    //Project on plane
    Eigen::Matrix< T, 3, 1> proj_pt = pt / pt(2);

    //Apply distortion
    const T r2 = proj_pt(0)*proj_pt(0) + proj_pt(1)*proj_pt(1);
    const T r_coeff = (T(1) + k1*r2);
    const T x_d = proj_pt(0) * r_coeff;
    const T y_d = proj_pt(1) * r_coeff;
    
    //Scale and shift
    out(0) = x_d * focal + principal_point_x;
    out(1) = y_d * focal + principal_point_y;
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
    
    Eigen::Matrix<T, 3, 1> pt3d_1;

    //From pixel to meters
    lift(cam_K, m_pos_2dpoint_first, pt3d_1);

    //Build relative rotation
    ceres::AngleAxisToRotationMatrix(cam_R1, oneRo.data());
    ceres::AngleAxisToRotationMatrix(cam_R2, twoRo.data());
    twoRone = twoRo * oneRo.transpose();

    //Transform point
    Eigen::Matrix< T, 3, 1> pt3d_2_est = twoRone * pt3d_1;

    //Project back to image space in pixels
    Eigen::Matrix< T, 3, 1> pt2d_2_est;
    unlift(cam_K, pt3d_2_est, pt2d_2_est);

    //Compute residual
    Eigen::Matrix< T, 3, 1> residual = pt2d_2_est - m_pos_2dpoint_second;

    out_residuals[0] = residual(0);
    out_residuals[1] = residual(1);

    return true;
  }

  Vec3 m_pos_2dpoint_first; // The 2D observation in first view
  Vec3 m_pos_2dpoint_second; // The 2D observation in second view
  Vec2 m_center; // Image center
};

/**
 * @brief Ceres functor to use a pair of pinhole on a pure rotation 2D constraint.
 *
 *  Data parameter blocks are the following <2,6,6,6>
 *  - 2 => dimension of the residuals,
 *  - 4 => the intrinsic data block for the first view [focal, principal point x, principal point y, K1, K2, K3],
 *  - 3 => the camera extrinsic data block for the first view 
 *  - 3 => the camera extrinsic data block for the second view 
 *
 */
struct ResidualErrorConstraintFunctor_PinholeRadialK3
{
  ResidualErrorConstraintFunctor_PinholeRadialK3(int w, int h, const Vec3 & pos_2dpoint_first, const Vec3 & pos_2dpoint_second) 
  : m_center(double(w) * 0.5, double(h) * 0.5), m_pos_2dpoint_first(pos_2dpoint_first), m_pos_2dpoint_second(pos_2dpoint_second)
  {
  }

  // Enum to map intrinsics parameters between aliceVision & ceres camera data parameter block.
  enum {
    OFFSET_FOCAL_LENGTH = 0,
    OFFSET_PRINCIPAL_POINT_X = 1,
    OFFSET_PRINCIPAL_POINT_Y = 2,
    OFFSET_DISTO_K1 = 3,
    OFFSET_DISTO_K2 = 4,
    OFFSET_DISTO_K3 = 5
  };

  static double distoFunctor(const std::vector<double> & params, double r2)
  {
    const double k1 = params[0];
    const double k2 = params[1];
    const double k3 = params[2];
    return r2 * Square(1.+r2*(k1+r2*(k2+r2*k3)));
  }

  template <typename T>
  void lift(const T* const cam_K, const Vec3 pt, Eigen::Matrix< T, 3, 1> & out) const
  {
    const T& focal = cam_K[OFFSET_FOCAL_LENGTH];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X] + m_center(0);
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y] + m_center(1);
    const T& k1 = cam_K[OFFSET_DISTO_K1];
    const T& k2 = cam_K[OFFSET_DISTO_K2];
    const T& k3 = cam_K[OFFSET_DISTO_K3];

    //Unshift then unscale back to meters
    T xd = (pt(0) - principal_point_x) / focal;
    T yd = (pt(1) - principal_point_y) / focal;
    T distorted_radius = sqrt(xd*xd + yd*yd);

    /*A hack to obtain undistorted point even if using automatic diff*/
    double xd_real, yd_real, k1_real, k2_real, k3_real; 
    xd_real = getJetValue<T>(xd);
    yd_real = getJetValue<T>(yd);
    k1_real = getJetValue<T>(k1);
    k2_real = getJetValue<T>(k2);
    k3_real = getJetValue<T>(k3);

    /*get rescaler*/
    std::vector<double> distortionParams = {k1_real, k2_real, k3_real};
    double distorted_radius_square = xd_real * xd_real + yd_real * yd_real;
    double rescaler = ::sqrt(camera::radial_distortion::bisection_Radius_Solve(distortionParams, distorted_radius_square, distoFunctor));;
    
    if (distorted_radius < 1e-12) {
      out(0) = xd;
      out(1) = yd;
      out(2) = static_cast<T>(1.0);
    }
    else {
      out(0) = (xd / distorted_radius) * static_cast<T>(rescaler);
      out(1) = (yd / distorted_radius) * static_cast<T>(rescaler);    
      out(2) = static_cast<T>(1.0);
    }
  }

  template <typename T>
  void unlift(const T* const cam_K, const Eigen::Matrix< T, 3, 1> & pt, Eigen::Matrix< T, 3, 1> & out) const
  {
    const T& focal = cam_K[OFFSET_FOCAL_LENGTH];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X] + m_center(0);
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y] + m_center(1);
    const T& k1 = cam_K[OFFSET_DISTO_K1];
    const T& k2 = cam_K[OFFSET_DISTO_K2];
    const T& k3 = cam_K[OFFSET_DISTO_K3];

    //Project on plane
    Eigen::Matrix< T, 3, 1> proj_pt = pt / pt(2);

    //Apply distortion
    const T r2 = proj_pt(0)*proj_pt(0) + proj_pt(1)*proj_pt(1);
    const T r4 = r2 * r2;
    const T r6 = r4 * r2;
    const T r_coeff = (T(1) + k1*r2 + k2*r4 + k3*r6);
    const T x_d = proj_pt(0) * r_coeff;
    const T y_d = proj_pt(1) * r_coeff;
    
    //Scale and shift
    out(0) = x_d * focal + principal_point_x;
    out(1) = y_d * focal + principal_point_y;
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
    
    Eigen::Matrix<T, 3, 1> pt3d_1;

    //From pixel to meters
    lift(cam_K, m_pos_2dpoint_first, pt3d_1);

    //Build relative rotation
    ceres::AngleAxisToRotationMatrix(cam_R1, oneRo.data());
    ceres::AngleAxisToRotationMatrix(cam_R2, twoRo.data());
    twoRone = twoRo * oneRo.transpose();

    //Transform point
    Eigen::Matrix< T, 3, 1> pt3d_2_est = twoRone * pt3d_1;

    //Project back to image space in pixels
    Eigen::Matrix< T, 3, 1> pt2d_2_est;
    unlift(cam_K, pt3d_2_est, pt2d_2_est);

    //Compute residual
    Eigen::Matrix< T, 3, 1> residual = pt2d_2_est - m_pos_2dpoint_second;

    out_residuals[0] = residual(0);
    out_residuals[1] = residual(1);

    return true;
  }

  Vec3 m_pos_2dpoint_first; // The 2D observation in first view
  Vec3 m_pos_2dpoint_second; // The 2D observation in second view
  Vec2 m_center; // Image center
};


/**
 * @brief Ceres functor to use a pair of fisheye on a pure rotation 2D constraint.
 *
 *  Data parameter blocks are the following <2,7,6,6>
 *  - 2 => dimension of the residuals,
 *  - 4 => the intrinsic data block for the first view ,
 *  - 3 => the camera extrinsic data block for the first view 
 *  - 3 => the camera extrinsic data block for the second view 
 *
 */

struct ResidualErrorConstraintFunctor_PinholeFisheye
{
  ResidualErrorConstraintFunctor_PinholeFisheye(int w, int h, const Vec3 & pos_2dpoint_first, const Vec3 & pos_2dpoint_second) 
  : m_center(double(w) * 0.5, double(h) * 0.5), m_pos_2dpoint_first(pos_2dpoint_first), m_pos_2dpoint_second(pos_2dpoint_second)
  {
  }

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
  void lift(const T* const cam_K, const Vec3 pt, Eigen::Matrix< T, 3, 1> & out) const
  {
    const T& focal = cam_K[OFFSET_FOCAL_LENGTH];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X] + m_center(0);
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y] + m_center(1);
    const T& k1 = cam_K[OFFSET_DISTO_K1];
    const T& k2 = cam_K[OFFSET_DISTO_K2];
    const T& k3 = cam_K[OFFSET_DISTO_K3];
    const T& k4 = cam_K[OFFSET_DISTO_K4];

    //Unshift then unscale back to meters
    T xd = (pt(0) - principal_point_x) / focal;
    T yd = (pt(1) - principal_point_y) / focal;
    T distorted_radius = sqrt(xd*xd + yd*yd);
    
    /*A hack to obtain undistorted point even if using automatic diff*/
    double xd_real, yd_real, k1_real, k2_real, k3_real, k4_real; 
    xd_real = getJetValue<T>(xd);
    yd_real = getJetValue<T>(yd);
    k1_real = getJetValue<T>(k1);
    k2_real = getJetValue<T>(k2);
    k3_real = getJetValue<T>(k3);
    k4_real = getJetValue<T>(k4);

    double scale = 1.0;
    {
      const double eps = 1e-8;
      const double theta_dist = sqrt(xd_real * xd_real + yd_real * yd_real);
      if (theta_dist > eps)
      {
        double theta = theta_dist;
        for (int j = 0; j < 10; ++j)
        {
          const double theta2 = theta*theta;
          const double theta4 = theta2*theta2;
          const double theta6 = theta4*theta2;
          const double theta8 = theta6*theta2;
          
          double theta_fix = (theta * (1.0 + k1_real * theta2 + k2_real * theta4 + k3_real * theta6 + k4_real * theta8) - theta_dist) / (1.0 + 3.0 * k1_real * theta2 + 5.0 * k2_real * theta4 + 7.0 * k3_real * theta6 + 9.0 *k4_real * theta8);

          theta = theta - theta_fix;
        }
    
        scale = std::tan(theta);
      }
    }

    if (distorted_radius < 1e-12) {
      out(0) = xd;
      out(1) = yd;
      out(2) = static_cast<T>(1.0);
    }
    else {
      out(0) = (xd / distorted_radius) * static_cast<T>(scale);
      out(1) = (yd / distorted_radius) * static_cast<T>(scale);    
      out(2) = static_cast<T>(1.0);
    }
  }

  template <typename T>
  void unlift(const T* const cam_K, const Eigen::Matrix< T, 3, 1> & pt, Eigen::Matrix< T, 3, 1> & out) const
  {
    const T& focal = cam_K[OFFSET_FOCAL_LENGTH];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X] + m_center(0);
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y] + m_center(1);
    const T& k1 = cam_K[OFFSET_DISTO_K1];
    const T& k2 = cam_K[OFFSET_DISTO_K2];
    const T& k3 = cam_K[OFFSET_DISTO_K3];
    const T& k4 = cam_K[OFFSET_DISTO_K4];

    //Project on plane
    Eigen::Matrix< T, 3, 1> proj_pt = pt / pt(2);

    //Apply distortion
    const T x_u = proj_pt(0);
    const T y_u = proj_pt(1);
    const T dist = sqrt(x_u*x_u + y_u*y_u);
    const T theta = atan(dist);
    const T theta2 = theta*theta;
    const T theta3 = theta2*theta;
    const T theta4 = theta2*theta2;
    const T theta5 = theta4*theta;
    const T theta6 = theta3*theta3;
    const T theta7 = theta6*theta;
    const T theta8 = theta4*theta4;
    const T theta9 = theta8*theta;

    const T theta_dist = theta + k1*theta3 + k2*theta5 + k3*theta7 + k4*theta9;

    const T inv_r = dist > T(1e-8) ? T(1.0)/dist : T(1.0);
    const T cdist = dist > T(1e-8) ? theta_dist * inv_r : T(1);

    const T x_d = x_u * cdist;
    const T y_d = y_u * cdist;

    //Scale and shift
    out(0) = x_d * focal + principal_point_x;
    out(1) = y_d * focal + principal_point_y;
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
    
    Eigen::Matrix< T, 3, 1> pt3d_1;

    //From pixel to meters
    lift(cam_K, m_pos_2dpoint_first, pt3d_1);

    //Build relative rotation
    ceres::AngleAxisToRotationMatrix(cam_R1, oneRo.data());
    ceres::AngleAxisToRotationMatrix(cam_R2, twoRo.data());
    twoRone = twoRo * oneRo.transpose();

    //Transform point
    Eigen::Matrix< T, 3, 1> pt3d_2_est = twoRone * pt3d_1;

    //Project back to image space in pixels
    Eigen::Matrix< T, 3, 1> pt2d_2_est;
    unlift(cam_K, pt3d_2_est, pt2d_2_est);

    //Compute residual
    Eigen::Matrix< T, 3, 1> residual = pt2d_2_est - m_pos_2dpoint_second;

    out_residuals[0] = residual(0);
    out_residuals[1] = residual(1);

    return true;
  }

  Vec3 m_pos_2dpoint_first; // The 2D observation in first view
  Vec3 m_pos_2dpoint_second; // The 2D observation in second view
  Vec2 m_center; // Image center
};

/**
 * @brief Ceres functor to use a pair of equidistant on a pure rotation 2D constraint.
 *
 *  Data parameter blocks are the following <2,3,6,6>
 *  - 2 => dimension of the residuals,
 *  - 3 => the intrinsic data block for the first view [focal, principal point x, principal point y],
 *  - 3 => the camera extrinsic data block for the first view 
 *  - 3 => the camera extrinsic data block for the second view 
 *
 */
struct ResidualErrorConstraintFunctor_Equidistant
{
  ResidualErrorConstraintFunctor_Equidistant(int w, int h, const Vec3 & pos_2dpoint_first, const Vec3 & pos_2dpoint_second, double radius_size) 
  : m_center(double(w) * 0.5, double(h) * 0.5), m_pos_2dpoint_first(pos_2dpoint_first), m_pos_2dpoint_second(pos_2dpoint_second), m_radius_size(radius_size)
  {
  }

  // Enum to map intrinsics parameters between aliceVision & ceres camera data parameter block.
  enum {
    OFFSET_FOCAL_LENGTH = 0,
    OFFSET_PRINCIPAL_POINT_X = 1,
    OFFSET_PRINCIPAL_POINT_Y = 2
  };

  template <typename T>
  void lift(const T* const cam_K, const Vec3 pt, Eigen::Matrix< T, 3, 1> & out) const
  {
    const T& fov = cam_K[OFFSET_FOCAL_LENGTH];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X] + m_center(0);
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y] + m_center(1);


    Eigen::Matrix< T, 2, 1> camcoords;
    camcoords(0) = (pt(0) - principal_point_x) / (m_radius_size);
    camcoords(1) = (pt(1) - principal_point_y) / (m_radius_size);


    T angle_radial = atan2(camcoords(1), camcoords(0));
    T angle_Z = camcoords.norm() * 0.5 * fov;

    out(2) = cos(angle_Z);
    out(0) = cos(angle_radial) /** / 1.0 / **/ * sin(angle_Z);
    out(1) = sin(angle_radial) /** / 1.0 / **/ * sin(angle_Z);
  }

  template <typename T>
  void unlift(const T* const cam_K, const Eigen::Matrix< T, 3, 1> & pt, Eigen::Matrix< T, 3, 1> & out) const
  {
    const T& fov = cam_K[OFFSET_FOCAL_LENGTH];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X] + m_center(0);
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y] + m_center(1);

    /* To unit sphere */
    Eigen::Matrix< T, 3, 1> pt_normalized = pt;
    pt_normalized.normalize();

    /* Compute angle with optical center */
    T angle_Z = atan2(sqrt(pt_normalized(0) * pt_normalized(0) + pt_normalized(1) * pt_normalized(1)), pt_normalized(2));
    T radius = angle_Z / (0.5 * fov);

    /* Ignore depth component and compute radial angle */
    T angle_radial = atan2(pt_normalized(1), pt_normalized(0));

    /* radius = focal * angle_Z */
    Eigen::Matrix< T, 2, 1> proj_pt;
    proj_pt(0) = cos(angle_radial) * radius;
    proj_pt(1) = sin(angle_radial) * radius;

    out(0) = proj_pt(0) * m_radius_size + principal_point_x;
    out(1) = proj_pt(1) * m_radius_size + principal_point_y;
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
  bool operator()(const T* const cam_K, const T* const cam_R1, const T* const cam_R2, T* out_residuals) const
  {
    Eigen::Matrix<T, 3, 3> oneRo, twoRo, twoRone;
    
    Eigen::Matrix< T, 3, 1> pt3d_1;

    lift(cam_K, m_pos_2dpoint_first, pt3d_1);

    ceres::AngleAxisToRotationMatrix(cam_R1, oneRo.data());
    ceres::AngleAxisToRotationMatrix(cam_R2, twoRo.data());

    twoRone = twoRo * oneRo.transpose();

    Eigen::Matrix< T, 3, 1> pt3d_2_est = twoRone * pt3d_1;

    Eigen::Matrix< T, 3, 1> pt2d_2_est;
    unlift(cam_K, pt3d_2_est, pt2d_2_est);

    Eigen::Matrix< T, 3, 1> residual = pt2d_2_est - m_pos_2dpoint_second;

    out_residuals[0] = residual(0);
    out_residuals[1] = residual(1);

    return true;
  }

  Vec3 m_pos_2dpoint_first; // The 2D observation in first view
  Vec3 m_pos_2dpoint_second; // The 2D observation in second view
  Vec2 m_center; // Image center
  double m_radius_size;
};



/**
 * @brief Ceres functor to use a pair of pinhole on a pure rotation 2D constraint.
 *
 *  Data parameter blocks are the following <2,6,6,6>
 *  - 2 => dimension of the residuals,
 *  - 4 => the intrinsic data block for the first view [focal, principal point x, principal point y, K1, K2, K3],
 *  - 3 => the camera extrinsic data block for the first view 
 *  - 3 => the camera extrinsic data block for the second view 
 *
 */
struct ResidualErrorConstraintFunctor_EquidistantRadialK3
{
  ResidualErrorConstraintFunctor_EquidistantRadialK3(int w, int h, const Vec3 & pos_2dpoint_first, const Vec3 & pos_2dpoint_second, double radius_size) 
  : m_center(double(w) * 0.5, double(h) * 0.5), m_pos_2dpoint_first(pos_2dpoint_first), m_pos_2dpoint_second(pos_2dpoint_second), m_radius_size(radius_size)
  {
  }

  // Enum to map intrinsics parameters between aliceVision & ceres camera data parameter block.
  enum {
    OFFSET_FOCAL_LENGTH = 0,
    OFFSET_PRINCIPAL_POINT_X = 1,
    OFFSET_PRINCIPAL_POINT_Y = 2,
    OFFSET_DISTO_K1 = 3,
    OFFSET_DISTO_K2 = 4,
    OFFSET_DISTO_K3 = 5
  };

  static double distoFunctor(const std::vector<double> & params, double r2)
  {
    const double k1 = params[0];
    const double k2 = params[1];
    const double k3 = params[2];
    
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;

    return r2 * Square(1.0 + k1 * r2 + k2 * r4 + k3 * r6);
  }

  template <typename T>
  void lift(const T* const cam_K, const Vec3 pt, Eigen::Matrix< T, 3, 1> & out) const
  {
    const T& fov = cam_K[OFFSET_FOCAL_LENGTH];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X] + m_center(0);
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y] + m_center(1);
    const T& k1 = cam_K[OFFSET_DISTO_K1];
    const T& k2 = cam_K[OFFSET_DISTO_K2];
    const T& k3 = cam_K[OFFSET_DISTO_K3];

    //Unshift then unscale back to meters
    T xd = (pt(0) - principal_point_x) / (m_radius_size);
    T yd = (pt(1) - principal_point_y) / (m_radius_size);
    T distorted_radius = sqrt(xd*xd + yd*yd);

    /*A hack to obtain undistorted point even if using automatic diff*/
    double xd_real, yd_real, k1_real, k2_real, k3_real; 
    xd_real = getJetValue<T>(xd);
    yd_real = getJetValue<T>(yd);
    k1_real = getJetValue<T>(k1);
    k2_real = getJetValue<T>(k2);
    k3_real = getJetValue<T>(k3);

    /*get rescaler*/
    std::vector<double> distortionParams = {k1_real, k2_real, k3_real};
 
    
    Eigen::Matrix<T, 2, 1> camcoords;
    if (distorted_radius < 1e-12) {
      camcoords(0) = xd;
      camcoords(1) = yd;
    }
    else {
      double distorted_radius_square = xd_real * xd_real + yd_real * yd_real;
      double rescaler = ::sqrt(camera::radial_distortion::bisection_Radius_Solve(distortionParams, distorted_radius_square, distoFunctor));;
      camcoords(0) = (xd / distorted_radius) * static_cast<T>(rescaler);
      camcoords(1) = (yd / distorted_radius) * static_cast<T>(rescaler);    
    }

    T angle_radial = atan2(camcoords(1), camcoords(0));
    T angle_Z = camcoords.norm() * 0.5 * fov;

    out(2) = cos(angle_Z);
    out(0) = cos(angle_radial) /** / 1.0 / **/ * sin(angle_Z);
    out(1) = sin(angle_radial) /** / 1.0 / **/ * sin(angle_Z);
  }

  template <typename T>
  void unlift(const T* const cam_K, const Eigen::Matrix< T, 3, 1> & pt, Eigen::Matrix< T, 3, 1> & out) const
  {
    const T& fov = cam_K[OFFSET_FOCAL_LENGTH];
    const T& principal_point_x = cam_K[OFFSET_PRINCIPAL_POINT_X] + m_center(0);
    const T& principal_point_y = cam_K[OFFSET_PRINCIPAL_POINT_Y] + m_center(1);
    const T& k1 = cam_K[OFFSET_DISTO_K1];
    const T& k2 = cam_K[OFFSET_DISTO_K2];
    const T& k3 = cam_K[OFFSET_DISTO_K3];

    /* To unit sphere */
    Eigen::Matrix< T, 3, 1> pt_normalized = pt;
    pt_normalized.normalize();

    /* Compute angle with optical center */
    T angle_Z = atan2(sqrt(pt_normalized(0) * pt_normalized(0) + pt_normalized(1) * pt_normalized(1)), pt_normalized(2));
    T radius = angle_Z / (0.5 * fov);

    /* Ignore depth component and compute radial angle */
    T angle_radial = atan2(pt_normalized(1), pt_normalized(0));

    /* radius = focal * angle_Z */
    Eigen::Matrix< T, 2, 1> proj_pt;
    proj_pt(0) = cos(angle_radial) * radius;
    proj_pt(1) = sin(angle_radial) * radius;

    //Apply distortion
    const T r = sqrt(proj_pt(0)*proj_pt(0) + proj_pt(1)*proj_pt(1));
    const T r2 = r * r;
    const T r4 = r2 * r2;
    const T r6 = r4 * r2;
    
    const T r_coeff = (T(1.0) + k1 * r2 + k2 * r4 + k3 * r6);
    const T x_d = proj_pt(0) * r_coeff;
    const T y_d = proj_pt(1) * r_coeff;
    
    //Scale and shift
    out(0) = x_d * m_radius_size + principal_point_x;
    out(1) = y_d * m_radius_size + principal_point_y;
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
    
    Eigen::Matrix<T, 3, 1> pt3d_1;

    //From pixel to meters
    lift(cam_K, m_pos_2dpoint_first, pt3d_1);

    //Build relative rotation
    ceres::AngleAxisToRotationMatrix(cam_R1, oneRo.data());
    ceres::AngleAxisToRotationMatrix(cam_R2, twoRo.data());
    twoRone = twoRo * oneRo.transpose();

    //Transform point
    Eigen::Matrix< T, 3, 1> pt3d_2_est = twoRone * pt3d_1;

    //Project back to image space in pixels
    Eigen::Matrix< T, 3, 1> pt2d_2_est;
    unlift(cam_K, pt3d_2_est, pt2d_2_est);

    //Compute residual
    Eigen::Matrix< T, 3, 1> residual = pt2d_2_est - m_pos_2dpoint_second;

    out_residuals[0] = residual(0);
    out_residuals[1] = residual(1);

    return true;
  }

  Vec3 m_pos_2dpoint_first; // The 2D observation in first view
  Vec3 m_pos_2dpoint_second; // The 2D observation in second view
  Vec2 m_center; // Image center
  double m_radius_size;
};

} // namespace sfm
} // namespace aliceVision
