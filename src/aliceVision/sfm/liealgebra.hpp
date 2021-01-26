#pragma once

#include <Eigen/Dense>
#include <unsupported/Eigen/KroneckerProduct>
#include <ceres/ceres.h>

namespace aliceVision {
namespace SO3 {

using Matrix = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

/**
Compute the skew symmetric matrix of the given vector 3d
@param int the 3d vector
@return a skew symmetric matrix
*/
inline Eigen::Matrix3d skew(const Eigen::Vector3d & in) {
    Eigen::Matrix3d ret;

    ret.fill(0);

    ret(0, 1) = -in(2);
    ret(1, 0) = in(2);
    ret(0, 2) = in(1);
    ret(2, 0) = -in(1);
    ret(1, 2) = -in(0);
    ret(2, 1) = in(0);

    return ret;
}

/**
Compute the exponential map of the given algebra on the group
@param algebra the 3d vector
@return a 3*3 SO(3) matrix
*/
inline Eigen::Matrix3d expm(const Eigen::Vector3d & algebra) {
    double angle = algebra.norm();

    if (angle < std::numeric_limits<double>::epsilon()) {
        return Eigen::Matrix3d::Identity();
    }

    Eigen::Matrix3d omega = skew(algebra);

    Eigen::Matrix3d ret;
    ret = Eigen::Matrix3d::Identity() + (sin(angle) / angle) * omega + ((1.0 - cos(angle)) / (angle * angle)) * omega * omega;

    return ret;
}

/**
Compute the algebra related to a given rotation matrix
@param R the input rotation matrix
@return the algebra
*/
inline Eigen::Vector3d logm(const Eigen::Matrix3d & R) {

    Eigen::Vector3d ret;

    double p1 = R(2, 1) - R(1, 2);
    double p2 = R(0, 2) - R(2, 0);
    double p3 = R(1, 0) - R(0, 1);

    double costheta = (R.trace() - 1.0) / 2.0;
    if (costheta < -1.0) {
        costheta = -1.0;
    }

    if (costheta > 1.0) {
        costheta = 1.0;
    }

    if (1.0 - costheta < 1e-24) {
        ret.fill(0);
        return ret;
    }

    double theta = acos(costheta);
    double scale = theta / (2.0 * sin(theta));

    ret(0) = scale * p1;
    ret(1) = scale * p2;
    ret(2) = scale * p3;

    return ret;
}

/**
Compute the jacobian of the logarithm wrt changes in the rotation matrix values
@param R the input rotation matrix
@return the jacobian matrix (3*9 matrix)
*/
inline Eigen::Matrix<double, 3, 9, Eigen::RowMajor> dlogmdr(const Eigen::Matrix3d & R) {
    double p1 = R(2, 1) - R(1, 2);
    double p2 = R(0, 2) - R(2, 0);
    double p3 = R(1, 0) - R(0, 1);

    double costheta = (R.trace() - 1.0) / 2.0;
    if (costheta > 1.0) costheta = 1.0;
    else if (costheta < -1.0) costheta = -1.0;

    double theta = acos(costheta);

    if (fabs(theta) < std::numeric_limits<float>::epsilon()) {
        Eigen::Matrix<double, 3, 9> J;
        J.fill(0);
        J(0, 5) = 1;
        J(0, 7) = -1;
        J(1, 2) = -1;
        J(1, 6) = 1;
        J(2, 1) = 1;
        J(2, 3) = -1;
        return J;
    }

    double scale = theta / (2.0 * sin(theta));

    Eigen::Vector3d resnoscale;
    resnoscale(0) = p1;
    resnoscale(1) = p2;
    resnoscale(2) = p3;

    Eigen::Matrix<double, 3, 3> dresdp = Eigen::Matrix3d::Identity() * scale;
    Eigen::Matrix<double, 3, 9> dpdmat;
    dpdmat.fill(0);
    dpdmat(0, 5) = 1;
    dpdmat(0, 7) = -1;
    dpdmat(1, 2) = -1;
    dpdmat(1, 6) = 1;
    dpdmat(2, 1) = 1;
    dpdmat(2, 3) = -1;


    double dscaledtheta = -0.5 * theta * cos(theta) / (sin(theta)*sin(theta)) + 0.5 / sin(theta);
    double dthetadcostheta = -1.0 / sqrt(-costheta*costheta + 1.0);

    Eigen::Matrix<double, 1, 9> dcosthetadmat;
    dcosthetadmat << 0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.5;
    Eigen::Matrix<double, 1, 9> dscaledmat = dscaledtheta * dthetadcostheta * dcosthetadmat;

    return dpdmat * scale + resnoscale * dscaledmat;
}

class LocalParameterization : public ceres::LocalParameterization {
 public:
  ~LocalParameterization() override = default;

  bool Plus(const double* x, const double* delta, double* x_plus_delta) const override {
 
    double* ptrBase = (double*)x;
    double* ptrResult = (double*)x_plus_delta;
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > rotation(ptrBase);
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > rotationResult(ptrResult);

    Eigen::Vector3d axis;
    axis(0) = delta[0];
    axis(1) = delta[1];
    axis(2) = delta[2];
    double angle = axis.norm();

    axis.normalize();

    Eigen::AngleAxisd aa(angle, axis);
    Eigen::Matrix3d Rupdate;
    Rupdate = aa.toRotationMatrix();

    rotationResult = Rupdate * rotation;

    return true;
  }

  bool ComputeJacobian(const double* /*x*/, double* jacobian) const override {
    
    Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> J(jacobian);
    //Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R(x);

    J.fill(0);

    J(1, 2) = 1;
    J(2, 1) = -1;
    J(3, 2) = -1;
    J(5, 0) = 1;
    J(6, 1) = 1;
    J(7, 0) = -1;

    return true;
  }

  int GlobalSize() const override { return 9; }

  int LocalSize() const override { return 3; }
};

}

namespace SE3 {

using Matrix = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;

/**
Compute the exponential map of the given algebra on the group
@param algebra the 6d vector
@return a 4*4 SE(3) matrix
*/
inline Eigen::Matrix4d expm(const Eigen::Matrix<double, 6, 1> & algebra){

  Eigen::Matrix4d ret;
  ret.setIdentity();

  Eigen::Vector3d vecR = algebra.block<3, 1>(0, 0);
  Eigen::Vector3d vecT = algebra.block<3, 1>(3, 0);


  double angle = vecR.norm();
  if (angle < std::numeric_limits<double>::epsilon()) {
    ret.setIdentity();
    ret.block<3, 1>(0, 3) = vecT;
    return ret;
  }

  Eigen::Matrix3d omega = SO3::skew(vecR);
  Eigen::Matrix3d V = Eigen::Matrix3d::Identity() + ((1.0 - cos(angle)) / (angle*angle)) * omega + ((angle - sin(angle)) / (angle*angle*angle)) * omega * omega;

  ret.block<3, 3>(0, 0) = SO3::expm(vecR);
  ret.block<3, 1>(0, 3) = V * vecT;

  return ret;
}


class LocalParameterization : public ceres::LocalParameterization {
public:
  LocalParameterization(bool refineRotation, bool refineTranslation) :
  _refineRotation(refineRotation), 
  _refineTranslation(refineTranslation)
  {
  }

  bool Plus(const double* x, const double* delta, double* x_plus_delta) const override {

    Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> T(x);
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> T_result(x_plus_delta);
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> vec_update(delta);
    Eigen::Matrix4d T_update = Eigen::Matrix4d::Identity();

    T_update = expm(vec_update);
    T_result = T_update * T;

    return true;
  }

  bool ComputeJacobian(const double * x, double* jacobian) const override {

    Eigen::Map<Eigen::Matrix<double, 16, 6, Eigen::RowMajor>> J(jacobian);
    Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> T(x);

    J.fill(0);

    if (_refineRotation) 
    {
      J(1, 2) = 1;
      J(2, 1) = -1;

      J(4, 2) = -1;
      J(6, 0) = 1;

      J(8, 1) = 1;
      J(9, 0) = -1;
    }

    if (_refineTranslation) 
    {
      J(12, 3) = 1;
      J(13, 4) = 1;
      J(14, 5) = 1;
    }

    return true;
  }

  int GlobalSize() const override {
    return 16;
  }

  int LocalSize() const override {
    return 6;
  }

private:
  bool _refineRotation;
  bool _refineTranslation;
};
}


namespace SO2 {

using Matrix = Eigen::Matrix<double, 2, 2, Eigen::RowMajor>;

/**
Compute the exponential map of the given algebra on the group
@param algebra the 1d vector
@return a 2*2 S0(2) matrix
*/
inline Eigen::Matrix2d expm(double algebra){

  Eigen::Matrix2d ret;
  
  ret(0, 0) = cos(algebra);
  ret(0, 1) = -sin(algebra);
  ret(1, 0) = sin(algebra);
  ret(1, 1) = cos(algebra);

  return ret;
}


class LocalParameterization : public ceres::LocalParameterization {
public:
  bool Plus(const double* x, const double* delta, double* x_plus_delta) const override {

    Eigen::Map<const Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> T(x);
    Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> T_result(x_plus_delta);
    double update = delta[0];

    Eigen::Matrix2d T_update = expm(update);
    T_result = T_update * T;

    return true;
  }

  bool ComputeJacobian(const double * x, double* jacobian) const override {

    Eigen::Map<Eigen::Matrix<double, 4, 1>> J(jacobian);

    J.fill(0);

    J(1, 0) = 1;
    J(2, 0) = -1;
    
    return true;
  }

  int GlobalSize() const override {
    return 4;
  }

  int LocalSize() const override {
    return 1;
  }
};

}

}