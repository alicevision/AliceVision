// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "rigidTransformation3D.hpp"

#include <aliceVision/robustEstimation/ACRansac.hpp>

namespace aliceVision {
namespace geometry {

int lm_SRTRefine_functor::operator()(const Vec &x, Vec &fvec) const
{
  // convert x to rotation matrix and a translation vector and a Scale factor
  // x = {tx,ty,tz,anglex,angley,anglez,S}
  Vec3 transAdd = x.block<3,1>(0,0);
  Vec3 rot = x.block<3,1>(3,0);
  double Sadd = x(6);

  //Build the rotation matrix
  Mat3 Rcor =
    (Eigen::AngleAxis<double>(rot(0), Vec3::UnitX())
    * Eigen::AngleAxis<double>(rot(1), Vec3::UnitY())
    * Eigen::AngleAxis<double>(rot(2), Vec3::UnitZ())).toRotationMatrix();

  const Mat3 nR  = _R*Rcor;
  const Vec3 nt = _t+transAdd;
  const double nS = _S+Sadd;

  // Evaluate re-projection errors
  Vec3 proj;
  for (Mat::Index i = 0; i < _x1.cols(); ++i)
  {
    proj = _x2.col(i) -  (nS *  nR * (_x1.col(i)) + nt);
    fvec[i*3]   = proj(0);
    fvec[i*3+1] = proj(1);
    fvec[i*3+2] = proj(2);
  }
  return 0;
}

int lm_RRefine_functor::operator()(const Vec &x, Vec &fvec) const
{
  // convert x to rotation matrix
  // x = {anglex,angley,anglez}
  Vec3 rot = x.block<3,1>(0,0);

  //Build the rotation matrix
  Mat3 Rcor =
    (Eigen::AngleAxis<double>(rot(0), Vec3::UnitX())
    * Eigen::AngleAxis<double>(rot(1), Vec3::UnitY())
    * Eigen::AngleAxis<double>(rot(2), Vec3::UnitZ())).toRotationMatrix();

  const Mat3 nR  = _R*Rcor;
  const Vec3 nt = _t;
  const double nS = _S;

  // Evaluate re-projection errors
  Vec3 proj;
  for (Mat::Index i = 0; i < _x1.cols(); ++i)
  {
    proj = _x2.col(i) -  (nS *  nR * (_x1.col(i)) + nt);
    fvec[i*3]   = proj(0);
    fvec[i*3+1] = proj(1);
    fvec[i*3+2] = proj(2);
  }
  return 0;
}

void Refine_RTS(const Mat &x1,
  const Mat &x2,
  double &S,
  Vec3 &t,
  Mat3 &R)
{
  {
    lm_SRTRefine_functor functor(7, 3*x1.cols(), x1, x2, S, R, t);

    Eigen::NumericalDiff<lm_SRTRefine_functor> numDiff(functor);

    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<lm_SRTRefine_functor> > lm(numDiff);
    lm.parameters.maxfev = 1000;
    Vec xlm = Vec::Zero(7); // The deviation vector {tx,ty,tz,rotX,rotY,rotZ,S}

    lm.minimize(xlm);

    Vec3 transAdd = xlm.block<3,1>(0,0);
    Vec3 rot = xlm.block<3,1>(3,0);
    double SAdd = xlm(6);

    //Build the rotation matrix
    Mat3 Rcor =
      (Eigen::AngleAxis<double>(rot(0), Vec3::UnitX())
      * Eigen::AngleAxis<double>(rot(1), Vec3::UnitY())
      * Eigen::AngleAxis<double>(rot(2), Vec3::UnitZ())).toRotationMatrix();

    R = R * Rcor;
    t = t + transAdd;
    S = S + SAdd;
  }

  // Refine rotation
  {
    lm_RRefine_functor functor(3, 3*x1.cols(), x1, x2, S, R, t);

    Eigen::NumericalDiff<lm_RRefine_functor> numDiff(functor);

    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<lm_RRefine_functor> > lm(numDiff);
    lm.parameters.maxfev = 1000;
    Vec xlm = Vec::Zero(3); // The deviation vector {rotX,rotY,rotZ}

    lm.minimize(xlm);

    Vec3 rot = xlm.block<3,1>(0,0);

    //Build the rotation matrix
    Mat3 Rcor =
      (Eigen::AngleAxis<double>(rot(0), Vec3::UnitX())
      * Eigen::AngleAxis<double>(rot(1), Vec3::UnitY())
      * Eigen::AngleAxis<double>(rot(2), Vec3::UnitZ())).toRotationMatrix();

    R = R * Rcor;
  }
}

bool ACRansac_FindRTS(const Mat& x1,
                      const Mat& x2,
                      std::mt19937 &randomNumberGenerator,
                      double& S,
                      Vec3& t,
                      Mat3& R,
                      std::vector<std::size_t>& vec_inliers,
                      bool refine)
{
  assert(3 == x1.rows());
  assert(3 <= x1.cols());
  assert(x1.rows() == x2.rows());
  assert(x1.cols() == x2.cols());

  const std::size_t numIterations = 1024;
  const double dPrecision = std::numeric_limits<double>::infinity();

  using SolverT = geometry::RTSSolver;
  using KernelT = ACKernelAdaptor_PointsRegistrationSRT<SolverT, geometry::RTSSquaredResidualError>;

  const KernelT kernel = KernelT(x1, x2);

  robustEstimation::MatrixModel<Mat4> RTS;

  // robust estimation of the Projection matrix and its precision
  const std::pair<double, double> ACRansacOut = robustEstimation::ACRANSAC(kernel, randomNumberGenerator, vec_inliers, numIterations, &RTS, dPrecision);

  const bool good = decomposeRTS(RTS.getMatrix(), S, t, R);

  // return if it is not good or refine is not required
  if(!good || !refine)
    return good;

  const std::size_t nbInliers = vec_inliers.size();
  //only refine the inliers
  Mat inliers1 = Mat3X(3, nbInliers);
  Mat inliers2 = Mat3X(3, nbInliers);

  for(std::size_t i = 0; i < nbInliers; ++i)
  {
    inliers1.col(i) = x1.col(vec_inliers[i]);
    inliers2.col(i) = x2.col(vec_inliers[i]);
  }

  geometry::Refine_RTS(inliers1, inliers2, S, t, R);

  return good;

}

} // namespace geometry
} // namespace aliceVision
