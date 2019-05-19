// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/projection.hpp>
#include <aliceVision/multiview/twoViewKernel.hpp>

#include <vector>

namespace aliceVision {
namespace resection {
namespace kernel {

/**
 * Six-point resection
 * P Matrix estimation (Pose estimation)
 * Compute a projection matrix using linear least squares.
 * Rely on Linear Resection algorithm.
 * Work from 6 to N points.
 */
struct SixPointResectionSolver {
  enum { MINIMUM_SAMPLES = 6 };
  enum { MAX_MODELS = 1 };
  // Solve the problem of camera pose.
  // First 3d point will be translated in order to have X0 = (0,0,0,1)
  static void Solve(const Mat &pt2D, const Mat &pt3D, std::vector<Mat34> *P, bool bcheck = true);

  // Compute the residual of the projection distance(pt2D, Project(P,pt3D))
  static double Error(const Mat34 & P, const Vec2 & pt2D, const Vec3 & pt3D){
    Vec2 x = Project(P, pt3D);
    return (x-pt2D).norm();
  }
};

//-- Generic Solver for the 6pt Resection algorithm using linear least squares.
template<typename SolverArg,
  typename ErrorArg,
  typename ModelArg = Mat34>
class ResectionKernel :
   public twoView::kernel::Kernel<SolverArg,ErrorArg, ModelArg>
{
public:
  // 2D / 3D points
  ResectionKernel(const Mat &pt2D, const Mat &pt3D):
  twoView::kernel::Kernel<SolverArg,ErrorArg, ModelArg>(pt2D,pt3D){}

  void Fit(const std::vector<size_t> &samples, std::vector<ModelArg> *models) const {
    Mat pt2d = ExtractColumns(this->x1_, samples);
    Mat pt3d = ExtractColumns(this->x2_, samples);

    assert(2 == pt2d.rows());
    assert(3 == pt3d.rows());
    assert(SolverArg::MINIMUM_SAMPLES <= pt2d.cols());
    assert(pt2d.cols() == pt3d.cols());

    SolverArg::Solve(pt2d, pt3d, models);
  }

  // Error : re-projection error of the sample
  double Error(size_t sample, const ModelArg &model) const {
    return ErrorArg::Error(model, this->x1_.col(sample), this->x2_.col(sample));
  }
};

//-- Usable solver for the 6pt Resection estimation
typedef twoView::kernel::Kernel<SixPointResectionSolver,
  SixPointResectionSolver, Mat34>  PoseResectionKernel;

}  // namespace kernel
}  // namespace resection
}  // namespace aliceVision

//--
// Euclidean resection kernel (Have K intrinsic helps)
//--

namespace aliceVision {
namespace resection {
namespace kernel {

/**
 * Computes the extrinsic parameters, R and t for a calibrated camera from 4 or
 * more 3D points and their images.
 *
 * \param x_camera Image points in normalized camera coordinates,
 *                 e.g. x_camera = inv(K) * x_image
 * \param X_world 3D points in the world coordinate system
 * \param R       Solution for the camera rotation matrix
 * \param t       Solution for the camera translation vector
 *
 * This is the algorithm described in:
 * "{EP$n$P: An Accurate $O(n)$ Solution to the P$n$P Problem", by V. Lepetit
 * and F. Moreno-Noguer and P. Fua, IJCV 2009. vol. 81, no. 2
 * \note: the non-linear optimization is not implemented here.
 */
bool EuclideanResectionEPnP(const Mat2X &x_camera,
                            const Mat3X &X_world,
                            Mat3 *R, Vec3 *t);

struct EpnpSolver {
  enum { MINIMUM_SAMPLES = /*5*/ 6 };
  enum { MAX_MODELS = 1 };
  // Solve the problem of camera pose.
  static void Solve(const Mat &pt2D, const Mat &pt3D, std::vector<Mat34> *models)
  {
    Mat3 R;
    Vec3 t;
    Mat34 P;
    if (EuclideanResectionEPnP(pt2D, pt3D, &R, &t)) {
      P_From_KRt(Mat3::Identity(), R, t, &P); // K = Id
      models->push_back(P);
    }
  }

  // Compute the residual of the projection distance(pt2D, Project(P,pt3D))
  static double Error(const Mat34 & P, const Vec2 & pt2D, const Vec3 & pt3D) {
    return (pt2D - Project(P, pt3D)).norm();
  }
};

class ResectionKernel_K {
 public:
  typedef Mat34 Model;
  enum { MINIMUM_SAMPLES = 6 };

  ResectionKernel_K(const Mat2X &x_camera, const Mat3X &X) : x_camera_(x_camera), X_(X) {
    assert(x_camera.cols() == X.cols());
    x_image_ = x_camera_;
    K_ = Mat3::Identity();
  }

  ResectionKernel_K(const Mat2X &x_image, const Mat3X &X, const Mat3 &K)
  : x_image_(x_image), X_(X), K_(K)
  {
    assert(x_image.cols() == X.cols());
    // Conversion from image coordinates to normalized camera coordinates
    EuclideanToNormalizedCamera(x_image_, K, &x_camera_);
  }

  void Fit(const std::vector<size_t> &samples, std::vector<Model> *models) const {
    Mat2X x = ExtractColumns(x_camera_, samples);
    Mat3X X = ExtractColumns(X_, samples);
    Mat34 P;
    Mat3 R;
    Vec3 t;
    if (EuclideanResectionEPnP(x, X, &R, &t))
    {
      P_From_KRt(K_, R, t, &P);
      models->push_back(P);
    }
  }

  double Error(size_t sample, const Model &model) const {
    Mat3X X = X_.col(sample);
    Mat2X error = Project(model, X) - x_image_.col(sample);
    return error.col(0).norm();
  }

  size_t NumSamples() const {
    return static_cast<size_t>(x_camera_.cols());
  }

 private:
  // x_camera_ contains the normalized camera coordinates
  Mat2X  x_camera_, x_image_;
  const Mat3X &X_;
  Mat3 K_;
};

}  // namespace kernel
}  // namespace resection
}  // namespace aliceVision
