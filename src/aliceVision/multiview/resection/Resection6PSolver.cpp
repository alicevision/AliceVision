// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Resection6PSolver.hpp"
#include <aliceVision/numeric/projection.hpp>

namespace aliceVision {
namespace multiview {
namespace resection {

void translate(const Mat3X& X, const Vec3& vecTranslation, Mat3X* XPoints)
{
    XPoints->resize(X.rows(), X.cols());
    for(int i = 0; i < X.cols(); ++i)
        XPoints->col(i) = X.col(i) + vecTranslation;
}

template <typename TMat, typename TVec>
double nullspaceRatio(TMat* A, TVec* nullspace)
{
    if(A->rows() >= A->cols())
    {
        Eigen::JacobiSVD<TMat> svd(*A, Eigen::ComputeFullV);
        (*nullspace) = svd.matrixV().col(A->cols() - 1);
        return svd.singularValues()(A->cols() - 2) / svd.singularValues()(0);
    }

    // extend A with rows of zeros to make it square. It's a hack, but is
    // necessary until Eigen supports SVD with more columns than rows.
    TMat A_extended(A->cols(), A->cols());
    A_extended.block(A->rows(), 0, A->cols() - A->rows(), A->cols()).setZero();
    A_extended.block(0, 0, A->rows(), A->cols()) = (*A);

    return Nullspace(&A_extended, nullspace);
}

/**
 * @brief Setup the Direct Linear Transform.
 *        Use template in order to support fixed or dynamic sized matrix.
 */
template <typename Matrix>
void buildActionMatrix(Matrix& A, const Mat& pt2D, const Mat& XPoints)
{
    const size_t n = pt2D.cols();
    for(size_t i = 0; i < n; ++i)
    {
        size_t row_index = i * 2;
        const Vec3& X = XPoints.col(i);
        const Vec2& x = pt2D.col(i);
        A(row_index, 0) = X(0);
        A(row_index, 1) = X(1);
        A(row_index, 2) = X(2);
        A(row_index, 3) = 1.0;
        A(row_index, 8) = -X(0) * x(0);
        A(row_index, 9) = -X(1) * x(0);
        A(row_index, 10) = -X(2) * x(0);
        A(row_index, 11) = -1.0 * x(0);

        row_index = i * 2 + 1;
        A(row_index, 4) = X(0);
        A(row_index, 5) = X(1);
        A(row_index, 6) = X(2);
        A(row_index, 7) = 1.0;
        A(row_index, 8) = -X(0) * x(1);
        A(row_index, 9) = -X(1) * x(1);
        A(row_index, 10) = -X(2) * x(1);
        A(row_index, 11) = -1.0 * x(1);
    }
    // Normalize each row
    for(size_t i = 0; i < static_cast<size_t>(A.rows()); ++i)
        A.row(i).normalize();
}

void solveProblem(const Mat& x2d, const Mat& x3d, std::vector<robustEstimation::Mat34Model>& models, bool bcheck,
                  const std::vector<double>& weights)
{
    assert(2 == x2d.rows());
    assert(3 == x3d.rows());
    assert(6 <= x2d.cols());
    assert(x2d.cols() == x3d.cols());
    assert(x2d.cols() == weights.size() || weights.empty());

    //-- Translate 3D points in order to have X0 = (0,0,0,1).
    Vec3 vecTranslation = -x3d.col(0);
    Mat4 translationMatrix = Mat4::Identity();
    translationMatrix << 1., .0, .0, vecTranslation(0), .0, 1., .0, vecTranslation(1), .0, .0, 1., vecTranslation(2),
        .0, .0, .0, 1;
    Mat3X XPoints;
    translate(x3d, vecTranslation, &XPoints);

    const auto numPts = x2d.cols();

    using Vec12 = Eigen::Matrix<double, 12, 1>;
    Vec12 p;
    double ratio = -1.0;

    if(numPts == 6)
    {
        // In the case of minimal configuration we use fixed sized matrix to let
        // Eigen and the compiler doing the maximum of optimization.
        using Mat12 = Eigen::Matrix<double, 12, 12>;
        Mat12 A = Mat12::Zero(12, 12);
        buildActionMatrix(A, x2d, XPoints);
        if(!weights.empty())
        {
            for(Mat12::Index ptIdx = 0; ptIdx < numPts; ++ptIdx)
            {
                A.row(ptIdx * 2) *= weights[ptIdx];
                A.row(ptIdx * 2 + 1) *= weights[ptIdx];
            }
        }
        ratio = nullspaceRatio(&A, &p);
    }
    else
    {
        Mat A = Mat::Zero(numPts * 2, 12);
        buildActionMatrix(A, x2d, XPoints);
        if(!weights.empty())
        {
            for(Mat::Index ptIdx = 0; ptIdx < numPts; ++ptIdx)
            {
                A.row(ptIdx * 2) *= weights[ptIdx];
                A.row(ptIdx * 2 + 1) *= weights[ptIdx];
            }
        }
        ratio = nullspaceRatio(&A, &p);
    }

    if(bcheck)
    {
        if(ratio > 1e-5) // assert that at least only one solution if found by SVD
        {
            Mat34 P = Map<Mat>(p.data(), 4, 3).transpose();
            P = P * translationMatrix;
            P /= P(2, 3);

            Mat3 K, R;
            Vec3 t;
            KRt_from_P(P, &K, &R, &t);

            // assert point in front of the cam
            std::size_t cpt = 0;

            for(std::size_t i = 0; i < numPts; ++i)
            {
                cpt += (Depth(R, t, x3d.col(i)) > 0) ? 1 : 0;
            }

            if(cpt == numPts)
            {
                models.emplace_back(P);
            }
        }
    }
    else
    {
        Mat34 P = Map<Mat>(p.data(), 4, 3).transpose();
        P = P * translationMatrix;
        P /= P(2, 3);
        models.emplace_back(P);
    }
}

void Resection6PSolver::solve(const Mat& x2d, const Mat& x3d, std::vector<robustEstimation::Mat34Model>& models) const
{
    std::vector<double> weights;
    solveProblem(x2d, x3d, models, true, weights);
}

void Resection6PSolver::solve(const Mat& x2d, const Mat& x3d, std::vector<robustEstimation::Mat34Model>& models,
                              const std::vector<double>& weights) const
{
    solveProblem(x2d, x3d, models, true, weights);
}

} // namespace resection
} // namespace multiview
} // namespace aliceVision
