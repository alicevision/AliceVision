// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/utils/scope.hpp>
#include <aliceVision/uncertainty/magma.hpp>

#include <Eigen/Sparse>
#include <map>
#include <vector>
#include <limits>

#include "uncertainty_kernels.hpp"

namespace aliceVision 
{
namespace uncertainty
{

/**
 * @brief Compute the pseudo-inverse of a dense Hessian via full SVD.
 *
 * Performs a full singular value decomposition of @p hessian using LAPACK
 * dgesvd (via MAGMA's CPU interface), then builds the Moore-Penrose
 * pseudo-inverse by inverting every singular value whose magnitude exceeds
 * 1e-12 and zeroing the rest:
 *
 *   H = U * diag(sv) * Vt
 *   H^+ = Vt^T * diag(sv^+) * U^T,   sv^+(i) = 1/sv(i) if sv(i) >= 1e-12, else 0
 *
 * The result is assembled with a BLAS dgemm call.
 *
 * @note This routine does not exploit symmetry and runs entirely on the CPU.
 *       For large symmetric positive-definite systems prefer computeHessianInverse(),
 *       which uses GPU Cholesky and is considerably faster.
 *
 * @param[out] inverse   Pseudo-inverse of @p hessian (n x n).
 * @param[in]  hessian   Dense square matrix to invert (n x n). The matrix is
 *                       read via its raw data pointer and must be stored
 *                       column-major (standard Eigen layout).
 * @return true on success; false if dgesvd reports an error.
 */
bool computeDenseHessianInverse(Eigen::MatrixXd & inverse, Eigen::MatrixXd & hessian);

/**
 * @brief Compute an approximate inverse of the camera Schur-complement Hessian.
 *
 * See the implementation in uncertainty.cpp for a full mathematical description.
 * In brief:
 *  1. Regularize: Z = Hcc + lambda * I  (lambda chosen to ensure Z is SPD).
 *  2. Invert Z via GPU Cholesky (dpotrf + dpotri).
 *  3. Recover Hcc^{-1} via a Neumann series correction truncated at 19 terms:
 *       Hcc^{-1} \approx Z^{-1} + \sum_{i=1}^{19} [\lambda^i / (i-1)!] * Z^{-(i+1)}
 *     The series is evaluated on GPU using MAGMA dgemm; each term is conditionally
 *     accumulated based on a GPU-side convergence check that stops before
 *     advancing to numerically unsafe higher powers.
 *
 * @param[out] HccInverse  Approximate inverse of Hcc (n x n, symmetric).
 * @param[in]  Hcc         Camera Schur-complement Hessian (n x n, symmetric SPD after regularization).
 * @param[in]  queue       Active MAGMA queue (provides the CUDA stream used for all GPU operations).
 * @return true on success; false if GPU allocation fails or Cholesky detects a non-SPD matrix.
 */
bool computeHessianInverse(Eigen::MatrixXd & HccInverse, const Eigen::MatrixXd & Hcc, magma_queue_t queue);

/**
 * @brief Build the camera Schur-complement Hessian Hcc from the BA Jacobian.
 *
 * Given the full bundle-adjustment Jacobian J partitioned as:
 *
 *   J = [ Jc | Jp ]   (Jc: camera columns, Jp: point/landmark columns)
 *
 * the full Hessian is:
 *
 *   JtJ = [ U   W  ]   U = Jc^T Jc,  W = Jc^T Jp,  V = Jp^T Jp
 *         [ W^T V  ]
 *
 * The Schur complement (marginalizing out landmarks) is:
 *
 *   Hcc = U - W V^{-1} W^T
 *
 * V is block-diagonal (3x3 per landmark), so V^{-1} is computed per-landmark
 * via Cholesky (LLT). Each 3x3 block is inverted independently.
 *
 * The Jacobian is expected to be column-scaled by @p vectorScale before
 * accumulation, i.e. the effective Jacobian used is J * diag(vectorScale).
 * This scaling is applied element-wise during the sparse pass and must be
 * undone on the result by the caller.
 *
 * @tparam Derived        Row-major Eigen sparse matrix type.
 * @param[out] Hcc        Camera Schur-complement Hessian (camParams x camParams, symmetric).
 * @param[in]  vectorScale Per-column scale vector of length J.cols().
 * @param[in]  jacobian   Row-major sparse Jacobian (residuals x (camParams + landmarkParams)).
 * @param[in]  countCameras   Number of camera parameters.
 * @param[in]  countLandmarks Number of landmark blocks (each contributes 3 parameters).
 * @return true on success.
 */
template<typename Derived>
bool computeHessianCameras(Eigen::MatrixXd & Hcc, const Eigen::VectorXd & vectorScale, const Derived & jacobian, size_t countLandmarks)
{
    const int n = jacobian.cols();
    const int landmarkBlocks = countLandmarks;
    const int landmarkParams = landmarkBlocks * 3;
    const int camParams = n - landmarkParams;

    // JtJ = 
    // [U  W]
    // [Wt V]

    // U: dense cam×cam Schur complement accumulator (symmetric)
    Eigen::MatrixXd U = Eigen::MatrixXd::Zero(camParams, camParams);

    struct ResidualContrib {
        // Camera indexed jacobian
        std::vector<std::pair<int, Eigen::VectorXd>> gradientsCameras; 
        Eigen::Vector3d gradientPoint = Eigen::Vector3d::Zero();
    };

    struct LandmarkAccum {
        Eigen::Matrix3d V = Eigen::Matrix3d::Zero();
        std::vector<ResidualContrib> residuals;
    };

    // One LandmarkAccum per landmark
    std::vector<LandmarkAccum> landmarkAccumulators(landmarkBlocks);

    // Single pass over J rows
    for (int row = 0; row < jacobian.outerSize(); ++row)
    {
        std::map<int, Eigen::VectorXd> gradientsCameras;
        Eigen::Vector3d gradientPoint = Eigen::Vector3d::Zero();
        int lmIdx = -1;
        std::vector<double> block;

        int baseCamera = 0;
        int last = 0;

        // Distribute the jacobian per block (camera / landmark)
        for (typename Derived::InnerIterator it(jacobian, row); it; ++it)
        {
            const int column = it.col();
            const double cellValue = it.value() * vectorScale(column);
            
            // We use the jacobian sparsity to create parameters groups
            // As we ignore the true structure of the jacobian here,
            // Just enforce the sparsity and divide by blocks.
            if (column > last + 1 || column >= camParams)
            {
                if (block.size() > 0)
                {
                    // BaseCamera is the first column for this block
                    // Read correctly this line,
                    // It is COPYING even if it's a map.
                    // We just avoir copying to an intermediate matrix.
                    gradientsCameras[baseCamera] = Eigen::Map<const Eigen::VectorXd>(block.data(), block.size());
                    block.clear();                    
                }

                baseCamera = column;
            }
            last = column;
            
            if (column < camParams)
            {
                block.push_back(cellValue);
            }
            else
            {
                // Compute initial column for landmark
                const int localId = column - camParams;
                const int idx = localId / 3;
                const int base = idx * 3;
                
                // There is only one landmark per residual
                if (lmIdx < 0) 
                {
                    lmIdx = idx;
                }
                
                // Store the jacobian for the landmark
                gradientPoint(localId - base) = cellValue;
            }
        }

        if (!block.empty())
        {
            gradientsCameras[baseCamera] = Eigen::Map<const Eigen::VectorXd>(block.data(), block.size());
            block.clear();
        }


        if (gradientsCameras.empty() && lmIdx < 0)
        {
            continue;
        }

        // Rank-1 updates to U: all pairs of camera blocks in this row
        for (auto & [base_i, cam_i] : gradientsCameras)
        {
            for (auto & [base_j, cam_j] : gradientsCameras)
            {
                U.block(base_i, base_j, cam_i.size(), cam_j.size()).noalias() += cam_i * cam_j.transpose();
            }
        }

        if (lmIdx < 0)
        {
            continue;
        }

        auto & landmarkAccumulator = landmarkAccumulators[lmIdx];

        // Rank-1 update to V
        landmarkAccumulator.V.noalias() += gradientPoint * gradientPoint.transpose();

        // Store residual for Schur complement only if cameras are present
        // (landmark-only priors contribute to V but not to the cross-term W)
        if (!gradientsCameras.empty())
        {
            ResidualContrib rc;
            rc.gradientPoint = gradientPoint;
            for (auto & [base, cam] : gradientsCameras)
            {
                rc.gradientsCameras.emplace_back(base, cam);
            }

            landmarkAccumulator.residuals.push_back(std::move(rc));
        }
    }

    // U is symmetric — it was built by accumulating all (cam_i, cam_j) pairs
    // including cross-camera terms, so just symmetrize the full matrix
    U = U.selfadjointView<Eigen::Upper>();

    // Schur complement: S = U - W * V^{-1} * W^T

    // V is block diagonal, so is V^-1. One block per landmark.
    // Therefore we can group gradients per landmark l
    // S = U - sum_l K_l
    // K_l = W_l * V_l^-1 * W_l.transpose()

    // For each residual i related to landmark l
    // W_l = sum_i gradientCamera_i * gradientPoint_i.transpose()
    // K_l = sum_i gradientCamera_i * gradientPoint_i.transpose() * V_l^-1 * (sum_j gradientCamera_j * gradientPoint_j.transpose()).transpose()
    // K_l = sum_i gradientCamera_i * gradientPoint_i.transpose() * V_l^-1 * (sum_j gradientPoint_j * gradientCamera_j.transpose())
    
    // Thanks to distributivity :
    // K_l = sum_i gradientCamera_i * gradientPoint_i.transpose() * (sum_j V_l^-1 * gradientPoint_j * gradientCamera_j.transpose())
    // K_l = sum_i (sum_j gradientCamera_i * gradientPoint_i.transpose() * V_l^-1 * gradientPoint_j * gradientCamera_j.transpose())

    // Note that gradientPoint_i.transpose() * V_l^-1 * gradientPoint_j is a scalar

    // K_l = sum_i (sum_j gradientPoint_i.transpose() * V_l^-1 * gradientPoint_j * gradientCamera_i * gradientCamera_j.transpose())
    
    Hcc = U;
    for (int b = 0; b < landmarkBlocks; ++b)
    {
        auto & landmarkAccumulator = landmarkAccumulators[b];
        if (landmarkAccumulator.residuals.empty()) 
        {
            continue;
        }

        Eigen::LLT<Eigen::Matrix3d> llt(landmarkAccumulator.V);

        for (const auto & ri : landmarkAccumulator.residuals)
        {
            const Eigen::Vector3d z_i = llt.solve(ri.gradientPoint);

            for (const auto & rj : landmarkAccumulator.residuals)
            {
                const double scale = rj.gradientPoint.dot(z_i);

                for (const auto & [base_i, cam_i] : ri.gradientsCameras)
                {
                    for (const auto & [base_j, cam_j] : rj.gradientsCameras)
                    {
                        Hcc.block(base_i, base_j, cam_i.size(), cam_j.size()).noalias() -= scale * (cam_i * cam_j.transpose());
                    }
                }
            }
        }
    }

    return true;
}

/**
 * @brief Compute the covariance matrix of all camera parameters from the BA Jacobian.
 * 
 * Camera parameters are all the "non-landmark" parameters. This include poses, distortions, intrinsics, etc.
 *
 * This is the top-level entry point. The full pipeline is:
 *
 *  1. **Column scaling**: normalize each Jacobian column by 1/||col|| so that
 *     all parameters live on a comparable scale. Degenerate columns (norm < 1e-12)
 *     are zeroed.
 *
 *  2. **Schur complement** (computeHessianCameras): build the scaled camera
 *     Hessian Hcc' = Dc * Hcc * Dc, where Dc = diag(invNorms[0..camParams-1]),
 *     by marginalizing out the landmark parameters.
 *
 *  3. **Inversion** (computeHessianInverse): compute Hcc'^{-1} on GPU via
 *     Cholesky + Neumann series correction.
 *
 *  4. **Unscaling**: recover the covariance in the original parameter units:
 *     Sigma_cc = Dc * Hcc'^{-1} * Dc
 *
 *
 * @tparam Derived         Row-major Eigen sparse matrix type.
 * @param[out] covarianceCameras  Output covariance matrix.
 * @param[in]  jacobian           Row-major sparse BA Jacobian. The parameters (columns) must be ordered such that cameras paramters appears first, then the landmarks.
 * @param[in]  countLandmarks     Number of landmarks.
 * @return true on success; false on any GPU or structural error.
 */
template<typename Derived>
bool computeUncertainty(Eigen::MatrixXd & covarianceCameras, const Eigen::SparseMatrixBase<Derived> & jacobian, size_t countLandmarks)
{
    static_assert(Derived::IsRowMajor, "jacobian must be a row-major sparse matrix (RowMajor storage order)");

	magma_int_t err = magma_init();
	if (err)
    {
        ALICEVISION_LOG_ERROR("Error on magma_init: " << magma_strerror(err));
        return false;
    }

    magma_queue_t queue;
	magma_queue_create(0, &queue);

    utils::ScopeGuard guard([&queue]() {
        magma_queue_destroy(queue);
	    magma_finalize();
    });

    const Derived & J = jacobian.derived();
    const int n = J.cols();
        
    // Build the the squared column norms of J
    // We iterate per row as it is the storage of the matrix
    Eigen::VectorXd colSqNorms = Eigen::VectorXd::Zero(n);
    for (int row = 0; row < J.outerSize(); ++row)
    {
        for (typename Derived::InnerIterator it(J, row); it; ++it)
        {
            colSqNorms(it.col()) += it.value() * it.value();
        }
    }
    
    // Compute the scale, which is the inverse of the square root of colSqNorm
    Eigen::VectorXd invNorms = (colSqNorms.array() < 1e-24).select(Eigen::ArrayXd::Zero(n), colSqNorms.array().sqrt().inverse());

    int landmarkBlocks = countLandmarks;
    int landmarkParams = landmarkBlocks * 3;
    int camParams = n - landmarkParams;

    // We should have at least the number of landmarks params in the jacobian
    if (camParams < 0)
    {
        ALICEVISION_LOG_ERROR("Incorrect structure");
        return false;
    }
    
    Eigen::MatrixXd Hcc;
    if (!computeHessianCameras(Hcc, invNorms, J, landmarkBlocks))
    {
        ALICEVISION_LOG_ERROR("Failed to build Hcc");
        return false;
    }

    Eigen::MatrixXd Hcc_inv;
    if (!computeHessianInverse(Hcc_inv, Hcc, queue))
    {
        ALICEVISION_LOG_ERROR("Failed to build inverse");
        return false;
    }
    
    // Cancel the scaling
    const Eigen::VectorXd invNorms_c = invNorms.head(camParams);
    covarianceCameras = invNorms_c.asDiagonal() * Hcc_inv * invNorms_c.asDiagonal();

    return true;
}

}
}