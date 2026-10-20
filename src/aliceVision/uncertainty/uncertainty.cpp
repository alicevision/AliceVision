// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/uncertainty/uncertainty.hpp>

#include <cmath>
#include <limits>


namespace
{

double factorial(int n)
{
    double result = 1.0;
    for (int i = 2; i <= n; ++i)
    {
        result *= i;
    }
    return result;
}

} // namespace


namespace aliceVision
{
namespace uncertainty
{

bool computeDenseHessianInverse(Eigen::MatrixXd & inverse, Eigen::MatrixXd & hessian)
{
    using DRMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
    using DRVector = Eigen::Vector<double, Eigen::Dynamic>;
    
    int numParameters = hessian.rows();
    inverse.resize(numParameters, numParameters);

    int lwork = numParameters*numParameters + 3*numParameters + 2*numParameters * 32;
	std::vector<double> hwork(lwork);

    DRVector sv(numParameters);
    DRMatrix U(numParameters, numParameters);
    DRMatrix Vt(numParameters, numParameters);
    
    magma_int_t err;
    lapackf77_dgesvd(
        lapack_vec_const(MagmaAllVec),
        lapack_vec_const(MagmaAllVec),
        &numParameters,
        &numParameters,
        hessian.data(),
        &numParameters,
        sv.data(),
        U.data(),
        &numParameters,
        Vt.data(),
        &numParameters,
        hwork.data(),
        &lwork,
        &err);

    if (err)
    {
        ALICEVISION_LOG_ERROR("lapackf77_dgesvd");
        ALICEVISION_LOG_ERROR(magma_strerror(err));
        return false;
    }


    for (int i = 0; i < numParameters; ++i) 
    {
        //std::cout << sv(i) << std::endl;
		for (int j = 0; j < numParameters; ++j)
        {
			U(i, j) *= (sv(j) < 1e-12) ? 0.0 : 1.0 / sv(j);
        }
    }
    

    double alpha = 1, beta = 0;
	blasf77_dgemm(
        lapack_trans_const(MagmaNoTrans), 
        lapack_trans_const(MagmaNoTrans), 
        &numParameters, 
        &numParameters, 
        &numParameters,
		&alpha, 
        U.data(), 
        &numParameters, 
        Vt.data(), 
        &numParameters, 
        &beta, 
        inverse.data(), 
        &numParameters);

    return true;
}


bool computeHessianInverse(Eigen::MatrixXd & HccInverse, const Eigen::MatrixXd & Hcc, magma_queue_t queue)
{
    const double scale = Hcc.diagonal().cwiseAbs().mean();
    double lambda = std::max(1e-14 * scale, std::numeric_limits<double>::epsilon());

    const magma_int_t n = static_cast<magma_int_t>(Hcc.rows());
    magma_int_t magmaError;

    // Allocate 4 GPU buffers: dIZorig (Z^{-1}), dHcc (series accumulator),
    // dIZadd (current power of Z^{-1}), dTemp (scratch for multiply)
    MagmaBuffer dIZorig(n * n);
    MagmaBuffer dHcc(n * n);
    MagmaBuffer dIZadd(n * n);
    MagmaBuffer dTemp(n * n);

    if (!dIZorig.valid() || !dHcc.valid() || !dIZadd.valid() || !dTemp.valid())
    {
        ALICEVISION_LOG_ERROR("computeHessianInverse: GPU memory allocation failed (n=" << n << ")");
        return false;
    }

    while (1)
    {
        ALICEVISION_LOG_DEBUG("Trying cholesky factorization with lambda = " << lambda);

        Eigen::MatrixXd Z = Hcc + lambda * Eigen::MatrixXd::Identity(Hcc.rows(), Hcc.cols());
        // Uploading Z to GPU (-> DIZorig)
        magma_dsetmatrix(n, n, Z.data(), n, dIZorig, n, queue);

        // Cholesky factorization: dIZorig = L * L^T  (lower triangle stored in-place).
        // Requires Z to be symmetric positive definite; info > 0 means the leading
        // minor of order info is not positive definite (Z is not SPD).
        // Z being not SPD would mean that the selected lambda is not large enough
        magma_dpotrf_gpu(MagmaLower, n, dIZorig, n, &magmaError);
        if (magmaError == 0)
        {
            break;
        }

        lambda *= 10.0;
    }

    // Inversion from the Cholesky factor: dIZorig <- Z^{-1}  (lower triangle only).
    // Uses the factored form produced by dpotrf; ~2x cheaper than a general LU inversion.
    // Only the lower triangle of dIZorig is valid after this call.
    magma_dpotri_gpu(MagmaLower, n, dIZorig, n, &magmaError);
    if (magmaError != 0)
    {
        ALICEVISION_LOG_ERROR("computeHessianInverse: magma_dpotrf_gpu failed.");
        ALICEVISION_LOG_ERROR("Reason : " << magma_strerror(magmaError));
        return false;
    }

    // potri fills only the lower triangle; download, symmetrize on CPU, re-upload
    HccInverse.resize(n, n);
    magma_dgetmatrix(n, n, dIZorig, n, HccInverse.data(), n, queue);
    HccInverse = HccInverse.selfadjointView<Eigen::Lower>();
    magma_dsetmatrix(n, n, HccInverse.data(), n, dIZorig, n, queue);

    // dHcc = Z^{-1}  (first term of the series: i=0)
    magma_dcopymatrix(n, n, dIZorig, n, dHcc, n, queue);

    // dIZadd = Z^{-2} = Z^{-1} * Z^{-1}
    // dIZorig is fully symmetrized (both triangles valid), so dgemm is used
    // throughout — it is significantly faster than dsymm for large n on GPU.
    magma_dgemm(MagmaNoTrans, MagmaNoTrans, n, n, n, 1.0, dIZorig, n, dIZorig, n, 0.0, dIZadd, n, queue);

    // Two device scalars for the GPU-side convergence check:
    //   d_cur_maxabs  — max|dIZadd| for the current iteration (written by reduction kernel)
    //   d_prev_norm   — max|k * dIZadd| from the previous iteration
    // Initialized to +inf so the first iteration always passes the check.
    MagmaBuffer d_cur_maxabs(1);
    MagmaBuffer d_prev_norm(1);
    if (!d_cur_maxabs.valid() || !d_prev_norm.valid())
    {
        ALICEVISION_LOG_ERROR("computeHessianInverse: GPU scalar allocation failed");
        return false;
    }

    cudaStream_t stream = magma_queue_get_cuda_stream(queue);

    // d_prev_norm = infinity;
    const double inf = std::numeric_limits<double>::infinity();
    cudaMemcpyAsync(d_prev_norm, &inf, sizeof(double), cudaMemcpyHostToDevice, stream);

    for (int i = 1; i < 20; i++)
    {
        double k = pow(lambda, i) / factorial(i - 1);

        // Compute max|dIZadd| fully on GPU (no CPU sync; result in d_cur_maxabs)
        launchMaxAbsReduction(dIZadd, d_cur_maxabs, n * n, stream);

        // dHcc += k * dIZadd, but only when max|k * dIZadd| has not increased.
        // The kernel skips the axpy and leaves dHcc unchanged when diverging.
        // d_prev_norm is updated in-place by the kernel when the check passes.
        // Note that even if the value increase, the computations are still done but ignored.
        // This is better than having to deal with costly cpu<-->gpu synchronisation
        launchConditionalAxpy(k, dIZadd, dHcc, d_cur_maxabs, d_prev_norm, n * n, stream);

        // Advance the power: dTemp = dIZadd * dIZorig
        magma_dgemm(MagmaNoTrans, MagmaNoTrans, n, n, n, 1.0, dIZadd, n, dIZorig, n, 0.0, dTemp, n, queue);

        std::swap(dIZadd, dTemp);
    }
    
    // Download result from GPU to host
    // The synchronisation with the GPU is done here.
    magma_dgetmatrix(n, n, dHcc, n, HccInverse.data(), n, queue);
    
    // Although theorical symmetry is preserved,
    // It is better to enforce numerically this symmetry here.
    HccInverse = HccInverse.selfadjointView<Eigen::Lower>();

    return true;
}

}
}