#pragma once

#include <Eigen/Dense>

namespace ppacr {

template <size_t M, size_t N>
Eigen::Matrix<double, M*N, M*N> getJacobian_At_wrt_A() {
	Eigen::Matrix<double, M*N, M*N> ret;

	/** vec(M1*M2*M3) = kron(M3.t, M1) * vec(M2) */
	/** vec(IAtB) = kron(B.t, I) * vec(A) */
	/** dvec(IAtB)/dA = kron(B.t, I) * dvec(At)/dA */

	ret.fill(0);

	size_t pos_at = 0;
	for (size_t i = 0; i < M; i++) {
		for (size_t j = 0; j < N; j++) {
			size_t pos_a = N * j + i;
			ret(pos_at, pos_a) = 1;

			pos_at++;
		}
	}

	return ret;
}

template <size_t M, size_t N, size_t K>
Eigen::Matrix<double, M*K, M*N> getJacobian_AB_wrt_A(const Eigen::Matrix<double, M , N> & A, const Eigen::Matrix<double, N, K> & B)  {
	Eigen::Matrix<double, M*K, M*N> ret;

	/** vec(M1*M2*M3) = kron(M3.t, M1) * vec(M2) */
	/** vec(IAB) = kron(B.t, I) * vec(A) */
	/** dvec(IAB)/dA = kron(B.t, I) * dvec(A)/dA */
	/** dvec(IAB)/dA = kron(B.t, I) */

	ret.fill(0);

	Eigen::Matrix<double, K, N> Bt = B.transpose();

	for (size_t row = 0; row < K; row++) {
		for (size_t col = 0; col < N; col++) {

			ret.template block<M, M>(row * M, col * M) = Bt(row, col) * Eigen::Matrix<double, M, M>::Identity();
		}
	}


	return ret;
}

template <size_t M, size_t N, size_t K>
Eigen::Matrix<double, M*K, M*N> getJacobian_AtB_wrt_A(const Eigen::Matrix<double, M, N> & A, const Eigen::Matrix<double, M, K> & B) {
	return getJacobian_AB_wrt_A<M, N, K>(A.transpose(), B) * getJacobian_At_wrt_A<M, N>();
}

template <size_t M, size_t N, size_t K>
Eigen::Matrix<double, M*K, N*K> getJacobian_AB_wrt_B(const Eigen::Matrix<double, M, N> & A, const Eigen::Matrix<double, N, K> & B) {
	Eigen::Matrix<double, M*K, N*K> ret;

	/** vec(M1*M2*M3) = kron(M3.t, M1) * vec(M2) */
	/** vec(ABI) = kron(I, A) * vec(B) */
	/** dvec(ABI)/dB = kron(I, A) * dvec(B)/dB */
	/** dvec(ABI)/dB = kron(I, A) */

	ret.fill(0);

	for (size_t index = 0; index < K; index++) {

		ret.template block<M, N>(M * index, N * index) = A;
	}

	return ret;
}


}
