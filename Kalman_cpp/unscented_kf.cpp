#include "unscented_kf.h"
#include <cmath>
#include <iostream>

using namespace std;
MatrixXd Cholesky_Decomposition(const MatrixXd& C, int& n);
unscented_kf::unscented_kf(
	const MatrixXd& A_in, //System 
	const MatrixXd& C_in, //State Covariance 
	const MatrixXd& R_in, //Measurement Covariance
	const MatrixXd& Q_in, //Process Noise Covariance
	const MatrixXd& P_in) //Anchor Positions
{
	A = A_in;
	R = R_in;
	Q = Q_in;
	P = P_in;
	C = C_in;
	n = A.rows(); //number of states
	m = R.rows(); //number of measurements per time step
	Lower.resize(n, n);
	Lower.setZero();
}

void unscented_kf::init() {
	x_est.resize(n);
	x_pred.resize(n);
	z_pred.resize(m);
	x_est.setZero();

	//setting scaling parameters for the sigma points

	alpha = 0.001;
	beta = 2.0;
	kappa = 0.0;
	lambda = pow(alpha, 2) * (n + kappa) - n;
	c = n + lambda;
}

void unscented_kf::update(const VectorXd& z) {

	//Cholesky decomposition

	Lower = Cholesky_Decomposition(C, n);
	scaling_matrix = sqrt(n + lambda) * Lower;

	//Creating 2n+1 sigma points

	MatrixXd plus_sigma = x_est* MatrixXd::Ones(1, n) + scaling_matrix;
	MatrixXd minus_sigma = x_est * MatrixXd::Ones(1, n) - scaling_matrix;

	xi << x_est, plus_sigma, minus_sigma;
	MatrixXd xi_p(n, 2 * n + 1);
	MatrixXd zeta_p(n, 2 * n + 1);
	VectorXd w_m_end = 0.5 / c * VectorXd::Ones(2 * n);
	
	VectorXd w_m (lambda/c, w_m_end);
	VectorXd w_c = w_m;
	w_c(0) = w_m(0) + (1 - pow(alpha, 2) + beta);

	// Prediction

	// Predicting the sigma points
	for (int i = 0; i < 2 * n + 1; i++) {

		xi_p.block(0, i, n, 1) = A * xi.block(0, i, n, 1);
			
	}

	//Using the weights to calculate the the predicted state
	for (int i = 0; i < 2 * n + 1; i++) {

		x_pred = w_m(i) * xi_p.block(0, i, n, 1) + x_pred;

	}

	
	//Using the weights and predicted state to calculate the covariance
	for (int i = 0; i < 2 * n + 1; i++) {
		Cov_sigma = (xi_p.block(0, i, n, 1) - x_pred);
		C += w_c(i) * (Cov_sigma * Cov_sigma.transpose());
		
	}

	//Nearly Constant Acceleration Model

	C += Q;

	//Calculate zeta_p (zeta_k|k-1)

	for (int i = 0; i < 2 * n + 1; i++) {
		for (int j = 0; j < 4; j++) {
			zeta_p(j, i) = sqrt(pow(xi_p(1, i) - P(1, j), 2) + pow(xi_p(2, i) - P(2, j), 2));
		}
	}


	C_xk_zk.resize(n, 4);
	C_zk.resize(4, 4);
	C_xk_zk.setZero();
	C_zk.setZero();
	intsum_z_pred.resize(4, 2 * n + 1);

	for (int i = 0; i < 2 * n + 1; i++) {
		intsum_z_pred.block(0, i, 1, 4) = w_m(i) * zeta_p.block(0, i, 4, 1);
	}

	z_pred.setZero();
	for (int i = 0; i < 2 * n + 1; i++) {
		z_pred += intsum_z_pred.block(0, i, 4, 1);
	}
	
	for (int i = 0; i < 2 * n + 1; i++) {
		C_xk_zk_sigma = w_c(i) * (xi_p.block(0, i, n, 1)-x_pred);
		C_zk_sigma = (zeta_p.block(0, i, 4, 1) - z_pred);

		C_xk_zk += w_c(i) * C_xk_zk_sigma * C_xk_zk_sigma.transpose();
		C_zk += w_c(i) * C_zk_sigma * C_zk_sigma.transpose();
	}

	C_zk += R;

	K = C_xk_zk * C_zk.inverse();

	x_est = x_pred + K*(z - z_pred);
	C = C_xk_zk * C_zk.inverse();

	/*



	*/

	//Generate sigma points


}

MatrixXd Cholesky_Decomposition(const MatrixXd& C, int& n)
{
	MatrixXd Lower(n, n);
	// Decomposing a matrix into Lower Triangular
	for (int i = 0; i < n; i++) {
		for (int j = 0; j <= i; j++) {
			int sum = 0;

			if (j == i) // Summation for diagonals
			{
				for (int k = 0; k < j; k++)
					sum += pow(Lower(j,k), 2);
				Lower(i,j) = sqrt(Lower(j,j) -
					sum);
			}
			else {

				// Evaluating L(i, j) using L(j, j)
				for (int k = 0; k < j; k++)
					sum += (Lower(i,k) * Lower(j,k));
				Lower(i,j) = (C(i,j) - sum) /
					Lower(i,j);
			}
		}
	}
	return Lower;
}