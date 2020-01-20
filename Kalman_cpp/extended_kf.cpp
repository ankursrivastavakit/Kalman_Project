#include "extended_kf.h"
#include <cmath>
#include <iostream>
using namespace std;

extended_kf::extended_kf(
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
}

// For the initial time step
void extended_kf::init() {
	x_est.resize(n);
	x_pred.resize(n);
	z_pred.resize(m);
	x_est.setZero();
}

void extended_kf::update(const Eigen::VectorXd& z_hat) {

	// Defining the linearized transition matrix H and the Kalman Gain K
	MatrixXd H(m, n);
	MatrixXd K(n, m);
	K.setZero();
	H.setZero();

	//Prediction step
	x_pred = A * x_est;
	C = (A * C * A.transpose()) + Q;

	//Calculating the predicted measurement vector and linearized H Matrix
	//EKF uses a first-order Taylor approximation
	z_pred.setZero();

	for (int i = 0; i < P.cols(); i++) {
		z_pred(i) = sqrt(pow(x_pred(0) - P(0,i), 2) + pow(x_pred(1) - P(1,i), 2));
		for (int j = 0; j < P.rows(); j++) {
			H(i, j) = (x_pred(j) - P(j,i))/z_pred(i);
		}
	}

	//Calculating the Kalman Gain
	K = (C * H.transpose()) * (H * C * H.transpose() + R).inverse();

	//Filter step
	x_est = x_pred + K * (z_hat - z_pred);
	C = C - K * H * C;
}