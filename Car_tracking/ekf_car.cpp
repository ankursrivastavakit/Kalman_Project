#include "ekf_car.h"
#include <cmath>
#include <iostream>
using namespace std;

ekf_car::ekf_car(
	const MatrixXd& C_in, //State Covariance 
	const MatrixXd& R_in, //Mea0urement Covariance
	const MatrixXd& Q_in, //Process Noise Covariance
	const MatrixXd& P_in) //Anchor Positions
{
	C = C_in;
	R = R_in;
	Q = Q_in;
	P = P_in;
	n = 4;
	m = R.rows();
}

void ekf_car::init() {
	x_est.resize(n);
	x_pred.resize(n);
	z_pred.resize(m);
	x_est.setZero();
	t = 0.1;
	l = 1.5; //distance between front and rear axle
}

void ekf_car::update(const Eigen::VectorXd& u_hat, const Eigen::VectorXd& z_hat)
{
	// Defining the linearized transition matrix H and the Kalman Gain K
	MatrixXd H(m, n);
	MatrixXd K(n, m);
	MatrixXd B(n, 4);
	K.setZero();
	H.setZero();

	//Calculating the Jacobian A and B matrices

	A << 1, 0, t* cos(x_est(3)), -t * x_est(2) * sin(x_est(3)), -pow(t, 2) / 2 * u_hat * sin(x_est(3)),
		0, 1, t* sin(x_est(3)), -t * x_est(2) * cos(x_est(3)), -pow(t, 2) / 2 * u_hat * cos(x_est(3)),
		0, 0, 1, 0,
		0, 0, (t / l)* tan(u_hat(1)), 1;

	B << 0.5 * pow(t, 2) * cos(x_est(3)), 0,
		0.5 * pow(t, 2) * sin(x_est(3)), 0,
		t, 0,
		0, (t / l)* pow(1 / cos(u_hat(1)), 2);

	// Prediction step for state vector
	x_pred(0) = x_est(0) * -x_est(2) * cos(x_est(3)) * t + u_hat(0) * pow(t, 2) * 0.5 * cos(x_est(3));
	x_pred(1) = x_est(1) * -x_est(2) * sin(x_est(3)) * t + u_hat(0) * pow(t, 2) * 0.5 * sin(x_est(3));
	x_pred(2) = x_est(2) + t*u_hat(0);
	x_pred(3) = x_est(3) + tan(u_hat(1))*t*(x_est(2))/l;

	//Prediction step for covariance matrix

	C = C * A * C.transpose() + B * Q * B.transpose();

	//Calculating the Jacobian H Matrix
	
	for (int i = 0; i < n; i++) {

		z_pred(i) = sqrt((pow((x_pred(0) - P(1, i)), 2) + (pow((x_pred(1) - P(2, i)), 2))));

		for(int j = 0; j < 2; j++) // 2 Dimensions
		{
			H(i, j) = (x_pred(j) - P(j, i)) / z_pred(i);
		}

	//Filter Step
		K = C * H.transpose()*(R + H*C * H.transpose()).inverse();
		x_est = x_pred + K * (z_hat - z_pred);
	}
}