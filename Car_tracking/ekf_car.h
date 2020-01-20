#pragma once
#include <Eigen/Dense>
using namespace Eigen;

class ekf_car
{
public:

	ekf_car(
		const MatrixXd& C, //State Covariance 
		const MatrixXd& R, //Measurement Covariance
		const MatrixXd& Q, //Process Noise Covariance
		const MatrixXd& P  //Anchor Positions
	);


	// for initializing with a zero state
	void init();

	void update(const VectorXd& u_hat, const VectorXd& z_hat);

	VectorXd state() {
		return x_est;
	};

	MatrixXd covariance() {
		return C;
	};

private:
	//State vectors
	VectorXd x_est, x_pred;

	//Measurement vector
	VectorXd z_hat, z_pred;

	//Matrices
	MatrixXd A, B, Q, C, R, P;

	//parameters
	int m, n;
	double t,l;

};