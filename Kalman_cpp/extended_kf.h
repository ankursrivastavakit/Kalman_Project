#pragma once
#include <Eigen/Dense>
using namespace Eigen;

class extended_kf
{
public:
	
	extended_kf(
		const MatrixXd& A, //System
		const MatrixXd& C, //State Covariance 
		const MatrixXd& R, //Measurement Covariance
		const MatrixXd& Q, //Process Noise Covariance
		const MatrixXd& P  //Anchor Positions
	);


	// for initializing with a zero state
	void init();

	void update(const VectorXd& z);

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
	MatrixXd A, Q, C, R, P;

	//parameters
	int m, n;

};
