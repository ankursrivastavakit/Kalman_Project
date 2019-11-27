#include "unscented_kf.h"
#include <cmath>
#include <iostream>

using namespace std;

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
}

