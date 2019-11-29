#include <Eigen/Dense>
using namespace Eigen;

class unscented_kf
{
	public:
		unscented_kf(
			const MatrixXd& A, //System
			const MatrixXd& C, //State Covariance 
			const MatrixXd& R, //Measurement Covariance
			const MatrixXd& Q, //Process Noise Covariance
			const MatrixXd& P  //Anchor Positions
		);


void init();

void update(const VectorXd& z);

VectorXd state() {
	return x_est;
};

MatrixXd covariance() {
	return C;
};

private:

//state vectors
VectorXd x_est, x_pred;

//Measurement vector
VectorXd z_hat, z_pred;
//Matrices
MatrixXd A, Q, C, R, P, Lower, scaling_matrix, xi, K, zeta_p; //standard KF matrices (and predicted measurement)

//Intermediate Matrices for UKF

MatrixXd C_xk_zk, C_zk, intsum_z_pred; //Correlations and cross-correlations of predicted values (see cpp file)
VectorXd Cov_sigma, C_xk_zk_sigma, C_zk_sigma; // the vectors of sigma values for calculating the correlation matrices
//parameters
int m, n;

double alpha, beta, kappa, lambda, c; //scaling factors

};