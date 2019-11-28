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
MatrixXd A, Q, C, R, P;

//parameters
int m, n;

};