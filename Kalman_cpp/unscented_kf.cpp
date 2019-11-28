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

void unscented_kf::init() {
	x_est.resize(n);
	x_pred.resize(n);
	z_pred.resize(m);
	x_est.setZero();
}

void unscented_kf::update(const VectorXd& z) {

	//Cholesky decomposition

	//replace with Eigen matrices
	/*
	void Cholesky_Decomposition(int matrix[][MAX],
									  int n)
{
	int lower[n][n];
	memset(lower, 0, sizeof(lower));

	// Decomposing a matrix into Lower Triangular
	for (int i = 0; i < n; i++) {
		for (int j = 0; j <= i; j++) {
			int sum = 0;

			if (j == i) // summation for diagnols
			{
				for (int k = 0; k < j; k++)
					sum += pow(lower[j][k], 2);
				lower[j][j] = sqrt(matrix[j][j] -
										sum);
			} else {

				// Evaluating L(i, j) using L(j, j)
				for (int k = 0; k < j; k++)
					sum += (lower[i][k] * lower[j][k]);
				lower[i][j] = (matrix[i][j] - sum) /
									  lower[j][j];
			}
		}
	}

	// Displaying Lower Triangular and its Transpose
	cout << setw(6) << " Lower Triangular"
		 << setw(30) << "Transpose" << endl;
	for (int i = 0; i < n; i++) {

		// Lower Triangular
		for (int j = 0; j < n; j++)
			cout << setw(6) << lower[i][j] << "\t";
		cout << "\t";

		// Transpose of Lower Triangular
		for (int j = 0; j < n; j++)
			cout << setw(6) << lower[j][i] << "\t";
		cout << endl;
	}
}
	*/

	//Generate sigma points


}
