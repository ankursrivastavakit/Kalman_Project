/*
Author: Ankur Srivastava
Kalman filter implementation for target tracking
*/

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include "extended_kf.h"


using namespace Eigen;
using namespace std;

//Function Declarations
void create_trajectory(double& t, double& max_v, int& iterations, vector<vector<double>>& result, double& start_x, double& start_y);
void create_measurements(bool& noise, double& sig_noise, vector<vector<double>>& trajectory, vector<vector<double>>& result, const MatrixXd& P);



int main() {

	// Parameters 
	double t = 0.1; // time step
	double max_v = 5; // max velocity
	bool noise = 1; // noise on/off
	double sigw = 3.0; // White noise value for NCA model
	int iterations = 200; // number of iterations
	double sig_noise = 0.1; // sigma of measurement noise
	int n = 6; //number of states
	int m = 4; //number of measurements per time step

	// Positions of the four anchors/satellites (meters)
	Vector2d A1(5, 5), A2(100, 5), A3(100, 100), A4(5
		, 100);
	MatrixXd P(2,4);
	P << A1, A2, A3, A4;

	// User input for starting x and y positions
	double start_x, start_y;
	cout << "Please enter a starting x value (double -50 to 50): " << endl;
	cin >> start_x;
	cout << "Please enter a starting y value (double -50 to 50):" << endl;
	cin >> start_y;

	//Storage for trajectory and for measurements
	vector<vector<double>> trajectory(iterations, vector<double> (2));
	vector<vector<double>> measurements(iterations, vector<double>(4));

	//Generating trajectory and measurement from the satellites
	create_trajectory(t, max_v, iterations, trajectory, start_x, start_y);
	create_measurements(noise, sig_noise, trajectory, measurements, P);

	MatrixXd A(n, n); //System matrix
	MatrixXd C(n, n); //State covariance 
	MatrixXd Q(n, n); // Process noise covariance
	MatrixXd R(m,m); // Measurement noise covariance
	MatrixXd B(n, 2); // Input (for calculating Q)
	Vector4d z_hat(0,0,0,0); //Measurement vector

	//For nearly constant accleration (NCA)
	A << 1, 0, t, 0, pow(t,2)* 0.5, 0,
		0, 1, 0, t, 0, pow(t, 2) * 0.5,
		0, 0, 1, 0, t, 0,
		0, 0, 0, 1, 0, t,
		0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 1;

	B << pow(t, 2) * 0.5, 0,
		0, pow(t, 2) * 0.5,
		t, 0,
		0, t,
		1, 0,
		0, 1;

	R = MatrixXd::Identity(m, m) * sig_noise;

	// A sensible initial covariance
	C << 10, 0, 0, 0, 0, 0,
		0, 10, 0, 0, 0, 0,
		0, 0, 5, 0, 0, 0,
		0, 0, 0, 5, 0, 0,
		0, 0, 0, 0, 3, 0,
		0, 0, 0, 0, 0, 3;

	// Calculating the NCA model value
	Q = B * sigw * B.transpose();

	//Initializing the EKF
    extended_kf kf(A, C, R, Q, P);
	kf.init();
	
	//Feeding measurements into the EKF
	for (int i = 0; i < iterations; i++) {
		for (int j = 0; j < 4; j++) {
			z_hat(j) = measurements[i][j];
		}
		kf.update(z_hat);
	}

	//Reading the final position from the EKF
	VectorXd EKF_result;
	EKF_result = kf.state();

	//Console output. Comparing the final position to the ground truth value.
	cout << "After " << iterations << " iterations:" << endl;
	cout << "EKF X Pos: " << EKF_result(0) << " m EKF Y Pos: " << EKF_result(1) << " m" << endl;
	cout << "True X Pos: " << trajectory[iterations-1][0] << " m True Y Pos: " << trajectory[iterations-1][1] << " m" << endl;
	system("pause");
}



/* Function to create a normal S-shaped trajectory. Can be expanded in the future
	for various modes (constant speed, constant acceleration, non-constant acceleration)
	*/
void create_trajectory(double& t, double& max_v, int& iterations, vector<vector<double>>& result, double& start_x, double& start_y) {

	// Trajectory for the initial time step
	result[0][0] = start_x + max_v * t;
	result[0][1] = start_y;

	// Loop for the first bend of the S Trajectory (increasing Y velocity)
	for (int i = 1; i < iterations / 2; i++) {

		result[i][0] = result[i - 1][0] + max_v * t;
		result[i][1] = result[i - 1][1] + max_v * t * (i / (iterations / 2.0));
	}

	// Loop for the second bend of the S Trajectory (decreasing Y velocity)
	for (int i = iterations / 2; i < iterations; i++) {

		result[i][0] = result[i - 1][0] + max_v * t;
		result[i][1] = result[i - 1][1] + max_v * t * (1.0 - (i / iterations));
	}
}

//Function for generating measurements
void create_measurements(bool& noise, double& sig_noise, vector<vector<double>>& trajectory, vector<vector<double>>& result, const MatrixXd& P) {

	default_random_engine generator;
	double value;

	for (int i = 0; i < trajectory.size(); i++) {
		for (int j = 0; j < 4; j++) {
			value = sqrt(pow(P(0, j) - trajectory[i][0], 2) + pow((P(1, j) - trajectory[i][1]), 2));
			if (noise == 1) {
				normal_distribution<double> distribution(value, sig_noise);
				result[i][j] = distribution(generator);
			}
			else
			{
				result[i][j] = value;
			}
		}
	}
}