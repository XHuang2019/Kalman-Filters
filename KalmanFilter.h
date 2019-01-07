/** Discrete-time standard Kalman Filter for simple linear system expressed in state space form
* @original author: Hayk Martirosyan https://github.com/hmartiro/kalman-cpp.git
* @revised by Xiaoyu Huang on 2019.01.05
Future modification:
1. check observability
2. add B matrix
*/
#include <Eigen/Dense>
#pragma once

class KalmanFilter {
/* Definition of Matrices
A - State matrix
B - Input matrix
C - Output matrix
Q - Process noise covariance
R - Measurement noise covariance
P - Estimate error covariance
*/
private:

	// Matrices for computation
	Eigen::MatrixXd A, C, Q, R, P, K, P0;
	// System dimensions
	int n; // number of states
	int m; // number of measurements

	// Initial and current time
	double t0, t;
	// Discrete time step
	double dt;

	bool initialized;

	// n-size identity (why needed?)
	Eigen::MatrixXd I;

	// Estimated states
	Eigen::VectorXd x_hat, x_hat_new;


public:

	// Constructor 1
	KalmanFilter(
		double dt,
		const Eigen::MatrixXd& A,
		const Eigen::MatrixXd& C,
		const Eigen::MatrixXd& Q,
		const Eigen::MatrixXd& R,
		const Eigen::MatrixXd& P
	);
	
	// Constructor 2:  blank filter.
	KalmanFilter();
	
	// Initialize the filter with initial states as zero.
	void init();

	// Initialize the filter with a guess for initial states.
	void init(double t0, const Eigen::VectorXd& x0);

	// Update the estimated state based on measured values.The time step is assumed to remain constant.
	void update(const Eigen::VectorXd& y);

	// Update the estimated state based on measured values using the given time step and dynamics matrix (why needed?)
	void update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A);

	// Return the current state and time.
	Eigen::VectorXd state() { return x_hat; };
	double time() { return t; };
};
