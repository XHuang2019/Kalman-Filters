/** Discrete-time standard Kalman Filter for simple linear system expressed in state space form
* @original author: Hayk Martirosyan https://github.com/hmartiro/kalman-cpp.git
* @revised by Xiaoyu Huang 01/2019
Future modifications:
1. Check dimensions
2. Visualize results
*/

/* Linear State Space Model :
	X_dot = A * X + B * U + w
	Y = C * X + D * U + v
	X: states
	U: inputs
	Y: measurements
	w: process noise
	v: measurement noise

	Definition of Matrices
	A - State matrix
	B - Input matrix
	C - Output matrix
	D - Feedforward matrix
	Q - Process noise covariance
	R - Measurement noise covariance
	P - Estimate error covariance
*/

#include <Eigen/Dense>
#pragma once

class KalmanFilter {

public:

	// Constructor 1
	KalmanFilter(
		double Ts,
		const Eigen::MatrixXd& A,
		const Eigen::MatrixXd& B, 
		const Eigen::MatrixXd& C,
		const Eigen::MatrixXd& D, 
		const Eigen::MatrixXd& Q,
		const Eigen::MatrixXd& R
	);
	
	// Constructor 2:  blank filter (revise later)
	KalmanFilter();
	
	// Initialize the filter with zero states.
	void init();

	// Initialize the filter.
	void init(double t0, 
				 const Eigen::VectorXd& x0,
				 const Eigen::MatrixXd& P0);

	// Set state matrix A
	void setA(const Eigen::MatrixXd &A) { A_ = A; }

	// Set input matrix B
	void setB(const Eigen::MatrixXd &B) { B_ = B; }

	// Set output matrix C
	void setC(const Eigen::MatrixXd &C) { C_ = C; }

	// Set feedforward matrix D
	void setD(const Eigen::MatrixXd &D) { D_ = D; }

	// Set process noise covariance Q
	void setQ(const Eigen::MatrixXd &Q) { Q_ = Q; }

	// Set measurement noise covariance R
	void setR(const Eigen::MatrixXd &R) { R_ = R; }

	// Return state matrix A
	const Eigen::MatrixXd &getA() const { return A_; }

	// Return input matrix B
	const Eigen::MatrixXd &getB() const { return B_; }

	// Return output matrix C
	const Eigen::MatrixXd &getC() const { return C_; }

	// Return feedforward matrix D
	const Eigen::MatrixXd &getD() const { return D_; }

	// Return process noise covariance Q
	const Eigen::MatrixXd &getQ() const { return Q_; }

	// Return measurement noise covariance R
	const Eigen::MatrixXd &getR() const { return R_; }

	// Update the estimated state based on measured values.The time step is assumed to remain constant.
	void update(const Eigen::VectorXd& y,
					   const Eigen::VectorXd& u);

	// Return the current state estimate
	Eigen::VectorXd getStateEst() const { return x_aPost; }
	
	// Return the current state estimate
	Eigen::MatrixXd getErrorCov() const { return P_aPost; }

	// Return current time
	double getTime() { return t; }

private:
	// Sampling time
	double Ts_;

	// Dimensions
	int nX; // dimension of states (n)
	int nY; // number of measurements (m)
	int nU; // number of control inputs (l)

	// State matrix (nX by nX)
	Eigen::MatrixXd A_;
	
	// Input matrix (nX by nU)
	Eigen::MatrixXd B_;

	// Output matrix (nY by nX)
	Eigen::MatrixXd C_;
	
	// Feedforward matrix (nY by nU)
	Eigen::MatrixXd D_;

	// Process noise covariance (nX by nX)
	Eigen::MatrixXd Q_;

	// Measurement noise covariance (nY by nY)
	Eigen::MatrixXd R_;

	// Kalman gain (nX by nY)
	Eigen::MatrixXd K;

	// a-priori error covariance (nX by nX)
	Eigen::MatrixXd P_aPrio;
	// a-posteriori error covariance (nX by nX)
	Eigen::MatrixXd P_aPost;

	// a-priori state estimate
	Eigen::VectorXd x_aPrio;
	// a-posteriori state estimate
	Eigen::VectorXd x_aPost;
	
	// Initial time
	double t0;

	// Current time
	double t;

	// Denote whether KF has been initialized
	bool initialized = false;

	// n-size identity (needed?)
	Eigen::MatrixXd I;

};
