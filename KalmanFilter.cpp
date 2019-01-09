/** Discrete-time standard Kalman Filter for simple linear system expressed in state space form
* @original author: Hayk Martirosyan https://github.com/hmartiro/kalman-cpp.git
* @revised by Xiaoyu Huang on 2019.01.05
* Implementation of KalmanFilter class.
*/

#include "pch.h"
#include <iostream>
#include <stdexcept>
#include "KalmanFilter.h"

// Constructor 1
KalmanFilter::KalmanFilter(
    double Ts,
    const Eigen::MatrixXd& A,
	const Eigen::MatrixXd& B,
	const Eigen::MatrixXd& C,
	const Eigen::MatrixXd& D, 
	const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R)
  : Ts_(Ts), A_(A), B_(B), C_(C), D_(D), Q_(Q), R_(R), 
	nX(A.rows()), nY(C.rows()), initialized(false),
	x_aPrio(nX), x_aPost(nX), I(nX, nX),
	P_aPrio(nX, nX), P_aPost(nX, nX), K(nX, nY)
{
	I.setIdentity();
}

// Constructor 2
KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(double t0, 
							const Eigen::VectorXd& x0,
							const Eigen::MatrixXd& P0)
{
	x_aPost = x0;
	P_aPost = P0;
	this->t0 = t0;
	t = t0;
	initialized = true;
}

void KalmanFilter::init() 
{
	x_aPost.setZero();
	P_aPost.setIdentity();
	t0 = 0;
	t = 0;
	initialized = true;
}

void KalmanFilter::update(const Eigen::VectorXd& y, // y is actual measurement and is not being updated
										const Eigen::VectorXd& u) 
{ 
	if (!initialized) {
		throw std::runtime_error("Filter is not initialized!");
	}

	// Time update (Predict)
	x_aPrio = A_ * x_aPost + B_ * u; // a-priori state estimate using a-posteriori estimate from last iteration
	P_aPrio = A_ * P_aPost * A_.transpose() + Q_; // a-priori error covariance estimate

	// Measurement update (Correct)
	K = P_aPrio * C_.transpose() * (C_ * P_aPrio * C_.transpose() + R_).inverse(); // Kalman gain
	x_aPost = x_aPrio + K * (y - C_ * x_aPrio - D_ * u); // a posteriori state estimate
	P_aPost = (I - K * C_) * P_aPrio; // a posteriori error covariance estimate

	// Record t, mainly used for plotting
	t += Ts_;
}