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
    double dt,
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P)
  : A(A), C(C), Q(Q), R(R), P0(P),
    m(C.rows()), n(A.rows()), dt(dt), initialized(false),
    I(n, n), x_hat(n), x_hat_new(n)
{
	I.setIdentity();
}

// Constructor 2
KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0) {
	x_hat = x0;
	P = P0;
	this->t0 = t0;
	t = t0;
	initialized = true;
}

void KalmanFilter::init() {
	x_hat.setZero();
	P = P0;
	t0 = 0;
	t = t0;
	initialized = true;
}

void KalmanFilter::update(const Eigen::VectorXd& y) { // y is actual measurement and is not being updated

	if (!initialized) {
		throw std::runtime_error("Filter is not initialized!");
	}

	// Time update
	x_hat_new = A * x_hat; // a priori state estimate; missing B * U
	P = A * P * A.transpose() + Q; // a priori error covariance estimate

	// Measurement update
	K = P * C.transpose() * (C * P * C.transpose() + R).inverse(); // Kalman gain
	x_hat_new += K * (y - C*x_hat_new); // a posteriori state estimate
	P = (I - K*C)*P; // a posteriori error covariance estimate

	// Assign back to x_hat for use in the next instance
	x_hat = x_hat_new;

	// Record t, mainly used for plotting
	t += dt;
}

void KalmanFilter::update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A) {

	// Time-varing A and dt?
	this->A = A;
	this->dt = dt;
	update(y);
}
