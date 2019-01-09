/** Discrete-time standard Kalman Filter for simple linear system expressed in state space form
* @original author: Hayk Martirosyan https://github.com/hmartiro/kalman-cpp.git
* @revised by Xiaoyu Huang on 2019.01.05
* Test for the KalmanFilter class with 1D projectile motion.
*/

#include "pch.h"
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "KalmanFilter.h"

int main() {

	int n = 3;	// Number of states
	int l = 1;	// Number of inputs
	int m = 1;	// Number of measurements

	double dt = 1.0/30; // Time step

	Eigen::MatrixXd A(n, n);	// State matrix
	Eigen::MatrixXd B(n, l);		// Input matrix
	Eigen::MatrixXd C(m, n);	// Output matrix
	Eigen::MatrixXd D(m, l);	// Feedforward matrix
	Eigen::MatrixXd Q(n, n);	// Process noise covariance
	Eigen::MatrixXd R(m, m);	// Measurement noise covariance
	Eigen::MatrixXd P(n, n);	// Estimate error covariance

	// Discrete LTI projectile motion, measuring position only
	A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
	B << 0, 0, 0;
	C << 1, 0, 0;
	D << 0;

	// Reasonable covariance matrices
	Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
	R << 5;
	P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;

	std::cout << "A: \n" << A << std::endl;
	std::cout << "B: \n" << B << std::endl;
	std::cout << "C: \n" << C << std::endl;
	std::cout << "D: \n" << D << std::endl;
	std::cout << "Q: \n" << Q << std::endl;
	std::cout << "R: \n" << R << std::endl;
	std::cout << "P: \n" << P << std::endl;

	// Construct the filter
	KalmanFilter kf(dt, A, B, C, D, Q, R, P);

	// List of noisy position measurements (y)
	std::vector<double> measurements = {
		1.04202710058, 1.10726790452, 1.2913511148, 1.48485250951, 1.72825901034,
		1.74216489744, 2.11672039768, 2.14529225112, 2.16029641405, 2.21269371128,
		2.57709350237, 2.6682215744, 2.51641839428, 2.76034056782, 2.88131780617,
		2.88373786518, 2.9448468727, 2.82866600131, 3.0006601946, 3.12920591669,
		2.858361783, 2.83808170354, 2.68975330958, 2.66533185589, 2.81613499531,
		2.81003612051, 2.88321849354, 2.69789264832, 2.4342229249, 2.23464791825,
		2.30278776224, 2.02069770395, 1.94393985809, 1.82498398739, 1.52526230354,
		1.86967808173, 1.18073207847, 1.10729605087, 0.916168349913, 0.678547664519,
		0.562381751596, 0.355468474885, -0.155607486619, -0.287198661013, -0.602973173813
	};
	std::vector<double> input = { 0 };

	// Best guess of initial states
	Eigen::VectorXd x0(n);
	x0 << measurements[0], 0, -9.81;
	kf.init(0, x0);

	// Feed measurements into filter, output estimated states
	double t = 0;
	Eigen::VectorXd y(m); 
	Eigen::VectorXd u(l);
	std::cout << "t = " << t << ", " << "x_hat[0]: " << kf.getStateEst().transpose() << std::endl;
	for (int i = 0; i < measurements.size(); ++i) 
	{
		t += dt;
		y << measurements[i];
		u << 0; // input is 0;
		kf.update(y, u);
		std::cout << "t = " << t << ", " << "\ty[" << i << "] = " << y.transpose()
		<< ", x_hat[" << i << "] = " << kf.getStateEst().transpose() << std::endl;
	}
  std::cout << "So far so good! 01/05/2019" << std::endl;
  return 0;
}
