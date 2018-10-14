/*** Quiz: Kalman Filter Equations in C++ ***/

// Write a function 'filter()' that implements a multi-
// dimensional Kalman Filter for the example given
//============================================================================
#include <iostream>
#include "Dense"
#include <vector>

using namespace std;
using namespace Eigen;

//Kalman Filter variables
VectorXd x;	// object state
MatrixXd P;	// object covariance matrix
VectorXd u;	// external motion
MatrixXd F; // state transition matrix
MatrixXd H;	// measurement matrix
MatrixXd R;	// measurement covariance matrix
MatrixXd I; // Identity matrix
MatrixXd Q;	// process covariance matrix

vector<VectorXd> measurements;
void filter(VectorXd &x, MatrixXd &P);


int main() {
	/**
	 * Code used as example to work with Eigen matrices
	 */
//	//you can create a  vertical vector of two elements with a command like this
//	VectorXd my_vector(2);
//	//you can use the so called comma initializer to set all the coefficients to some values
//	my_vector << 10, 20;
//
//
//	//and you can use the cout command to print out the vector
//	cout << my_vector << endl;
//
//
//	//the matrices can be created in the same way.
//	//For example, This is an initialization of a 2 by 2 matrix
//	//with the values 1, 2, 3, and 4
//	MatrixXd my_matrix(2,2);
//	my_matrix << 1, 2,
//			3, 4;
//	cout << my_matrix << endl;
//
//
//	//you can use the same comma initializer or you can set each matrix value explicitly
//	// For example that's how we can change the matrix elements in the second row
//	my_matrix(1,0) = 11;    //second row, first column
//	my_matrix(1,1) = 12;    //second row, second column
//	cout << my_matrix << endl;
//
//
//	//Also, you can compute the transpose of a matrix with the following command
//	MatrixXd my_matrix_t = my_matrix.transpose();
//	cout << my_matrix_t << endl;
//
//
//	//And here is how you can get the matrix inverse
//	MatrixXd my_matrix_i = my_matrix.inverse();
//	cout << my_matrix_i << endl;
//
//
//	//For multiplying the matrix m with the vector b you can write this in one line as let’s say matrix c equals m times v.
//	//
//	MatrixXd another_matrix;
//	another_matrix = my_matrix*my_vector;
//	cout << another_matrix << endl;


	//design the KF with 1D motion
	x = VectorXd(2);
	x << 0, 0;

	P = MatrixXd(2, 2);
	P << 1000, 0, 0, 1000;

	u = VectorXd(2);
	u << 0, 0;

	F = MatrixXd(2, 2);
	F << 1, 1, 0, 1;

	H = MatrixXd(1, 2);
	H << 1, 0;

	R = MatrixXd(1, 1);
	R << 1;

	I = MatrixXd::Identity(2, 2);

	Q = MatrixXd(2, 2);
	Q << 0, 0, 0, 0;

	//create a list of measurements
	VectorXd single_meas(1);
	single_meas << 1;
	measurements.push_back(single_meas);
	single_meas << 2;
	measurements.push_back(single_meas);
	single_meas << 3;
	measurements.push_back(single_meas);

	//call Kalman filter algorithm
	filter(x, P);

	return 0;

}


void filter(VectorXd &x, MatrixXd &P) {

	for (unsigned int n = 0; n < measurements.size(); ++n) {

		VectorXd z = measurements[n];
		//YOUR CODE HERE
		/*
		 * KF Measurement update step
		 */
		VectorXd y = z - H * x;
		MatrixXd Ht = H.transpose();
		MatrixXd S = H * P * Ht + R;
		MatrixXd Si = S.inverse();
		MatrixXd K =  P * Ht * Si;

		//new state
		x = x + (K * y);
		P = (I - K * H) * P;

		/*
		 * KF Prediction step
		 */
		x = F * x + u;
		MatrixXd Ft = F.transpose();
		P = F * P * Ft + Q;

		std::cout << "x=" << std::endl <<  x << std::endl;
		std::cout << "P=" << std::endl <<  P << std::endl;


	}
}


/*** Quiz: Laser Measurements ***/

#include "kalman_filter.h"

KalmanFilter::KalmanFilter() {
}

KalmanFilter::~KalmanFilter() {
}

void KalmanFilter::Predict() {
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}


#include "Dense"
#include <iostream>
#include "tracking.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tracking::Tracking() {
	is_initialized_ = false;
	previous_timestamp_ = 0;

	//create a 4D state vector, we don't know yet the values of the x state
	kf_.x_ = VectorXd(4);

	//state covariance matrix P
	kf_.P_ = MatrixXd(4, 4);
	kf_.P_ << 1, 0, 0, 0,
			  0, 1, 0, 0,
			  0, 0, 1000, 0,
			  0, 0, 0, 1000;


	//measurement covariance
	kf_.R_ = MatrixXd(2, 2);
	kf_.R_ << 0.0225, 0,
			  0, 0.0225;

	//measurement matrix
	kf_.H_ = MatrixXd(2, 4);
	kf_.H_ << 1, 0, 0, 0,
			  0, 1, 0, 0;

	//the initial transition matrix F_
	kf_.F_ = MatrixXd(4, 4);
	kf_.F_ << 1, 0, 1, 0,
			  0, 1, 0, 1,
			  0, 0, 1, 0,
			  0, 0, 0, 1;

	//set the acceleration noise components
	noise_ax = 5;
	noise_ay = 5;

}

Tracking::~Tracking() {

}

// Process a single measurement
void Tracking::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
	if (!is_initialized_) {
		//cout << "Kalman Filter Initialization " << endl;

		//set the state with the initial location and zero velocity
		kf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

		previous_timestamp_ = measurement_pack.timestamp_;
		is_initialized_ = true;
		return;
	}

	//compute the time elapsed between the current and previous measurements
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;

	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;

	//Modify the F matrix so that the time is integrated
	kf_.F_(0, 2) = dt;
	kf_.F_(1, 3) = dt;

	//set the process covariance matrix Q
	kf_.Q_ = MatrixXd(4, 4);
	kf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
			   0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
			   dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
			   0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

	//predict
	kf_.Predict();

	//measurement update
	kf_.Update(measurement_pack.raw_measurements_);

	std::cout << "x_= " << kf_.x_ << std::endl;
	std::cout << "P_= " << kf_.P_ << std::endl;

}


/*** Quiz: Jacobian Matrix ***/

#include <iostream>
#include "Dense"
#include <vector>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd CalculateJacobian(const VectorXd& x_state);

int main() {

	/*
	 * Compute the Jacobian Matrix
	 */

	//predicted state  example
	//px = 1, py = 2, vx = 0.2, vy = 0.4
	VectorXd x_predicted(4);
	x_predicted << 1, 2, 0.2, 0.4;

	MatrixXd Hj = CalculateJacobian(x_predicted);

	cout << "Hj:" << endl << Hj << endl;

	return 0;
}

MatrixXd CalculateJacobian(const VectorXd& x_state) {

	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//pre-compute a set of terms to avoid repeated calculation
	float c1 = px*px+py*py;
	float c2 = sqrt(c1);
	float c3 = (c1*c2);

	//check division by zero
	if(fabs(c1) < 0.0001){
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj;
	}

	//compute the Jacobian matrix
	Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

	return Hj;
}

/*** Quiz: Evaluating KF Performance ***/

#include <iostream>
#include "Dense"
#include <vector>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
		const vector<VectorXd> &ground_truth);

int main() {
	/*
	 * Compute RMSE
	 */
	vector<VectorXd> estimations;
	vector<VectorXd> ground_truth;

	//the input list of estimations
	VectorXd e(4);
	e << 1, 1, 0.2, 0.1;
	estimations.push_back(e);
	e << 2, 2, 0.3, 0.2;
	estimations.push_back(e);
	e << 3, 3, 0.4, 0.3;
	estimations.push_back(e);

	//the corresponding list of ground truth values
	VectorXd g(4);
	g << 1.1, 1.1, 0.3, 0.2;
	ground_truth.push_back(g);
	g << 2.1, 2.1, 0.4, 0.3;
	ground_truth.push_back(g);
	g << 3.1, 3.1, 0.5, 0.4;
	ground_truth.push_back(g);

	//call the CalculateRMSE and print out the result
	cout << CalculateRMSE(estimations, ground_truth) << endl;


	return 0;
}

VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
		const vector<VectorXd> &ground_truth){

	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}
