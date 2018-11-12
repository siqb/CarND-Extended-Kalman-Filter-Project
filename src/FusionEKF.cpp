#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  // What's a covariance matrix?
  // OK - explanation here:
  // https://www.theanalysisfactor.com/covariance-matrices/
  // https://medium.com/@adamzerner/covariance-and-correlation-d4c64769d4f1

  // Overall idea is that it shows
  //

  // a matrix whose element in the i, j position is the covariance between the i-th and j-th elements 

  // How are these value derived? 
  // Do they need to be tuned?
  
  // P is called measurement covariance matrix

  
  // Q is called the process/noise covariance matrix.
  // It is asscoiated with uncertainty due to influences we don't
  // keep track of (enviornmanetal - random wind, bumps, etc.)

  // New uncertainty is predited from the old uncertainty with
  // some additional uncertainty from the enviornment
  //
  // Pk = Fk * Pk-1 * (Fk)^T + Qk

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
    KalmanFilter ekf_;

	//create a 4D state vector, we don't know yet the values of the x state
	ekf_.x_ = VectorXd(4);

	//state covariance matrix P
	ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ << 1, 0, 0, 0,
			  0, 1, 0, 0,
			  0, 0, 1000, 0,
			  0, 0, 0, 1000;


	//measurement covariance
	ekf_.R_ = MatrixXd(2, 2);
	ekf_.R_ << 0.0225, 0,
			  0, 0.0225;

	//measurement matrix
	ekf_.H_ = MatrixXd(2, 4);
	ekf_.H_ << 1, 0, 0, 0,
			  0, 1, 0, 0;

	//the initial transition matrix F_
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, 1, 0,
			  0, 1, 0, 1,
			  0, 0, 1, 0,
			  0, 0, 0, 1;

	//set the acceleration noise components
	noise_ax = 5;
	noise_ay = 5;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  // Next, update the prediction with a measurement
  // This step basically multiplies the Gaussian of
  // the prediction with the Gaussian of the measurement
  // which produces a smaller, more certain Gaussian.
  // The updated position becomes the input to the next
  // prediction!

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar measurement updates
    // Only radar requires an EKF due to non-linearity
	ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser measurement updates
    // Laser uses a regular KF due to linearity
	ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
