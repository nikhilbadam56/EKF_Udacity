#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <math.h>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  
  //extended kalman filter variables
  ekf_.Q_ = MatrixXd(4,4);
  ekf_.F_ = MatrixXd(4,4);
  ekf_.P_ = MatrixXd(4,4);

  H_laser_ << 1,0,0,0,
  			 0,1,0,0;

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  
  ekf_.F_ << 1,0,1,0,
  			 0,1,0,1,
  			 0,0,1,0,
  			 0,0,0,1;
  
  
  ekf_.P_ << 1, 0, 0, 0,
            0,1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

  
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
  
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      
      double rho = measurement_pack.raw_measurements_[0];
      double fi = measurement_pack.raw_measurements_[1];
      double rho_rate = measurement_pack.raw_measurements_[2];
      
      //calculating the px, py, vx, vy based on the rho , fi , rho_rate relations
      double px = rho*cos(fi);
      double py = rho*sin(fi);
      double vx = rho_rate*cos(fi);
      double vy = rho_rate*sin(fi);
     ekf_.x_ << px,py,0,0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      ekf_.x_<< measurement_pack.raw_measurements_[0],
      			measurement_pack.raw_measurements_[1],
      			0,
      			0;
    }
    
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
  long long dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;
  double noise_ax = 9.0;
  double noise_ay = 9.0;
  
  double dt2 = dt*dt;
  double dt3 = dt2*dt;
  double dt4 = dt3*dt;
  double dt4_4 = dt4/4;
  double dt3_2 = dt3/2;
  ekf_.Q_ << dt4_4 * noise_ax, 0 , dt3_2*noise_ax , 0 ,
  			 0  ,dt4_4 * noise_ay,               0,   dt3_2*noise_ay,
  			dt3_2*noise_ax, 0,dt2*noise_ax,0,
  			0,dt3_2*noise_ay,0,dt2*noise_ay;
  
  
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    ekf_.R_ = R_radar_;
    ekf_.H_  = tools.CalculateJacobian(ekf_.x_);
  	ekf_.Init(ekf_.x_,ekf_.P_,ekf_.F_,ekf_.H_,ekf_.R_,ekf_.Q_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } 
  else {
    // TODO: Laser updates
    //for laser we can directly use the r_laser and H_laser;
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Init(ekf_.x_,ekf_.P_,ekf_.F_,ekf_.H_,ekf_.R_,ekf_.Q_);
    ekf_.Update(measurement_pack.raw_measurements_);
  }
  // print the output
  cout<< "x_ = " << ekf_.x_ << endl;
  cout<< "P_ = " << ekf_.P_ << endl;
}
