#include "kalman_filter.h"
#include <math.h>
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  //predict equations
  x_ = F_*x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_*P_*Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
   VectorXd y = z - H_*x_;
  this->y_update(y);
  }
void KalmanFilter::UpdateEKF(const VectorXd &z) 
{
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  //calculating the z_pred by converting the given cartesian state to polar 
  VectorXd hx = VectorXd(3);
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);
  double rho = sqrt(px*px + py*py);
  if(rho<0.0001)
  {
    rho  = 0.0001;
  }
  double fi = atan2(py,px);
  double rho_rate = (px*vx + py*vy)/rho;
  hx << rho,fi,rho_rate;
  VectorXd y = z - hx;
  //normalizing the angle to be in the range of -pi to pi
  //we continusouly subtract the pi or add -pi from the y(1) for making it lie inside the range
  while (y(1)> M_PI) 
  {
    y(1)-=2.*M_PI;
  }
  while (y(1)<-M_PI) 
  {
    y(1)+=2.*M_PI;
  }
  if(y(1)<=M_PI && y(1)>=-M_PI)
  {
    std::cout<<"In range"<<y(1)<<std::endl;
  }
  this->y_update(y);
}
void KalmanFilter::y_update(const VectorXd& y)
{
	// here we update the state predicted this is the optimal estimate based on the evidence from the sensor measurement
  //calculating y
  //this needs us to calculate the matrices P_*H_.transpose()/H_*P_*H_.transpose() + R_
  MatrixXd HT = H_.transpose();
  MatrixXd s = H_*P_*HT + R_;
  MatrixXd si = s.inverse();
  MatrixXd K = P_*HT*si; // kalman gain
  x_ = x_ + K*y; // updating the state based on the kalman gain weighing , it weights based on the relative uncertainity between estimat and measurement
  //updating the previous process covariance matrix 
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  
}
