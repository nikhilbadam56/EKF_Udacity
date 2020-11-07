#include "kalman_filter.h"
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
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_*P_*Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd y = z - H_*x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;    
  x_ = x_ + (K * y);//optimal estimate of the state based on the prediction and measurment covariance matrices decided by the kalman gain
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}
void KalmanFilter::UpdateEKF(const VectorXd &z) 
{
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  //calculating the z_pred by converting the given cartesian state to polar 
  VectorXd hx = VectorXd(3);
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  float rho = sqrt(px*px + py*py);
  
  if(rho<0.0001)
  {
    rho  = 0.0001;
  }
  
  float fi = atan2(py,px);
  float rho_rate = (px*vx+py*vy)/rho;

  hx << rho,fi,rho_rate;
  VectorXd y = z - hx;
  //normalizing the angle to be in the range of -pi to pi
  //we continusouly subtract the 2pi or add -2pi from the y(1) for making it lie inside the range
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
  
  //common steps
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
    
  x_ = x_ + (K * y);//optimal estimate of the state based on the prediction and measurment covariance matrices decided by the kalman gain
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}