#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    std::cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  
  // calculating the jacobian based on the px,py linear relations
  MatrixXd Hj_ = MatrixXd(3,4);
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);
  double dist = px*px + py*py;
  if(dist<0.0001)
  {
    cout << "Distance Too Small ! exiting"<<endl;
    return Hj_;
  }
  double dist_sqrt = sqrt(dist);
  double dist_3_2 = dist_sqrt*dist;
 
  //jacobian
  Hj_<< px/dist_sqrt,py/dist_sqrt,0,0,
  		-py/dist, px/dist,0,0,
  		py*(vx*py - px*vy)/dist_3_2 , px*(vy*px - vx*py)/dist_3_2,px/dist_sqrt,py/dist_sqrt;
  return Hj_;
}
