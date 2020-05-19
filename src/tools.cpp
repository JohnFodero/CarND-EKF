#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */

  VectorXd rmse(4);
  rmse << 0,0,0,0;

  if((estimations.size() == 0) || (estimations.size() != ground_truth.size())) {
      return rmse;
  }
  for (int i=0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  rmse = rmse/estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
   MatrixXd Hj(3,4);

  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  if ((px == 0) && (py == 0)) {
      cout << "Error - divide by zero!" << endl;
      return Hj;
  }
  float denom = (px*px) + (py*py);
  Hj(0, 0) = px/pow(denom, 0.5);
  Hj(0, 1) = py/pow(denom, 0.5);
  Hj(1, 0) = (-1*py)/denom;
  Hj(1, 1) = px/denom;
  Hj(2, 0) = py*(vx*py-vy*px)/pow(denom, 1.5);
  Hj(2, 1) = px*(vy*px-vx*py)/pow(denom, 1.5);
  Hj(2, 2) = px/pow(denom, 0.5);
  Hj(2, 3) = py/pow(denom, 0.5);
  return Hj;
}
