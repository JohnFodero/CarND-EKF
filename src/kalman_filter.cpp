#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

constexpr double PI = 3.14159265358979323846;
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
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
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

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
   VectorXd z_pred = VectorXd(3);
   float px = x_[0];
   float py = x_[1];
   float vx = x_[2];
   float vy = x_[3];
   float combined = pow(pow(px, 2) + pow(py, 2), 0.5);
   if (combined < 0.00001) {
     std::cout << "DIV BY 0" << std::endl;
   }
   z_pred << combined, std::atan2(py,px), (px*vx + py*vy)/combined;
   VectorXd y = z - z_pred;

   // normalize angle
   while (y(1) < -PI) {
     y(1) += PI;
   }
   while (y(1) > PI) {
     y(1) -= PI;
   }

   MatrixXd Ht = H_.transpose();
   MatrixXd S = H_*P_*Ht + R_;
   MatrixXd Si = S.inverse();
   MatrixXd PHt = P_ * Ht;
   MatrixXd K = PHt * Si;

   x_ = x_ + (K * y);
   long x_size = x_.size();
   MatrixXd I = MatrixXd::Identity(x_size, x_size);
   P_ = (I - K * H_) * P_;
}
