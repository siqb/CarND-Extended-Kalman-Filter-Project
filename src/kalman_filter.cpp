#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
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
  P_ = P_ - K * H_*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];
  float rho = sqrt(px*px + py*py);
  float theta = atan2(py,px);
  float rho_dot = (px*vx + py*vy)/rho;

  // When rho is 0 skip update.
  if(fabs(rho) < .0001){
    return;
  }
  VectorXd h_x(3);
  h_x << rho, theta, rho_dot;

  VectorXd y = z - h_x;
  
  // Normalize angle
  y(1) = atan2(sin(y(1)), cos(y(1)));
  
  // Another method to normalize
  //while (y(1) > M_PI) {
  //  y(1) -= 2*M_PI;
  //}
  //while (y(1) < -M_PI) {
  //  y(1) += 2*M_PI;
  //}

  MatrixXd S = H_*P_*H_.transpose() + R_;
  MatrixXd Sinv = S.inverse();
  MatrixXd K =  P_*H_.transpose()*Sinv;

  // new state
  x_ = x_ + K*y;
  P_ = P_ - K*H_*P_;
}
