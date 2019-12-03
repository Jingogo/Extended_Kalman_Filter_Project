#include "kalman_filter.h"
#include "tools.h"
#include <iostream>
/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::predict()
{
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::VectorXd &z)
{
  Eigen::MatrixXd Ht = H_.transpose();
  Eigen::MatrixXd S = (H_ * P_) * Ht + R_;
  Eigen::MatrixXd K = P_ * Ht * S.inverse() ;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(4, 4);
  x_ = x_ + K * (z - H_ * x_);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::updateEKF(const Eigen::VectorXd &z)
{
  Eigen::MatrixXd Ht = H_.transpose();
  Eigen::MatrixXd S = (H_ * P_) * Ht + R_;
  Eigen::MatrixXd K = P_ * Ht * S.inverse();
  std::cout<<x_<<'\n' <<K <<'\n'<<std::endl;
  Eigen::VectorXd z_pred = calculateZPred();
  Eigen::VectorXd y = z - z_pred;
  double theta = y[1];
  theta = tools::normalizeTheta(theta);
  y[1] = theta;
  x_ = x_ + K * y;
  std::cout<<x_<<'\n' <<z <<'\n'<<z_pred<<std::endl;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(4, 4);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::setX(const Eigen::VectorXd &x_in)
{
  x_ = x_in;
}

void KalmanFilter::setP(const Eigen::MatrixXd &P_in)
{
  P_ = P_in;
}

void KalmanFilter::setF(const Eigen::MatrixXd &F_in)
{
  F_ = F_in;
}

void KalmanFilter::setQ(const Eigen::MatrixXd &Q_in)
{
  Q_ = Q_in;
}

void KalmanFilter::setH(const Eigen::MatrixXd &H_in)
{
  H_ = H_in;
}

void KalmanFilter::setJacobianH()
{
  H_ = calculateJacobian();
}

Eigen::MatrixXd KalmanFilter::calculateJacobian()
{
  double px = x_[0];
  double py = x_[1];
  double vx = x_[2];
  double vy = x_[3];
  double z = std::pow(px, 2) + std::pow(py, 2);
  z = tools::addEpsIfZero(z);
  double a = (vx * py - vy * px) / std::pow(z, 1.5);
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(3, 4);

  jacobian << px / sqrt(z), py / sqrt(z), 0, 0,
      -py / z, px / z, 0, 0,
      py * a, -px * a, px / sqrt(z), py / sqrt(z);

  return jacobian;
}

Eigen::VectorXd KalmanFilter::calculateZPred()
{
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);
  double z = std::pow(px, 2) + std::pow(py, 2);
  z = tools::addEpsIfZero(z);
  double rho = std::sqrt(z);
  double theta = std::atan2(py, px);
  double rho_dot = (px * vx + py * vy) / rho;

  Eigen::VectorXd z_pred(3);
  z_pred << rho, theta, rho_dot;

  return z_pred;
}

void KalmanFilter::setR(const Eigen::MatrixXd &R_in)
{
  R_ = R_in;
}

Eigen::VectorXd KalmanFilter::getX() const
{
  return x_;
}

Eigen::MatrixXd KalmanFilter::getP() const
{
  return P_;
}

Eigen::MatrixXd KalmanFilter::getF() const
{
  return F_;
}

Eigen::MatrixXd KalmanFilter::getQ() const
{
  return Q_;
}

Eigen::MatrixXd KalmanFilter::getH() const
{
  return H_;
}

Eigen::MatrixXd KalmanFilter::getR() const
{
  return R_;
}