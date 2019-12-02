#include <iostream>
#include <cmath>
#include "Eigen/Dense"
#include "EKF.h"
#include "tools.h"

EKF::~EKF() {}

void EKF::processMeasurement(const MeasurementPackage &measurement_pack)
{
  readMeasurementPackage(measurement_pack);
  if (!is_initialized_)
  {
    initializeEKF();
    return;
  }

  updateF();
  updateQ();
  ekf_.predict();

  if (sensor_type_ == MeasurementPackage::RADAR)
  {
    updateJacobianH();
    updateR(measurement_noise_cov_radar_);

    ekf_.updateEKF(measurements_);
  }
  else
  {
    updateH(measurement_matrix_laser_);
    updateR(measurement_noise_cov_laser_);
    ekf_.update(measurements_);
  }
}

void EKF::readMeasurementPackage(const MeasurementPackage &measurement_pack)
{
  measurements_ = measurement_pack.raw_measurements_;
  sensor_type_ = measurement_pack.sensor_type_;
  long long new_timestamp = measurement_pack.timestamp_;
  dt_ = (new_timestamp - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = new_timestamp;
}

void EKF::initializeEKF()
{
  initX();
  initP();
  is_initialized_ = true;
}

void EKF::initX()
{
  Eigen::VectorXd x_init(4);
  x_init << 0, 0, 0, 0;

  if (sensor_type_ == MeasurementPackage::RADAR)
  {
    const double ro = measurements_[0];
    const double theta = measurements_[1];
    x_init[0] = ro * cos(theta);
    x_init[1] = ro * sin(theta);
  }
  else if (sensor_type_ == MeasurementPackage::LASER)
  {
    x_init[0] = measurements_[0];
    x_init[1] = measurements_[1];
  }

  ekf_.setX(x_init);
}

void EKF::initP()
{
  Eigen::MatrixXd P_init(4, 4);
  P_init << 1., 0, 0, 0,
      0, 1., 0, 0,
      0, 0, 1000., 0,
      0, 0, 0, 1000.;
  ekf_.setP(P_init);
}

void EKF::updateF()
{
  Eigen::MatrixXd F_new(4, 4);
  F_new << 1, 0, dt_, 0,
      0, 1, 0, dt_,
      0, 0, 1, 0,
      0, 0, 0, 1;

  ekf_.setF(F_new);
}

void EKF::updateQ()
{
  Eigen::MatrixXd Q_new(4, 4);
  const double dt_2 = dt_ * dt_;
  const double dt_3 = dt_2 * dt_;
  const double dt_4 = dt_3 * dt_;
  double noise_ax = noise_(0);
  double noise_ay = noise_(1);
  Q_new << dt_4 * noise_ax / 4, 0, dt_3 * noise_ax / 2, 0,
      0, dt_4 * noise_ay / 4, 0, dt_3 * noise_ay / 2,
      dt_3 * noise_ax / 2, 0, dt_2 * noise_ax, 0,
      0, dt_3 * noise_ay / 2, 0, dt_2 * noise_ay;

  ekf_.setQ(Q_new);
}

void EKF::updateJacobianH()
{
  ekf_.setJacobianH();
}

void EKF::updateH(const Eigen::MatrixXd &H_new)
{
  ekf_.setH(H_new);
}

void EKF::updateR(const Eigen::MatrixXd &R_new)
{
  ekf_.setR(R_new);
}

void EKF::printOutput()
{
  std::cout << "x_ = " << ekf_.getX() << std::endl;
  std::cout << "P_ = " << ekf_.getP() << std::endl;
}
