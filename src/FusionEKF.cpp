#include <iostream>
#include <cmath>
#include "Eigen/Dense"
#include "FusionEKF.h"
#include "tools.h"

FusionEKF::~FusionEKF() {}

void FusionEKF::processMeasurement(const MeasurementPackage &measurement_pack)
{
  Eigen::VectorXd raw_measurements = measurement_pack.raw_measurements_;
  MeasurementPackage::SensorType sensor_type = measurement_pack.sensor_type_;
  long long new_timestamp = measurement_pack.timestamp_;
  double dt = (new_timestamp - previous_timestamp_) / 1000000.0;

  if (!is_initialized_)
  {
    initX(raw_measurements, sensor_type);
    initP();
    updateTimestamp(new_timestamp);
    is_initialized_ = true;
    return;
  }

  updateF(dt);
  updateQ(dt);
  ekf_.predict();

  if (sensor_type == MeasurementPackage::RADAR)
  {
    updateJacobianH();
    updateR(R_radar_);
    ekf_.updateEKF(raw_measurements);
  }
  else
  {
    updateH(H_laser_);
    updateR(R_laser_);
    ekf_.update(raw_measurements);
  }

  updateTimestamp(new_timestamp);

  //printOutput();
}

void FusionEKF::initX(const Eigen::VectorXd raw_measurements,
                      const MeasurementPackage::SensorType &sensor_type)
{

  Eigen::VectorXd x_init(4);
  x_init << 0, 0, 0, 0;

  if (sensor_type == MeasurementPackage::RADAR)
  {
    // TODO: Convert radar from polar to cartesian coordinates
    //         and initialize state.
    const double  ro = raw_measurements[0];
    const double theta = raw_measurements[1];
    x_init[0] = ro * cos(theta);
    x_init[1] = ro * sin(theta);
  }
  else if (sensor_type == MeasurementPackage::LASER)
  {
    // TODO: Initialize state.
    x_init[0] = raw_measurements[0];
    x_init[1] = raw_measurements[1];
  }

  ekf_.setX(x_init);
}

void FusionEKF::initP()
{
  Eigen::MatrixXd P_init(4, 4);
  P_init << 1., 0, 0, 0,
      0, 1., 0, 0,
      0, 0, 1000., 0,
      0, 0, 0, 1000.;
  ekf_.setP(P_init);
}

void FusionEKF::updateF(const double &dt)
{
  Eigen::MatrixXd F_new(4, 4);
  F_new << 1, 0, dt, 0,
           0, 1, 0, dt,
           0, 0, 1, 0,
           0, 0, 0, 1;

  ekf_.setF(F_new);
}

void FusionEKF::updateQ(const double &dt)
{
  Eigen::MatrixXd Q_new(4, 4);
  const double dt_2 = dt * dt;
  const double dt_3 = dt_2 * dt;
  const double dt_4 = dt_3 * dt;
  double noise_ax = noise_(0);
  double noise_ay = noise_(1);
  Q_new << dt_4 * noise_ax / 4, 0, dt_3 * noise_ax / 2, 0,
           0, dt_4 * noise_ay / 4, 0, dt_3 * noise_ay / 2,
           dt_3 * noise_ax / 2, 0, dt_2 * noise_ax, 0,
           0, dt_3 * noise_ay / 2, 0, dt_2 * noise_ay;

  ekf_.setQ(Q_new);
}

void FusionEKF::updateJacobianH()
{
  ekf_.setJacobianH();
}

void FusionEKF::updateH(const Eigen::MatrixXd &H_new)
{
  ekf_.setH(H_new);
}

void FusionEKF::updateR(const Eigen::MatrixXd &R_new)
{
  ekf_.setR(R_new);
}

void FusionEKF::updateTimestamp(const long long &new_timestamp)
{
  previous_timestamp_ = new_timestamp;
}

void FusionEKF::printOutput()
{
  std::cout << "x_ = " << ekf_.getX() << std::endl;
  std::cout << "P_ = " << ekf_.getP() << std::endl;
}
