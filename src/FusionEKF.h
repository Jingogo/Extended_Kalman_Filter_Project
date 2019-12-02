#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"

class FusionEKF 
{
public:
  FusionEKF(Eigen::MatrixXd R_laser, Eigen::MatrixXd R_radar, Eigen::MatrixXd H_laser,
            Eigen::VectorXd noise): measurement_noise_cov_laser_(R_laser),
            measurement_noise_cov_radar_(R_radar), 
            measurement_matrix_laser_(H_laser), noise_(noise){};
  virtual ~FusionEKF();

  void processMeasurement(const MeasurementPackage &measurement_pack);
  void readMeasurementPackage(const MeasurementPackage &measurement_pack);
  void initializeFusionEKF();
  void initX();
  void initP();
  void updateF();
  void updateQ();  
  void updateJacobianH();
  void updateH(const Eigen::MatrixXd &H_new);
  void updateR(const Eigen::MatrixXd &R_new);
  void printOutput();

  KalmanFilter ekf_;


 private:
  bool is_initialized_ = false;
  long long previous_timestamp_ = 0;
  Eigen::VectorXd measurements_;
  double dt_ = 0.0;
  MeasurementPackage::SensorType sensor_type_ = MeasurementPackage::RADAR;
  Eigen::MatrixXd measurement_noise_cov_laser_;
  Eigen::MatrixXd measurement_noise_cov_radar_;
  Eigen::MatrixXd measurement_matrix_laser_;
  Eigen::VectorXd noise_;
};

#endif // FusionEKF_H_
