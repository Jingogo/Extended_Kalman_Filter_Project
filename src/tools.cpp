#include "tools.h"
#include <iostream>
#include <cmath>
#include <stdlib.h>

using std::vector;

namespace tools
{
Eigen::VectorXd calculateRMSE(const vector<Eigen::VectorXd> &estimations,
                                     const vector<Eigen::VectorXd> &ground_truth)
{
  if (estimations.empty() or ground_truth.empty())
  {
    throw "Empty estimations or groud truth.";
  }
  auto estimations_begin = estimations.begin();
  auto ground_truth_begin = ground_truth.begin();
  
  Eigen::VectorXd RMSE(4);
  RMSE << 0,0,0,0;
  for (; estimations_begin != estimations.cend(); estimations_begin++, ground_truth_begin++)
  {
    Eigen::VectorXd diff = *estimations_begin - *ground_truth_begin;
    Eigen::VectorXd MSE = (diff.array() * diff.array());
    RMSE += MSE;
  }

  const auto estimations_size = estimations.size();
  RMSE = RMSE/estimations_size;
  RMSE = RMSE.array().sqrt();
  return RMSE;
}

double addEpsIfZero(double div)
{
  if (div < 0.00001)
  {
    div += 0.00001 ;
  }
  return div;
}

double normalizeTheta(double theta)
{
  if (theta > M_PI)
  {
    theta -= 2*M_PI;
  }

  if (theta < -M_PI) 
  {
    theta += 2*M_PI;
  }

  return theta;
}

MeasurementPackage readMeasurement(std::istringstream &iss)
{
  std::string sensor_type;
  iss >> sensor_type;
  MeasurementPackage meas_package;

  if (sensor_type.compare("L") == 0){
    meas_package.sensor_type_ = MeasurementPackage::LASER;
    meas_package.raw_measurements_ = Eigen::VectorXd(2);
    float px, py;
    iss >> px >> py;
    meas_package.raw_measurements_ << px, py;
  }
  else if (sensor_type.compare("R") == 0)
  {
    meas_package.sensor_type_ = MeasurementPackage::RADAR;
    meas_package.raw_measurements_ = Eigen::VectorXd(3);
    float ro, theta, ro_dot;
    iss >> ro >> theta >> ro_dot;
    meas_package.raw_measurements_ << ro, theta, ro_dot;
  }

  long long timestamp;
  iss >> timestamp;
  meas_package.timestamp_ = timestamp;

  return meas_package;
}

Eigen::VectorXd readGroundTruth(std::istringstream &iss)
{
  float x_gt, y_gt, vx_gt, vy_gt;
  iss >> x_gt >> y_gt >> vx_gt >> vy_gt;
  Eigen::VectorXd gt_values(4);
  gt_values << x_gt, y_gt, vx_gt, vy_gt;
  return gt_values;
}

} // namespace tools
