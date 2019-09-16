#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "FusionEKF.h"
#include "tools.h"
#include <fstream>
using std::string;
using std::vector;

int main(int argc, char *argv[])
{
  if (argc != 2)
  {
    std::cout << "The programsneed two arguments" << std::endl;
    return -1;
  }

  std::ifstream input_file(argv[1]);
  if (!input_file.is_open())
  {
    std::cout << "Could not open file" << argv[1] << std::endl;
  }

  FusionEKF fusionEKF;
  vector<Eigen::VectorXd> estimations;
  vector<Eigen::VectorXd> ground_truth;

  int cnt = 1;
  std::string line;
  while (std::getline(input_file, line))
  {
    std::cout << "iteration:" << cnt << std::endl;
    std::istringstream iss(line);
    std::string sensor_type;
    iss >> sensor_type;
    MeasurementPackage meas_package;

    if (sensor_type.compare("L") == 0)
    {
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = Eigen::VectorXd(2);
      float px;
      float py;
      iss >> px;
      iss >> py;
      meas_package.raw_measurements_ << px, py;
    }
    else if (sensor_type.compare("R") == 0)
    {
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = Eigen::VectorXd(3);
      float ro;
      float theta;
      float ro_dot;
      iss >> ro;
      iss >> theta;
      iss >> ro_dot;
      //theta = tools::normalizeTheta(theta);
      meas_package.raw_measurements_ << ro, theta, ro_dot;
    }

    long long timestamp;
    iss >> timestamp;
    meas_package.timestamp_ = timestamp;

    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;

    Eigen::VectorXd gt_values(4);
    gt_values(0) = x_gt;
    gt_values(1) = y_gt;
    gt_values(2) = vx_gt;
    gt_values(3) = vy_gt;
    ground_truth.push_back(gt_values);

    // Call ProcessMeasurement(meas_package) for Kalman filter


    if (cnt == 400)
    {
      std::cout<<cnt<<std::endl;
    }
    fusionEKF.processMeasurement(meas_package);
    

    // Push the current estimated x,y positon from the Kalman filter's
    //   state vector

    Eigen::VectorXd estimate(4);
    Eigen::VectorXd x = fusionEKF.ekf_.getX();
    double p_x = x(0);
    double p_y = x(1);
    double v1 = x(2);
    double v2 = x(3);
    
    estimate(0) = p_x;
    estimate(1) = p_y;
    estimate(2) = v1;
    estimate(3) = v2;

    estimations.push_back(estimate);
    Eigen::VectorXd RMSE = tools::calculateRMSE(estimations, ground_truth);
    cnt += 1;
    std::cout << "Estimation: x,y,vx,vy\n" << estimate << std::endl;
    std::cout << "ground_truth: x,y,vx,vy\n" << gt_values << std::endl;
    std::cout << "RMSE: x,y,vx,vy\n" << RMSE << std::endl;
  }
}