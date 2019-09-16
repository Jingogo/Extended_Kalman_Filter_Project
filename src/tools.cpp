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

} // namespace tools
