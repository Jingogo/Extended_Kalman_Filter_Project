#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

namespace tools
{
Eigen::VectorXd calculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                              const std::vector<Eigen::VectorXd> &ground_truth);
double addEpsIfZero(double div);
double normalizeTheta(double theta);
} // namespace tools

#endif // TOOLS_H_
