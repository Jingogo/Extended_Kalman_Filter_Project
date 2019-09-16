#include "gtest/gtest.h"
#include "tools.h"
#include "kalman_filter.h"
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

TEST(CalculateRMSETests, TestRMSECalulation)
{
    VectorXd estimate(4);
    estimate[0] = 2;
    estimate[1] = 3;
    estimate[2] = 1;
    estimate[3] = 5;    
    vector<VectorXd> estimations;
    estimations.push_back(estimate);

    VectorXd gt_values(4);
    gt_values(0) = 5;
    gt_values(1) = 4;
    gt_values(2) = 1;
    gt_values(3) = 2;
    vector<VectorXd> ground_truth;
    ground_truth.push_back(gt_values);

    VectorXd RMSE(4);
    RMSE(0) = 3;
    RMSE(1) = 1;
    RMSE(2) = 0;
    RMSE(3) = 3;

    EXPECT_EQ(RMSE, tools::calculateRMSE(estimations, ground_truth));
}


// TEST(CalculateRMSETests, TestRMSECalulationWithEmptyVector)
// {    
//     vector<VectorXd> estimations;
//     vector<VectorXd> ground_truth;

//     Tools t;
//     t.CalculateRMSE(estimations, ground_truth);
// }

TEST(CalculateJacobianTests, TestJacobianCalulation)
{
    VectorXd x_state(4);
    x_state(0) = 3.0;
    x_state(1) = 4.0;
    x_state(2) = 1.0;
    x_state(3) = 1.0;

    MatrixXd jacobian(3,4);
    jacobian(0,0) = 0.6;
    jacobian(0,1) = 0.8;
    jacobian(0,2) = 0;
    jacobian(0,3) = 0;
    jacobian(1,0) = -4/25.;
    jacobian(1,1) = 3/25.;
    jacobian(1,2) = 0;
    jacobian(1,3) = 0;
    jacobian(2,0) = 4/125.;
    jacobian(2,1) = -3/125.;
    jacobian(2,2) = 0.6;
    jacobian(2,3) = 0.8;
    
    KalmanFilter ekf;
    ekf.setX(x_state);
    EXPECT_EQ(jacobian, ekf.calculateJacobian());
}