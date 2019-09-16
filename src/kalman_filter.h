#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter 
{
public:
  KalmanFilter();
  virtual ~KalmanFilter();
  
  void predict();
  void update(const Eigen::VectorXd &z);
  void updateEKF(const Eigen::VectorXd &z);
  
  void setX(const Eigen::VectorXd &x_in);
  void setP(const Eigen::MatrixXd &P_in);
  void setF(const Eigen::MatrixXd &F_in);
  void setQ(const Eigen::MatrixXd &Q_in);  
  void setH(const Eigen::MatrixXd &H_in);  
  void setJacobianH();  
  void setR(const Eigen::MatrixXd &R_in);
  Eigen::MatrixXd calculateJacobian();
  Eigen::VectorXd calculateZPred();

  Eigen::VectorXd getX() const;
  Eigen::MatrixXd getP() const;
  Eigen::MatrixXd getF() const;
  Eigen::MatrixXd getQ() const;
  Eigen::MatrixXd getH() const;
  Eigen::MatrixXd getR() const;


private:
  // state vector
  Eigen::VectorXd x_;
  // state covariance matrix
  Eigen::MatrixXd P_;
  // state transition matrix
  Eigen::MatrixXd F_;
  // process covariance matrix
  Eigen::MatrixXd Q_;
  // measurement matrix
  Eigen::MatrixXd H_;
  // measurement covariance matrix
  Eigen::MatrixXd R_;
};

#endif // KALMAN_FILTER_H_
