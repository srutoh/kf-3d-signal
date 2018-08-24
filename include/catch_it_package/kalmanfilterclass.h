#ifndef KALMANFILTERCLASS_H
#define KALMANFILTERCLASS_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <Eigen/Core>

class KalmanFilterClass
{
  //X is state, with position, velocity, acceleration
  Eigen::Vector3d X;
  Eigen::RowVector3d H;
  Eigen::Vector3d K;
  Eigen::Matrix3d A;
  Eigen::Matrix3d P;
  Eigen::Matrix3d Q;
  double R;
  bool is_initialized;


public:
  KalmanFilterClass();

  void init(double x_init, Eigen::Matrix3d P0);
  void setR(double new_R);
  Eigen::Vector3d update(double z, double dt);
  void reset();
};

#endif // KALMANFILTERCLASS_H
