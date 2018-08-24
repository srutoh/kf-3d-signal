#include <catch_it_package/kalmanfilterclass.h>

KalmanFilterClass::KalmanFilterClass()
{
  is_initialized = false;
  X << 0,0,0;
  H << 1,0,0;
  R = 0.01;
}

void KalmanFilterClass::init(double x_init, Eigen::Matrix3d P0)
{
  if (!is_initialized)
  {
    X << x_init, 0, 0;
    P = P0;
    is_initialized = true;
  }
}

void KalmanFilterClass::setR(double new_R)
{
  R = new_R;
}

Eigen::Vector3d KalmanFilterClass::update(double measurement, double dt)
{
  Eigen::Vector3d X_;
  Eigen::Matrix3d P_;
  Eigen::Matrix3d I;
  I << Eigen::MatrixXd::Identity(3, 3);

  //A and Q are dependent on dt, they change at each time step
  A << 1.0, dt, pow(dt,2)/2.0,
       0.0, 1.0, dt,
       0.0, 0.0, 1.0;

  Q << pow(dt,5)/20.0, pow(dt,4)/8.0, pow(dt,3)/6.0,
       pow(dt,4)/8.0, pow(dt,3)/3.0, pow(dt,2)/2.0,
       pow(dt,3)/6.0, pow(dt,2)/2.0, dt;

  //time update (prediction)
  X_ = A * X;
  P_ = A * P * A.transpose() + Q;

  //measurement update (correction)
  K = P_ * H.transpose() / (H * P_ * H.transpose() + R);
  X = X_ + K * (measurement - H * X_);
  P = (I - K * H) * P_;

  return X;

}
void KalmanFilterClass::reset()
{
  is_initialized = false;
}
