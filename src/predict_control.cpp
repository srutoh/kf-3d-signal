#include "ros/ros.h"
#include "std_msgs/String.h"

#include <catch_it_package/Target_pos.h>
#include <catch_it_package/targetclass.h>
#include <catch_it_package/kalmanfilterclass.h>

double find_quadratic_roots(double a, double b, double c)
{
  double discriminant = b*b - 4*a*c;
  if (discriminant >= 0)
    return (-b + sqrt(discriminant)) / (2*a);
  else
    return 0.0;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "predict_control");
  ros::NodeHandle nh;
  ros::Rate loop_rate(25);

  catchClass::Target target;

  ros::Publisher predict_pub = nh.advertise<catch_it_package::Target_pos>("predicted_target_pos", 1);
  ros::Subscriber sub = nh.subscribe("target_pos", 100, &catchClass::Target::pos_vis_callback, &target);

  std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> all_pos;
  KalmanFilterClass KF_x, KF_y, KF_z;
  bool KF_init = false;
  Eigen::Matrix3d P0;
  P0 << 10, 0, 0,
        0, 10, 0,
        0, 0, 10;

  double z_pred = 0.5;
  double x_pred, y_pred;


  ros::Publisher vel_pub = nh.advertise<catch_it_package::Target_pos>("predicted_target_vel", 1);
  ros::Publisher acc_pub = nh.advertise<catch_it_package::Target_pos>("predicted_target_acc", 1);

  while (ros::ok())
  {
    if (target.in_range(5))
    {
      if (!KF_init)
      {
        //initializing KF
        Eigen::Vector3d init_pos;
        init_pos = target.get_current_pos();
        KF_x.init(init_pos(0), P0);
        KF_y.init(init_pos(1), P0);
        KF_z.init(init_pos(2), P0);

        KF_init = true;
      }

      //pos_dt with 0:x, 1:y, 2:z, 3:dt from previous state to current
      Eigen::Vector4d pos_dt;
      pos_dt = target.get_current_pos_dt();

      //current states
      double x_k, y_k, z_k, dt;
      x_k = pos_dt(0);
      y_k = pos_dt(1);
      z_k = pos_dt(2);
      dt = pos_dt(3);

      //predict state with KF
      //these vectors contain state position, velocity, acceleration prediction
      Eigen::Vector3d v_X_pred, v_Y_pred, v_Z_pred;
      v_X_pred = KF_x.update(x_k, dt);
      v_Y_pred = KF_y.update(y_k, dt);
      v_Z_pred = KF_z.update(z_k, dt);

      //calculate predicted position at z_pred = 0.5 if target still above z_pred
      if (z_k > z_pred)
      {
        //find t for equation: delta_z + v * t + 1/2 *a * t^2 = 0
        double delta_z, t_pred;
        delta_z = z_k - z_pred;
        t_pred = find_quadratic_roots(v_Z_pred(2)/2, v_Z_pred(1), delta_z);

        //with predicted duration t, calculate predicted x and y position
        x_pred = v_X_pred(0) + v_X_pred(1) * t_pred + v_X_pred(2) / 2 * pow(t_pred,2);
        y_pred = v_Y_pred(0) + v_Y_pred(1) * t_pred + v_Y_pred(2) / 2 * pow(t_pred,2);

        ROS_INFO_STREAM("t_pred: "<<t_pred<<"; ");
      }

      x_pred = v_X_pred(0);
      y_pred = v_Y_pred(0);
      z_pred = v_Z_pred(0);

      catch_it_package::Target_pos vel_pred;
      vel_pred.x = v_X_pred(1);
      vel_pred.y = v_Y_pred(1);
      vel_pred.z = v_Z_pred(1);
      vel_pub.publish(vel_pred);
      catch_it_package::Target_pos acc_pred;
      acc_pred.x = v_X_pred(2);
      acc_pred.y = v_Y_pred(2);
      acc_pred.z = v_Z_pred(2);
      acc_pub.publish(acc_pred);


      //estimate averaged velocities on x, y and acceleration on z
//      all_pos = target.get_all_pos();
//      std::vector<double> v_vel_x, v_vel_y, v_vel_z, v_acc_z;
//      double v_x, v_y, acc_z;

//      for (std::vector<int>::size_type i = 1; i != all_pos.size(); i++)
//      {
//        Eigen::Vector4d previous_state, current_state;
//        //in each state vector: 0:x, 1:y, 2:z, 3:time stamp
//        previous_state = all_pos[i-1];
//        current_state = all_pos[i];
//        double delta_t = current_state(3) - previous_state(3);
//        v_vel_x.push_back((current_state(0) - previous_state(0)) / delta_t);
//        v_vel_y.push_back((current_state(1) - previous_state(1)) / delta_t);
//        v_vel_z.push_back((current_state(2) - previous_state(2)) / delta_t);

//      }
//      //calculate average of velocity x, y
//      v_x = 1.0*std::accumulate( v_vel_x.begin(), v_vel_x.end(), 0.0 )/v_vel_x.size();
//      v_y = 1.0*std::accumulate( v_vel_y.begin(), v_vel_y.end(), 0.0 )/v_vel_y.size();

//      ROS_INFO_STREAM("v_x and v_y: "<< v_x<< ", "<< v_y <<"; with v_vel_x and v_vel_y sizes: "<<v_vel_x.size()<<", "<<v_vel_y.size());

//      //calculate acceleration on z
//      for (std::vector<int>::size_type i = 1; i != v_vel_z.size(); i++)
//      {
//        v_acc_z.push_back(v_vel_z[i] - v_vel_z[i-1]);
//      }
//      acc_z = 1.0*std::accumulate( v_acc_z.begin(), v_acc_z.end(), 0.0 )/v_acc_z.size();

//      ROS_INFO_STREAM("z acceleration: "<<acc_z);

//      //starting position
//      double x_recent, y_recent, z_recent;
//      x_recent = all_pos.back()(0);
//      y_recent = all_pos.back()(1);
//      z_recent = all_pos.back()(2);
//      //with velocity and acceleration estimation, predict position at z_pred = 0.5
//      double z_pred = 0.5;
//      double t_pred, x_pred, y_pred, delta_z;
//      delta_z = z_pred - z_recent;
//      t_pred = sqrt(fabs(2 * delta_z / acc_z));

//      ROS_INFO_STREAM("t_pred is : "<< t_pred);

//      x_pred = x_recent + v_x * t_pred;
//      y_pred = y_recent + v_y * t_pred;

//      ROS_INFO_STREAM("x_pred and y_pred : "<< x_pred <<", "<<y_pred);

//      double x_pred = 0.5;
//      double y_pred = 0.5;
//      double z_pred = 0.5;



      //publish the position prediction
      catch_it_package::Target_pos pos_pred;
      pos_pred.x = x_pred;
      pos_pred.y = y_pred;
      pos_pred.z = z_pred;

      predict_pub.publish(pos_pred);
    }
    else if (!target.is_moving())
    {
      target.reset();
      KF_x.reset();
      KF_y.reset();
      KF_z.reset();
      KF_init = false;
    }


    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
