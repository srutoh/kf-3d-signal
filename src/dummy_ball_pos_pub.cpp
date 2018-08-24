#include <math.h>
#define PI 3.14159265
#include <random>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <catch_it_package/Target_pos.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_ball_pos_pub");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<catch_it_package::Target_pos>("target_pos", 1000);

  ros::Rate loop_rate(10);

  //initial values
  double posX = 1.0;
  double posY = 0.0;
  double posZ = 2.0;
  double alpha = 2.0;
  double v_x, v_y, v_z;

  double g = - 0.098;

  //adding random gaussian noise
  const double mean = 0.0;
  const double stddev = 0.01;
  std::default_random_engine generator;
  std::normal_distribution<double> dist(mean, stddev);

  catch_it_package::Target_pos msg;
  double t_pre = ros::Time::now().toSec();

  //give small random velocities on x, y and z
  v_y =  500 * dist(generator);
  v_x =  500 * dist(generator);
  v_z =  -10/* * dist(generator)*/;

  while (ros::ok())
  {
    double t_now = ros::Time::now().toSec();
    double delta_t = t_now - t_pre;
    posX = posX + v_x * delta_t + dist(generator);
    posY = posY + v_y * delta_t + dist(generator);
    posZ = posZ + v_z * delta_t + dist(generator);
    v_z = v_z + g * delta_t;

    msg.x = posX;
    msg.y = posY;
    msg.z = posZ;

    pub.publish(msg);

    if (posZ < 0)
    {
      //reset position and velocity and sleep for 5 sec when reaches ground
      ros::Duration(3.0).sleep();
      posX = 1.0;
      posY = 0.0;
      posZ = 2.0;
      v_y = 0.1 * dist(generator);
      v_x = 0.1 * dist(generator);
      v_z = 0;
    }
    ros::spinOnce();

    t_pre = t_now;
    loop_rate.sleep();
  }

  return 0;
}
