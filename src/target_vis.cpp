#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>

//#include <catch_it_package/Target_pos.h>
#include <catch_it_package/targetclass.h>


visualization_msgs::Marker createMarker(std::string frame_id, std::string ns, int id, double x, double y, double z,
                                        std::string colour, float scale)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;

  if (colour == "red")
  {
    marker.color.r = 0.9;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
  }
  else
  {
    marker.color.r = 0;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
  }
  marker.color.a = 1;
  marker.lifetime = ros::Duration(1.0);

  return marker;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "target_vis");
  ros::NodeHandle nh;
  ros::Rate r(30);

  catchClass::Target target;
  catchClass::Target pred_target;

  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("pos_marker", 1);
  ros::Publisher pred_marker_pub = nh.advertise<visualization_msgs::Marker>("prediction_marker", 1);
  ros::Subscriber sub = nh.subscribe("target_pos", 100, &catchClass::Target::pos_vis_callback, &target);
  ros::Subscriber sub2 = nh.subscribe("predicted_target_pos", 100, &catchClass::Target::pos_vis_callback, &pred_target);

  int marker_id_counter = 1;

  while(ros::ok())
  {

    if (target.is_moving())
    {
      visualization_msgs::Marker pos_marker;
      Eigen::Vector3d pos = target.get_current_pos();
      pos_marker = createMarker("/pos_marker_frame", "pos_markers", marker_id_counter,pos(0),pos(1),pos(2),"green", 0.05);

      marker_pub.publish(pos_marker);
      ++marker_id_counter;

      if (pred_target.is_moving())
      {
        visualization_msgs::Marker pred_pos_marker;
        Eigen::Vector3d pred_pos = pred_target.get_current_pos();
        pred_pos_marker = createMarker("/pos_marker_frame", "pred_markers", 0,pred_pos(0),pred_pos(1),pred_pos(2),"red", 0.1);

        pred_marker_pub.publish(pred_pos_marker);
      }

    }
    else
    {
      target.reset();
      pred_target.reset();
      marker_id_counter = 1;
    }


    ros::spinOnce();

    r.sleep();
  }
  return 0;
}
