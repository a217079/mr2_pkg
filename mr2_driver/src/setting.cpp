#define _USE_MATH_DEFINES
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/Float64.h"

std_msgs::Float64 fr_leg1_pos;
std_msgs::Float64 fr_leg2_pos;
std_msgs::Float64 fr_leg3_pos;
std_msgs::Float64 fl_leg1_pos;
std_msgs::Float64 fl_leg2_pos;
std_msgs::Float64 fl_leg3_pos;
std_msgs::Float64 rr_leg1_pos;
std_msgs::Float64 rr_leg2_pos;
std_msgs::Float64 rr_leg3_pos;
std_msgs::Float64 rl_leg1_pos;
std_msgs::Float64 rl_leg2_pos;
std_msgs::Float64 rl_leg3_pos;

double deg2rad(double deg)
{
  return deg*M_PI/180;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "setting_node");
  ros::NodeHandle n;

  // publish
  ros::Publisher fr_leg1_pos_pub = n.advertise<std_msgs::Float64>("mr2/fr_leg1_controller/command", 10);
  ros::Publisher fr_leg2_pos_pub = n.advertise<std_msgs::Float64>("mr2/fr_leg2_controller/command", 10);
  ros::Publisher fr_leg3_pos_pub = n.advertise<std_msgs::Float64>("mr2/fr_leg3_controller/command", 10);
  ros::Publisher fl_leg1_pos_pub = n.advertise<std_msgs::Float64>("mr2/fl_leg1_controller/command", 10);
  ros::Publisher fl_leg2_pos_pub = n.advertise<std_msgs::Float64>("mr2/fl_leg2_controller/command", 10);
  ros::Publisher fl_leg3_pos_pub = n.advertise<std_msgs::Float64>("mr2/fl_leg3_controller/command", 10);
  ros::Publisher rr_leg1_pos_pub = n.advertise<std_msgs::Float64>("mr2/rr_leg1_controller/command", 10);
  ros::Publisher rr_leg2_pos_pub = n.advertise<std_msgs::Float64>("mr2/rr_leg2_controller/command", 10);
  ros::Publisher rr_leg3_pos_pub = n.advertise<std_msgs::Float64>("mr2/rr_leg3_controller/command", 10);
  ros::Publisher rl_leg1_pos_pub = n.advertise<std_msgs::Float64>("mr2/rl_leg1_controller/command", 10);
  ros::Publisher rl_leg2_pos_pub = n.advertise<std_msgs::Float64>("mr2/rl_leg2_controller/command", 10);
  ros::Publisher rl_leg3_pos_pub = n.advertise<std_msgs::Float64>("mr2/rl_leg3_controller/command", 10);

  // subscribe
  ros::Rate loop_rate(20);
  
  XmlRpc::XmlRpcValue offsets;
  n.getParam("/offsets", offsets);
  fr_leg1_pos.data = deg2rad(-(double)offsets["servo0"]);
  fr_leg2_pos.data = deg2rad(-(double)offsets["servo1"]);
  fr_leg3_pos.data = deg2rad(-(double)offsets["servo2"]); 
  rr_leg1_pos.data = deg2rad(-(double)offsets["servo3"]);
  rr_leg2_pos.data = deg2rad(-(double)offsets["servo4"]);
  rr_leg3_pos.data = deg2rad(-(double)offsets["servo5"]); 
  fl_leg1_pos.data = deg2rad(-(double)offsets["servo6"]); 
  fl_leg2_pos.data = deg2rad(-(double)offsets["servo7"]);
  fl_leg3_pos.data = deg2rad(-(double)offsets["servo8"]);
  rl_leg1_pos.data = deg2rad(-(double)offsets["servo9"]);
  rl_leg2_pos.data = deg2rad(-(double)offsets["servo10"]);
  rl_leg3_pos.data = deg2rad(-(double)offsets["servo11"]);

  while(ros::ok())
  {
    fr_leg1_pos_pub.publish(fr_leg1_pos);
    fr_leg2_pos_pub.publish(fr_leg2_pos);
    fr_leg3_pos_pub.publish(fr_leg3_pos);
    fl_leg1_pos_pub.publish(fl_leg1_pos);
    fl_leg2_pos_pub.publish(fl_leg2_pos);
    fl_leg3_pos_pub.publish(fl_leg3_pos);
    rr_leg1_pos_pub.publish(rr_leg1_pos);
    rr_leg2_pos_pub.publish(rr_leg2_pos);
    rr_leg3_pos_pub.publish(rr_leg3_pos);
    rl_leg1_pos_pub.publish(rl_leg1_pos);
    rl_leg2_pos_pub.publish(rl_leg2_pos);
    rl_leg3_pos_pub.publish(rl_leg3_pos);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
