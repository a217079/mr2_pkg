#define _USE_MATH_DEFINES
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "mr2_driver/LegsCommands.h"

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

void cmds_callback(const mr2_driver::LegsCommands& cmds)
{
  fr_leg1_pos.data = deg2rad(cmds.deg[0]);
  fr_leg2_pos.data = deg2rad(cmds.deg[1]);
  fr_leg3_pos.data = deg2rad(cmds.deg[2]);
  fl_leg1_pos.data = deg2rad(cmds.deg[3]);
  fl_leg2_pos.data = deg2rad(cmds.deg[4]);
  fl_leg3_pos.data = deg2rad(cmds.deg[5]);
  rr_leg1_pos.data = deg2rad(cmds.deg[6]);
  rr_leg2_pos.data = deg2rad(cmds.deg[7]);
  rr_leg3_pos.data = deg2rad(cmds.deg[8]);
  rl_leg1_pos.data = deg2rad(cmds.deg[9]);
  rl_leg2_pos.data = deg2rad(cmds.deg[10]);
  rl_leg3_pos.data = deg2rad(cmds.deg[11]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "command_publisher");
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
  ros::Subscriber cmds_sub  = n.subscribe("cmds", 10, cmds_callback); 

  ros::Rate loop_rate(5);

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
