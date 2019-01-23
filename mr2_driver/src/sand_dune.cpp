#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "ik_solver.h"
#include "mr2_driver/LegsCommands.h"
#include <fstream>
#include <string>
#include <sstream>

using namespace std;

int setArray(double (&data)[512][12])
{
  int row, column;
  ifstream ifs("/home/tats/catkin_ws/src/mr2_pkg/mr2_driver/csv/sand_dune_gait.csv");
  if (!ifs)
  {
    cerr << "Failed to open the file." << endl;
    return -1;
  }
  string line;
  row = 0;
  while (getline(ifs, line))
  {
    string tmp;
    istringstream stream(line);
    column = 0;
    while (getline(stream, tmp, ','))
    {
      data[row][column] = stod(tmp);
      ROS_INFO("(%d,%d):%lf", row, column, data[row][column]);
      column++;
    }
    row++;
  }
  ifs.close();
  return row-1;
}

int main(int argc, char **argv)
{
  double csv_data[512][12];
  int max_step;
  max_step = setArray(csv_data);
  ROS_INFO("max_step:%d",max_step);
  ros::init(argc, argv, "simple_creep_gait");
  ros::NodeHandle n;

  ros::Publisher cmds_pub = n.advertise<mr2_driver::LegsCommands>("cmds", 10);
  mr2_driver::LegsCommands cmds;

  ros::Rate loop_rate(2);

  IKSolver ik_fr(FRONT,RIGHT,REAL_MODEL2);
  IKSolver ik_fl(FRONT,LEFT,REAL_MODEL2);
  IKSolver ik_rr(REAR,RIGHT,REAL_MODEL2);
  IKSolver ik_rl(REAR,LEFT,REAL_MODEL2);
  Vector3d theta_fr;
  Vector3d theta_fl;
  Vector3d theta_rr;
  Vector3d theta_rl;
  Vector3d target[4];
  int step = 0;
  ros::Duration(5.0).sleep();

  while (ros::ok())
  {
    for (int i=0; i<4; i++)
      target[i] << csv_data[step][3*i], csv_data[step][3*i + 1], csv_data[step][3*i + 2];

    ik_fr.inverseKinematics(target[0]);
    ik_rr.inverseKinematics(target[1]);
    ik_fl.inverseKinematics(target[2]);
    ik_rl.inverseKinematics(target[3]);
    theta_fr = ik_fr.getDegree();
    theta_fl = ik_fl.getDegree();
    theta_rr = ik_rr.getDegree();
    theta_rl = ik_rl.getDegree(); 

    XmlRpc::XmlRpcValue offsets;
    n.getParam("/offsets", offsets); 
    cmds.deg[0] = theta_fr(0) - (double)offsets["servo0"];
    cmds.deg[1] = theta_fr(1) - (double)offsets["servo1"];
    cmds.deg[2] = theta_fr(2) - (double)offsets["servo2"];
    cmds.deg[3] = theta_rr(0) - (double)offsets["servo3"];
    cmds.deg[4] = theta_rr(1) - (double)offsets["servo4"];
    cmds.deg[5] = theta_rr(2) - (double)offsets["servo5"];
    cmds.deg[6] = theta_fl(0) - (double)offsets["servo6"];
    cmds.deg[7] = theta_fl(1) - (double)offsets["servo7"];
    cmds.deg[8] = theta_fl(2) - (double)offsets["servo8"];
    cmds.deg[9] = theta_rl(0) - (double)offsets["servo9"];
    cmds.deg[10] = theta_rl(1) - (double)offsets["servo10"];
    cmds.deg[11] = theta_rl(2) - (double)offsets["servo11"];

    if (step==1) ros::Duration(8.0).sleep();

    cmds_pub.publish(cmds);

    if (step < max_step) step++;

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}


