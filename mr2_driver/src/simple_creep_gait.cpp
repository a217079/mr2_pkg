#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "ik_solver.h"
#include "mr2_driver/LegsCommands.h"

#define X 300
#define Y 300
#define Z -100
#define RANGE 100
#define Z_UP 50
#define ALPHA -10

void setTarget(int, Vector3d*);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_creep_gait");
  ros::NodeHandle n;

  ros::Publisher cmds_pub = n.advertise<mr2_driver::LegsCommands>("cmds", 10);
  mr2_driver::LegsCommands cmds;
  
  ros::Rate loop_rate(5);
  
  IKSolver ik_fr(FRONT,RIGHT,REAL_MODEL);
  IKSolver ik_fl(FRONT,LEFT,REAL_MODEL);
  IKSolver ik_rr(REAR,RIGHT,REAL_MODEL);
  IKSolver ik_rl(REAR,LEFT,REAL_MODEL);
  Vector3d theta_fr;
  Vector3d theta_fl;
  Vector3d theta_rr;
  Vector3d theta_rl;
  Vector3d target[4];
  int step = 0;

  while(ros::ok())
  {
    setTarget(step, target);
    ik_fr.inverseKinematics(target[0]);
    ik_fl.inverseKinematics(target[1]);
    ik_rr.inverseKinematics(target[2]);
    ik_rl.inverseKinematics(target[3]);
    theta_fr = ik_fr.getDegree();
    theta_fl = ik_fl.getDegree();
    theta_rr = ik_rr.getDegree();
    theta_rl = ik_rl.getDegree(); 

    for(int i=0;i<3;i++)
      cmds.deg[i] = theta_fr(i);
    for(int i=0;i<3;i++)
      cmds.deg[i+3] = theta_fl(i);
    for(int i=0;i<3;i++)
      cmds.deg[i+6] = theta_rr(i);
    for(int i=0;i<3;i++)
      cmds.deg[i+9] = theta_rl(i);

    cmds_pub.publish(cmds);

    step = (step + 1) % 11;

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

enum LegID
{
  FRONT_RIGHT,
  FRONT_LEFT,
  REAR_RIGHT,
  REAR_LEFT,
};

void setTarget(int step, Vector3d* targetPtr)
{
  switch(step)
  {
    case 0:
      targetPtr[FRONT_RIGHT] << X, Y-(RANGE+ALPHA), Z;
      targetPtr[FRONT_LEFT] << -X, Y, Z;
      targetPtr[REAR_RIGHT] << X, -(Y-(RANGE+ALPHA)), Z;
      targetPtr[REAR_LEFT] << -X, -Y, Z;
      break;
    case 1:
      targetPtr[FRONT_RIGHT] << X, Y, Z+Z_UP;
      targetPtr[FRONT_LEFT] << -X, Y, Z;
      targetPtr[REAR_RIGHT] << X, -(Y-(RANGE+ALPHA)), Z;
      targetPtr[REAR_LEFT] << -X, -Y, Z;
      break;
    case 2:
      targetPtr[FRONT_RIGHT] << X, Y+(RANGE+ALPHA), Z;
      targetPtr[FRONT_LEFT] << -X, Y, Z;
      targetPtr[REAR_RIGHT] << X, -(Y-(RANGE+ALPHA)), Z;
      targetPtr[REAR_LEFT] << -X, -Y, Z;
      break;
    case 3:
      targetPtr[FRONT_RIGHT] << X, Y, Z;
      targetPtr[FRONT_LEFT] << -X, Y-(RANGE-ALPHA), Z;
      targetPtr[REAR_RIGHT] << X, -Y, Z;
      targetPtr[REAR_LEFT] << -X, -(Y+(RANGE-ALPHA)), Z;
      break;
    case 4:
      targetPtr[FRONT_RIGHT] << X, Y, Z;
      targetPtr[FRONT_LEFT] << -X, Y-(RANGE-ALPHA), Z;
      targetPtr[REAR_RIGHT] << X, -Y, Z;
      targetPtr[REAR_LEFT] << -X, -Y, Z+Z_UP;
      break;
    case 5:
      targetPtr[FRONT_RIGHT] << X, Y, Z;
      targetPtr[FRONT_LEFT] << -X, Y-(RANGE-ALPHA), Z;
      targetPtr[REAR_RIGHT] << X, -Y, Z;
      targetPtr[REAR_LEFT] << -X, -(Y-(RANGE-ALPHA)), Z;
      break;
    case 6:
      targetPtr[FRONT_RIGHT] << X, Y, Z;
      targetPtr[FRONT_LEFT] << -X, Y, Z+Z_UP;
      targetPtr[REAR_RIGHT] << X, -Y, Z;
      targetPtr[REAR_LEFT] << -X, -(Y-(RANGE-ALPHA)), Z;
      break;
    case 7:
      targetPtr[FRONT_RIGHT] << X, Y, Z;
      targetPtr[FRONT_LEFT] << -X, Y+(RANGE-ALPHA), Z;
      targetPtr[REAR_RIGHT] << X, -Y, Z;
      targetPtr[REAR_LEFT] << -X, -(Y-(RANGE-ALPHA)), Z;
      break;
    case 8:
      targetPtr[FRONT_RIGHT] << X, Y-(RANGE+ALPHA), Z;
      targetPtr[FRONT_LEFT] << -X, Y, Z;
      targetPtr[REAR_RIGHT] << X, -(Y+(RANGE+ALPHA)), Z;
      targetPtr[REAR_LEFT] << -X, -Y, Z;
      break;
    case 9:
      targetPtr[FRONT_RIGHT] << X, Y-(RANGE+ALPHA), Z;
      targetPtr[FRONT_LEFT] << -X, Y, Z;
      targetPtr[REAR_RIGHT] << X, -Y, Z+Z_UP;
      targetPtr[REAR_LEFT] << -X, -Y, Z;
      break;
    case 10:
      targetPtr[FRONT_RIGHT] << X, Y-(RANGE+ALPHA), Z;
      targetPtr[FRONT_LEFT] << -X, Y, Z;
      targetPtr[REAR_RIGHT] << X, -(Y-(RANGE+ALPHA)), Z;
      targetPtr[REAR_LEFT] << -X, -Y, Z;
      break;
  }
}
