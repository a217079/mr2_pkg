#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "ik_solver.h"
#include "mr2_driver/LegsCommands.h"

#define X 325
#define X_SLIDE 65
#define X_UP 325
#define Y 325
#define Z -230
#define STRIDE 75
#define LIFT 130
#define ALPHA 0

enum LegID
{
  FRONT_RIGHT,
  FRONT_LEFT,
  REAR_RIGHT,
  REAR_LEFT,
};

void setTarget(int, Vector3d*);

int main(int argc, char **argv)
{
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

  target[FRONT_RIGHT] << X+X_SLIDE, Y-STRIDE, Z;
  target[FRONT_LEFT] << -X+X_SLIDE, Y, Z;
  target[REAR_RIGHT] << X+X_SLIDE, -(Y-STRIDE), Z;
  target[REAR_LEFT] << -X+X_SLIDE, -Y, Z;
  ik_fr.inverseKinematics(target[0]);
  ik_fl.inverseKinematics(target[1]);
  ik_rr.inverseKinematics(target[2]);
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

  ros::Duration(5.0).sleep();
  cmds_pub.publish(cmds);
  ros::Duration(8.0).sleep();

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

    cmds_pub.publish(cmds);

    step = (step + 1) % 15;

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}


void setTarget(int step, Vector3d* targetPtr)
{
  switch(step)
  {
    case 0:
      targetPtr[FRONT_RIGHT] << X+X_SLIDE, Y-(STRIDE+ALPHA), Z;
      targetPtr[FRONT_LEFT] << -X+X_SLIDE, Y, Z;
      targetPtr[REAR_RIGHT] << X+X_SLIDE, -(Y-(STRIDE+ALPHA)), Z;
      targetPtr[REAR_LEFT] << -X+X_SLIDE, -Y, Z;
      break;
    case 1:
      targetPtr[FRONT_RIGHT] << X_UP, Y-(STRIDE+ALPHA), Z+LIFT;
      targetPtr[FRONT_LEFT] << -X+X_SLIDE, Y, Z;
      targetPtr[REAR_RIGHT] << X+X_SLIDE, -(Y-(STRIDE+ALPHA)), Z;
      targetPtr[REAR_LEFT] << -X+X_SLIDE, -Y, Z;
      break;
    case 2:
      targetPtr[FRONT_RIGHT] << X_UP, Y+(STRIDE+ALPHA), Z+LIFT;
      targetPtr[FRONT_LEFT] << -X+X_SLIDE, Y, Z;
      targetPtr[REAR_RIGHT] << X+X_SLIDE, -(Y-(STRIDE+ALPHA)), Z;
      targetPtr[REAR_LEFT] << -X+X_SLIDE, -Y, Z;
      break;
    case 3:
      targetPtr[FRONT_RIGHT] << X+X_SLIDE, Y+(STRIDE+ALPHA), Z;
      targetPtr[FRONT_LEFT] << -X+X_SLIDE, Y, Z;
      targetPtr[REAR_RIGHT] << X+X_SLIDE, -(Y-(STRIDE+ALPHA)), Z;
      targetPtr[REAR_LEFT] << -X+X_SLIDE, -Y, Z;
      break;
    case 4:
      targetPtr[FRONT_RIGHT] << X-X_SLIDE, Y, Z;
      targetPtr[FRONT_LEFT] << -X-X_SLIDE, Y-(STRIDE-ALPHA), Z;
      targetPtr[REAR_RIGHT] << X-X_SLIDE, -Y, Z;
      targetPtr[REAR_LEFT] << -X-X_SLIDE, -(Y+(STRIDE-ALPHA)), Z;
      break;
    case 5:
      targetPtr[FRONT_RIGHT] << X-X_SLIDE, Y, Z;
      targetPtr[FRONT_LEFT] << -X-X_SLIDE, Y-(STRIDE-ALPHA), Z;
      targetPtr[REAR_RIGHT] << X-X_SLIDE, -Y, Z;
      targetPtr[REAR_LEFT] << -X_UP, -(Y+(STRIDE-ALPHA)), Z+LIFT;
      break;
    case 6:
      targetPtr[FRONT_RIGHT] << X-X_SLIDE, Y, Z;
      targetPtr[FRONT_LEFT] << -X-X_SLIDE, Y-(STRIDE-ALPHA), Z;
      targetPtr[REAR_RIGHT] << X-X_SLIDE, -Y, Z;
      targetPtr[REAR_LEFT] << -X_UP, -(Y-(STRIDE-ALPHA)), Z+LIFT;
      break;
    case 7:
      targetPtr[FRONT_RIGHT] << X-X_SLIDE, Y, Z;
      targetPtr[FRONT_LEFT] << -X-X_SLIDE, Y-(STRIDE-ALPHA), Z;
      targetPtr[REAR_RIGHT] << X-X_SLIDE, -Y, Z;
      targetPtr[REAR_LEFT] << -X-X_SLIDE, -(Y-(STRIDE-ALPHA)), Z;
      break;
    case 8:
      targetPtr[FRONT_RIGHT] << X-X_SLIDE, Y, Z;
      targetPtr[FRONT_LEFT] << -X_UP, Y-(STRIDE-ALPHA), Z+LIFT;
      targetPtr[REAR_RIGHT] << X-X_SLIDE, -Y, Z;
      targetPtr[REAR_LEFT] << -X-X_SLIDE, -(Y-(STRIDE-ALPHA)), Z;
      break;
    case 9:
      targetPtr[FRONT_RIGHT] << X-X_SLIDE, Y, Z;
      targetPtr[FRONT_LEFT] << -X_UP, Y+(STRIDE-ALPHA), Z+LIFT;
      targetPtr[REAR_RIGHT] << X-X_SLIDE, -Y, Z;
      targetPtr[REAR_LEFT] << -X-X_SLIDE, -(Y-(STRIDE-ALPHA)), Z;
      break;
    case 10:
      targetPtr[FRONT_RIGHT] << X-X_SLIDE, Y, Z;
      targetPtr[FRONT_LEFT] << -X-X_SLIDE, Y+(STRIDE-ALPHA), Z;
      targetPtr[REAR_RIGHT] << X-X_SLIDE, -Y, Z;
      targetPtr[REAR_LEFT] << -X-X_SLIDE, -(Y-(STRIDE-ALPHA)), Z;
      break;
    case 11:
      targetPtr[FRONT_RIGHT] << X+X_SLIDE, Y-(STRIDE+ALPHA), Z;
      targetPtr[FRONT_LEFT] << -X+X_SLIDE, Y, Z;
      targetPtr[REAR_RIGHT] << X+X_SLIDE, -(Y+(STRIDE+ALPHA)), Z;
      targetPtr[REAR_LEFT] << -X+X_SLIDE, -Y, Z;
      break;
    case 12:
      targetPtr[FRONT_RIGHT] << X+X_SLIDE, Y-(STRIDE+ALPHA), Z;
      targetPtr[FRONT_LEFT] << -X+X_SLIDE, Y, Z;
      targetPtr[REAR_RIGHT] << X_UP, -(Y+(STRIDE+ALPHA)), Z+LIFT;
      targetPtr[REAR_LEFT] << -X+X_SLIDE, -Y, Z;
      break;
    case 13:
      targetPtr[FRONT_RIGHT] << X+X_SLIDE, Y-(STRIDE+ALPHA), Z;
      targetPtr[FRONT_LEFT] << -X+X_SLIDE, Y, Z;
      targetPtr[REAR_RIGHT] << X_UP, -(Y-(STRIDE+ALPHA)), Z+LIFT;
      targetPtr[REAR_LEFT] << -X+X_SLIDE, -Y, Z;
      break;
    case 14:
      targetPtr[FRONT_RIGHT] << X+X_SLIDE, Y-(STRIDE+ALPHA), Z;
      targetPtr[FRONT_LEFT] << -X+X_SLIDE, Y, Z;
      targetPtr[REAR_RIGHT] << X+X_SLIDE, -(Y-(STRIDE+ALPHA)), Z;
      targetPtr[REAR_LEFT] << -X+X_SLIDE, -Y, Z;
      break;
  }
}
