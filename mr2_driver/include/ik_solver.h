#ifndef _IK_SOLVER_H_
#define _IK_SOLVER_H_

#include <Eigen/Dense>

using namespace Eigen;

enum FrontRear
{
  FRONT = 1,
  REAR = -1,
};
enum RightLeft
{
  RIGHT = 1,
  LEFT = -1,
};
enum Model
{
  VIRTUAL_MODEL,
  REAL_MODEL,
  REAL_MODEL2,
  REAL_MODEL3,
};

class IKSolver
{
private:
  //    KINEMATICS INFORMATION OF THE LEG
  // location of the leg
  int fr;  // specify which leg is this ( front or rear )
  int rl;  // specify which leg is this ( right or left )
  // geometry information about each part
  double a;   // distance between right base joints and left base joints ( default 240[mm] )
  double b;   // distance between front base joints and rear base joints ( default 240[mm] )
  double l1;  // length of base link              ( default :  60[mm] )
  double l2;  // length of middle link            ( default : 130[mm] )
  double l3;  // length of top link (default [mm] ( default : 200[mm] )

  //    STATUS OF THE LEG
  Vector3d theta;   // angles of each joint (th1, th2, th3)
  Vector4d origin;  // position of origin (0, 0, 0, 1)
  Vector4d toe;     // position of toe    (x, y, z, 1)

  Vector3d target;  // target point (x, y, z)
  
  void setKinematicsInfo(Model);
public:
  IKSolver(FrontRear, RightLeft, Model);
  ~IKSolver();

  void inverseKinematics(const Vector3d);
  Vector3d getRadian() const;
  Vector3d getDegree() const;
};

#endif
