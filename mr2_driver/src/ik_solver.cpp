#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include "ik_solver.h"
#include "transformation.h"

IKSolver::IKSolver(FrontRear front_rear, RightLeft right_left, Model model)
{
  fr = front_rear;
  rl = right_left;
  
  setKinematicsInfo(model);
  theta << 0, 0, 0;
  origin << 0, 0, 0, 1;
}

IKSolver::~IKSolver(){}

void IKSolver::setKinematicsInfo(Model model)
{
  switch(model)
  {
    case VIRTUAL_MODEL:
      a = 240, b = 240;
      l1 = 60, l2 = 130, l3 = 270;
      break;
    case REAL_MODEL:
      a = 228, b = 260;
      l1 = 200, l2 = 100, l3 = 200;
      break;
    default:
      a = 240, b = 240;
      l1 = 60, l2 = 130, l3 = 270;
  }
}

void IKSolver::inverseKinematics(const Vector3d target)
{
  Vector4d o = trans((rl*a)/2, (fr*b)/2, 0) * origin;
  theta(0) = atan( (target(1) - o(1)) / (target(0) - o(0)) );
  double z = origin(2) - target(2);
  Vector4d p1 = trans((rl*a)/2, (fr*b)/2, 0) * rotZ(theta(0)) * trans(rl*l1, 0, 0) * origin;
  Vector3d L_vec = target - p1.topRows(3);
  double L = L_vec.norm();
  theta(1) = acos(z/L) + acos( (pow(l3,2) - pow(l2,2) - pow(L,2)) / (-2 * l2 * L) ) - M_PI/2;
  theta(2) = acos( (pow(L,2) - pow(l2,2) - pow(l3,2)) / (-2 * l2 * l3) ) - M_PI/2;
  theta(0) = -theta(0);
}

Vector3d IKSolver::getRadian() const
{
  return theta;
}

Vector3d IKSolver::getDegree() const
{
  Vector3d degree;
  for(int i=0;i<3;i++)
    degree(i) = theta(i) * 180 / M_PI;
  return degree;
}
