#include "transformation.h"
#include <cmath>

Matrix4d trans(double x, double y, double z){
  Matrix4d M;
  M << 1, 0, 0, x,
       0, 1, 0, y,
       0, 0, 1, z,
       0, 0, 0, 1;
  return M;
}

Matrix4d rotY(double th){
  Matrix4d M;
  M <<  cos(th), 0, sin(th), 0,
              0, 1,       0, 0,
       -sin(th), 0, cos(th), 0,
              0, 0,       0, 1;
  return M;
}

Matrix4d rotZ(double th){
  Matrix4d M;
  M  << cos(th), -sin(th), 0, 0,
        sin(th),  cos(th), 0, 0,
              0,        0, 1, 0,
              0,        0, 0, 1;
  return M;
}
