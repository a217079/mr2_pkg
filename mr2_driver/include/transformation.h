#ifndef _TRANSFORMATION_H_
#define _TRANSFORMATION_H_

#include <Eigen/Dense>

using namespace Eigen;

Matrix4d trans(double x, double y, double z);
Matrix4d rotY(double theta);
Matrix4d rotZ(double theta);

#endif
