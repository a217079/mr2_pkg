#include "gait.h"

void Gait::setTargetStraight(int step, Vector3d* targetPtr)
{
  switch(step)
  {
    case 0:
      targetPtr[FRONT_RIGHT] << X, Y-(STRIDE+ALPHA), Z;
      targetPtr[FRONT_LEFT] << -X, Y, Z;
      targetPtr[REAR_RIGHT] << X, -(Y-(STRIDE+ALPHA)), Z;
      targetPtr[REAR_LEFT] << -X, -Y, Z;
      break;
    case 1:
      targetPtr[FRONT_RIGHT] << X, Y-(STRIDE+ALPHA), Z+LIFT;
      targetPtr[FRONT_LEFT] << -X, Y, Z;
      targetPtr[REAR_RIGHT] << X, -(Y-(STRIDE+ALPHA)), Z;
      targetPtr[REAR_LEFT] << -X, -Y, Z;
      break;
    case 2:
      targetPtr[FRONT_RIGHT] << X, Y+(STRIDE+ALPHA), Z+LIFT;
      targetPtr[FRONT_LEFT] << -X, Y, Z;
      targetPtr[REAR_RIGHT] << X, -(Y-(STRIDE+ALPHA)), Z;
      targetPtr[REAR_LEFT] << -X, -Y, Z;
      break;
    case 3:
      targetPtr[FRONT_RIGHT] << X, Y+(STRIDE+ALPHA), Z;
      targetPtr[FRONT_LEFT] << -X, Y, Z;
      targetPtr[REAR_RIGHT] << X, -(Y-(STRIDE+ALPHA)), Z;
      targetPtr[REAR_LEFT] << -X, -Y, Z;
      break;
    case 4:
      targetPtr[FRONT_RIGHT] << X, Y, Z;
      targetPtr[FRONT_LEFT] << -X, Y-(STRIDE-ALPHA), Z;
      targetPtr[REAR_RIGHT] << X, -Y, Z;
      targetPtr[REAR_LEFT] << -X, -(Y+(STRIDE-ALPHA)), Z;
      break;
    case 5:
      targetPtr[FRONT_RIGHT] << X, Y, Z;
      targetPtr[FRONT_LEFT] << -X, Y-(STRIDE-ALPHA), Z;
      targetPtr[REAR_RIGHT] << X, -Y, Z;
      targetPtr[REAR_LEFT] << -X, -(Y+(STRIDE-ALPHA)), Z+LIFT;
      break;
    case 6:
      targetPtr[FRONT_RIGHT] << X, Y, Z;
      targetPtr[FRONT_LEFT] << -X, Y-(STRIDE-ALPHA), Z;
      targetPtr[REAR_RIGHT] << X, -Y, Z;
      targetPtr[REAR_LEFT] << -X, -(Y-(STRIDE-ALPHA)), Z+LIFT;
      break;
    case 7:
      targetPtr[FRONT_RIGHT] << X, Y, Z;
      targetPtr[FRONT_LEFT] << -X, Y-(STRIDE-ALPHA), Z;
      targetPtr[REAR_RIGHT] << X, -Y, Z;
      targetPtr[REAR_LEFT] << -X, -(Y-(STRIDE-ALPHA)), Z;
      break;
    case 8:
      targetPtr[FRONT_RIGHT] << X, Y, Z;
      targetPtr[FRONT_LEFT] << -X, Y-(STRIDE-ALPHA), Z+LIFT;
      targetPtr[REAR_RIGHT] << X, -Y, Z;
      targetPtr[REAR_LEFT] << -X, -(Y-(STRIDE-ALPHA)), Z;
      break;
    case 9:
      targetPtr[FRONT_RIGHT] << X, Y, Z;
      targetPtr[FRONT_LEFT] << -X, Y+(STRIDE-ALPHA), Z+LIFT;
      targetPtr[REAR_RIGHT] << X, -Y, Z;
      targetPtr[REAR_LEFT] << -X, -(Y-(STRIDE-ALPHA)), Z;
      break;
    case 10:
      targetPtr[FRONT_RIGHT] << X, Y, Z;
      targetPtr[FRONT_LEFT] << -X, Y+(STRIDE-ALPHA), Z;
      targetPtr[REAR_RIGHT] << X, -Y, Z;
      targetPtr[REAR_LEFT] << -X, -(Y-(STRIDE-ALPHA)), Z;
      break;
    case 11:
      targetPtr[FRONT_RIGHT] << X, Y-(STRIDE+ALPHA), Z;
      targetPtr[FRONT_LEFT] << -X, Y, Z;
      targetPtr[REAR_RIGHT] << X, -(Y+(STRIDE+ALPHA)), Z;
      targetPtr[REAR_LEFT] << -X, -Y, Z;
      break;
    case 12:
      targetPtr[FRONT_RIGHT] << X, Y-(STRIDE+ALPHA), Z;
      targetPtr[FRONT_LEFT] << -X, Y, Z;
      targetPtr[REAR_RIGHT] << X, -(Y+(STRIDE+ALPHA)), Z+LIFT;
      targetPtr[REAR_LEFT] << -X, -Y, Z;
      break;
    case 13:
      targetPtr[FRONT_RIGHT] << X, Y-(STRIDE+ALPHA), Z;
      targetPtr[FRONT_LEFT] << -X, Y, Z;
      targetPtr[REAR_RIGHT] << X, -(Y-(STRIDE+ALPHA)), Z+LIFT;
      targetPtr[REAR_LEFT] << -X, -Y, Z;
      break;
    case 14:
      targetPtr[FRONT_RIGHT] << X, Y-(STRIDE+ALPHA), Z;
      targetPtr[FRONT_LEFT] << -X, Y, Z;
      targetPtr[REAR_RIGHT] << X, -(Y-(STRIDE+ALPHA)), Z;
      targetPtr[REAR_LEFT] << -X, -Y, Z;
      break;
  }
}
