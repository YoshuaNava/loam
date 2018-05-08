#ifndef LOAM_TWIST_H
#define LOAM_TWIST_H


#include "Angle.h"
#include "Vector3.h"


namespace loam {


/** \brief Twist composed by three angles and a three-dimensional position.
 *
 */
class Twist {
public:
  Twist()
        : rot_x(),
          rot_y(),
          rot_z(),
          pos() {};

  Angle rot_x;
  Angle rot_y;
  Angle rot_z;
  Vector3 pos;

  void setZero()
  {
    rot_x = 0;
    rot_y = 0;
    rot_z = 0;
    pos.x() = 0;
    pos.y() = 0;
    pos.z() = 0;
  }

  void operator=(const Twist &tw) {
    rot_x = (tw.rot_x);
    rot_y = (tw.rot_y);
    rot_z = (tw.rot_z);
    pos = (tw.pos);
  }
};

} // end namespace loam

#endif //LOAM_TWIST_H
