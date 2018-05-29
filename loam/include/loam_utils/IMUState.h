#ifndef LOAM_IMUSTATE_H
#define LOAM_IMUSTATE_H

#include "loam_utils/common.h"
#include "loam_utils/Angle.h"
#include "loam_utils/Vector3.h"
#include "loam_utils/CircularBuffer.h"


namespace loam {

/** IMU state data. */
typedef struct IMUState {
  /** The time of the measurement leading to this state (in seconds). */
  Time stamp;

  /** The current roll angle. */
  Angle roll;

  /** The current pitch angle. */
  Angle pitch;

  /** The current yaw angle. */
  Angle yaw;

  /** The accumulated global IMU position in 3D space. */
  Vector3 position;

  /** The accumulated global IMU velocity in 3D space. */
  Vector3 velocity;

  /** The current (local) IMU acceleration in 3D space. */
  Vector3 acceleration;

  /** \brief Interpolate between two IMU states.
   *
   * @param start the first IMUState
   * @param end the second IMUState
   * @param ratio the interpolation ratio
   * @param result the target IMUState for storing the interpolation result
   */
  static void interpolate(const IMUState& start,
                          const IMUState& end,
                          const float& ratio,
                          IMUState& result)
  {
    float invRatio = 1 - ratio;

    result.roll = start.roll.rad() * invRatio + end.roll.rad() * ratio;
    result.pitch = start.pitch.rad() * invRatio + end.pitch.rad() * ratio;
    if (start.yaw.rad() - end.yaw.rad() > M_PI) {
      result.yaw = start.yaw.rad() * invRatio + (end.yaw.rad() + 2 * M_PI) * ratio;
    } else if (start.yaw.rad() - end.yaw.rad() < -M_PI) {
      result.yaw = start.yaw.rad() * invRatio + (end.yaw.rad() - 2 * M_PI) * ratio;
    } else {
      result.yaw = start.yaw.rad() * invRatio + end.yaw.rad() * ratio;
    }

    result.velocity = start.velocity * invRatio + end.velocity * ratio;
    result.position = start.position * invRatio + end.position * ratio;
  };
} IMUState;

}

#endif //LOAM_IMUSTATE_H