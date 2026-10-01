#pragma once

#include "route_robot/controller.hpp"
#include <cmath>

namespace route_robot {
// Exact integration of constant differential-drive wheel speeds over one step.
inline Pose integrate(Pose pose, double left, double right, double track, double dt) {
  const double v = (left + right)/2;
  const double w = (right - left)/track;
  const double half_turn = w*dt/2;
  const double sinc = std::abs(half_turn) < 1e-8 ? 1.0 : std::sin(half_turn)/half_turn;
  pose.x += v*dt*sinc*std::cos(pose.yaw + half_turn);
  pose.y += v*dt*sinc*std::sin(pose.yaw + half_turn);
  pose.yaw = wrap_angle(pose.yaw + w*dt);
  return pose;
}
}  // namespace route_robot
