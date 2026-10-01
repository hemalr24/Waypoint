#pragma once

#include "route_robot/controller.hpp"
#include <cmath>
#include <stdexcept>

namespace route_robot {
// Consumes measured wheel angle increments only; no access to simulator state.
class WheelOdometry {
 public:
  WheelOdometry(Pose initial, double left_radius, double right_radius, double track)
      : pose_(initial), left_radius_(left_radius), right_radius_(right_radius), track_(track) {
    for (double value : {left_radius, right_radius, track})
      if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("invalid odometry geometry");
    if (!std::isfinite(initial.x) || !std::isfinite(initial.y) || !std::isfinite(initial.yaw))
      throw std::invalid_argument("invalid initial odometry pose");
  }
  void update(double left_angle_delta, double right_angle_delta) {
    if (!std::isfinite(left_angle_delta) || !std::isfinite(right_angle_delta))
      throw std::invalid_argument("invalid encoder increment");
    const double left = left_angle_delta*left_radius_;
    const double right = right_angle_delta*right_radius_;
    const double translation = (right+left)/2;
    const double rotation = (right-left)/track_;
    const double half = rotation/2;
    const double scale = std::abs(half) < 1e-8 ? 1.0 : std::sin(half)/half;
    pose_.x += translation*scale*std::cos(pose_.yaw+half);
    pose_.y += translation*scale*std::sin(pose_.yaw+half);
    pose_.yaw = wrap_angle(pose_.yaw+rotation);
  }
  Pose pose() const { return pose_; }
 private:
  Pose pose_;
  double left_radius_, right_radius_, track_;
};
}  // namespace route_robot
