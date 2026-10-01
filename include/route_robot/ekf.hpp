#pragma once
#include "route_robot/controller.hpp"
#include <array>

namespace route_robot {
using EkfState = std::array<double, 5>; // x, y, yaw, forward speed, turn rate
using EkfCovariance = std::array<std::array<double, 5>, 5>;

struct EkfConfig {
  double wheel_speed_std{0.04};       // m/s, observation tuning
  double wheel_turn_std{0.25};        // rad/s, allows wheel-model errors
  double gyro_std{0.02};              // rad/s
  double acceleration_std{1.0};       // m/s^2, independent each prediction interval
  double angular_acceleration_std{4.0}; // rad/s^2
};

// Midpoint discrete motion model and its analytic state Jacobian.
EkfState ekf_motion(EkfState state, double dt);
EkfCovariance ekf_motion_jacobian(const EkfState& state, double dt);

class PlanarEkf {
 public:
  explicit PlanarEkf(Pose initial, EkfConfig config = {});
  // Observations are interval-average rates for the just-finished dt interval.
  // Receives no truth, absolute position, odometry pose, or heading measurement.
  void update(double wheel_speed, double wheel_turn, double gyro_turn, double dt);
  Pose pose() const { return {state_[0], state_[1], state_[2]}; }
  const EkfState& state() const { return state_; }
  const EkfCovariance& covariance() const { return covariance_; }
  double gyro_innovation() const { return gyro_innovation_; }
  double gyro_innovation_variance() const { return gyro_innovation_variance_; }
 private:
  void observe(int component, double value, double variance);
  EkfConfig config_;
  EkfState state_{};
  EkfCovariance covariance_{};
  double gyro_innovation_{}, gyro_innovation_variance_{};
};
}  // namespace route_robot
