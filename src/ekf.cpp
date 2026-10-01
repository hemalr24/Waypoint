#include "route_robot/ekf.hpp"
#include <cmath>
#include <stdexcept>

namespace route_robot {
namespace {
EkfCovariance identity() {
  EkfCovariance matrix{};
  for (int i = 0; i < 5; ++i) matrix[i][i] = 1;
  return matrix;
}
EkfCovariance sandwich(const EkfCovariance& a, const EkfCovariance& p) {
  EkfCovariance ap{}, result{};
  for (int i = 0; i < 5; ++i)
    for (int j = 0; j < 5; ++j)
      for (int k = 0; k < 5; ++k) ap[i][j] += a[i][k]*p[k][j];
  for (int i = 0; i < 5; ++i)
    for (int j = 0; j < 5; ++j)
      for (int k = 0; k < 5; ++k) result[i][j] += ap[i][k]*a[j][k];
  return result;
}
}

EkfState ekf_motion(EkfState s, double dt) {
  const double direction = s[2] + 0.5*s[4]*dt;
  s[0] += s[3]*dt*std::cos(direction);
  s[1] += s[3]*dt*std::sin(direction);
  s[2] = wrap_angle(s[2]+s[4]*dt);
  return s;
}

EkfCovariance ekf_motion_jacobian(const EkfState& s, double dt) {
  auto f = identity();
  const double c = std::cos(s[2]+0.5*s[4]*dt), sn = std::sin(s[2]+0.5*s[4]*dt);
  f[0][2] = -s[3]*dt*sn; f[0][3] = dt*c; f[0][4] = -0.5*s[3]*dt*dt*sn;
  f[1][2] = s[3]*dt*c; f[1][3] = dt*sn; f[1][4] = 0.5*s[3]*dt*dt*c;
  f[2][4] = dt;
  return f;
}

PlanarEkf::PlanarEkf(Pose initial, EkfConfig config) : config_(config) {
  for (double value : {config.wheel_speed_std, config.wheel_turn_std, config.gyro_std,
                       config.acceleration_std, config.angular_acceleration_std})
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("EKF deviations must be finite and positive");
  for (double value : {initial.x, initial.y, initial.yaw})
    if (!std::isfinite(value)) throw std::invalid_argument("invalid EKF initial pose");
  state_ = {initial.x, initial.y, wrap_angle(initial.yaw), 0, 0};
  covariance_ = identity();
  for (int i = 0; i < 3; ++i) covariance_[i][i] = 1e-9;
}

void PlanarEkf::observe(int component, double value, double variance) {
  const double innovation = value-state_[component];
  const double innovation_variance = covariance_[component][component]+variance;
  EkfState gain{};
  for (int i = 0; i < 5; ++i) gain[i] = covariance_[i][component]/innovation_variance;
  for (int i = 0; i < 5; ++i) state_[i] += gain[i]*innovation;
  state_[2] = wrap_angle(state_[2]);
  auto a = identity();
  for (int i = 0; i < 5; ++i) a[i][component] -= gain[i];
  // Joseph form keeps measurement covariance positive semidefinite numerically.
  auto next = sandwich(a, covariance_);
  for (int i = 0; i < 5; ++i)
    for (int j = 0; j < 5; ++j) next[i][j] += gain[i]*variance*gain[j];
  for (int i = 0; i < 5; ++i)
    for (int j = 0; j < 5; ++j) covariance_[i][j] = 0.5*(next[i][j]+next[j][i]);
}

void PlanarEkf::update(double wheel_speed, double wheel_turn, double gyro_turn, double dt) {
  for (double value : {wheel_speed, wheel_turn, gyro_turn, dt})
    if (!std::isfinite(value)) throw std::invalid_argument("non-finite EKF input");
  if (dt <= 0) throw std::invalid_argument("EKF dt must be positive");
  const double direction = state_[2]+0.5*state_[4]*dt;
  const auto f = ekf_motion_jacobian(state_, dt);
  state_ = ekf_motion(state_, dt);
  covariance_ = sandwich(f, covariance_);
  // Piecewise constant acceleration uncertainty: Q = G diag(sigma^2) G^T.
  const EkfState gv{0.5*dt*dt*std::cos(direction), 0.5*dt*dt*std::sin(direction), 0, dt, 0};
  const EkfState gw{0, 0, 0.5*dt*dt, 0, dt};
  for (int i = 0; i < 5; ++i)
    for (int j = 0; j < 5; ++j)
      covariance_[i][j] += gv[i]*gv[j]*config_.acceleration_std*config_.acceleration_std
                        + gw[i]*gw[j]*config_.angular_acceleration_std*config_.angular_acceleration_std;
  // Independent rate observations. Do not also fuse integrated encoder pose.
  observe(3, wheel_speed, config_.wheel_speed_std*config_.wheel_speed_std);
  observe(4, wheel_turn, config_.wheel_turn_std*config_.wheel_turn_std);
  gyro_innovation_ = gyro_turn-state_[4];
  gyro_innovation_variance_ = covariance_[4][4]+config_.gyro_std*config_.gyro_std;
  observe(4, gyro_turn, config_.gyro_std*config_.gyro_std);
}
}  // namespace route_robot
