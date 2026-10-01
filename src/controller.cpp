#include "route_robot/controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace route_robot {
namespace {
constexpr double pi = 3.14159265358979323846;
double square(double x) { return x * x; }
bool finite(Point p) { return std::isfinite(p.x) && std::isfinite(p.y); }
}

double wrap_angle(double angle) { return std::remainder(angle, 2.0 * pi); }
double distance(Point a, Point b) { return std::hypot(a.x-b.x, a.y-b.y); }

Path::Path(std::vector<Point> points) : points_(std::move(points)) {
  if (points_.size() < 2) throw std::invalid_argument("path needs at least two points");
  arc_.push_back(0.0);
  for (std::size_t i = 0; i < points_.size(); ++i) {
    if (!finite(points_[i])) throw std::invalid_argument("non-finite path point");
    if (i) {
      const double step = distance(points_[i-1], points_[i]);
      if (step < 1e-9) throw std::invalid_argument("adjacent path points must differ");
      arc_.push_back(arc_.back() + step);
    }
  }
}

Point Path::at(double s) const {
  s = std::clamp(s, 0.0, length());
  const auto it = std::upper_bound(arc_.begin(), arc_.end(), s);
  const std::size_t i = std::min(static_cast<std::size_t>(it-arc_.begin()), arc_.size()-1);
  const double t = (s-arc_[i-1])/(arc_[i]-arc_[i-1]);
  return {points_[i-1].x + t*(points_[i].x-points_[i-1].x),
          points_[i-1].y + t*(points_[i].y-points_[i-1].y)};
}

double Path::project(Point p, double begin, double end) const {
  begin = std::clamp(begin, 0.0, length());
  end = std::clamp(end, begin, length());
  double best_s = begin, best_d = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i < points_.size(); ++i) {
    if (arc_[i] < begin || arc_[i-1] > end) continue;
    const Point a = points_[i-1], b = points_[i];
    const double ds = arc_[i]-arc_[i-1];
    const double t = std::clamp(((p.x-a.x)*(b.x-a.x)+(p.y-a.y)*(b.y-a.y))/square(ds),
                               std::max(0.0, (begin-arc_[i-1])/ds),
                               std::min(1.0, (end-arc_[i-1])/ds));
    const double d = square(p.x-a.x-t*(b.x-a.x)) + square(p.y-a.y-t*(b.y-a.y));
    if (d < best_d) { best_d = d; best_s = arc_[i-1] + t*ds; }
  }
  return best_s;
}

double Path::cross_track_error(Point p) const { return distance(p, at(project(p, 0.0, length()))); }
double Path::final_heading() const {
  const Point a = points_[points_.size()-2], b = points_.back();
  return std::atan2(b.y-a.y, b.x-a.x);
}

PurePursuit::PurePursuit(ControllerConfig config) : config_(config) {
  for (double v : {config.lookahead, config.cruise_speed, config.max_angular_speed,
                   config.track_width, config.max_wheel_speed, config.goal_tolerance,
                   config.approach_gain}) {
    if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("controller parameters must be finite and positive");
  }
  if (config.goal_tolerance >= config.lookahead)
    throw std::invalid_argument("goal tolerance must be smaller than lookahead");
}

ControlResult PurePursuit::update(const Pose& pose, const Path& path) {
  if (!finite({pose.x, pose.y}) || !std::isfinite(pose.yaw))
    throw std::invalid_argument("non-finite robot pose");
  // Local, monotonic progress prevents a closed route from finishing at its start.
  progress_ = path.project({pose.x, pose.y}, progress_, progress_ + 2*config_.lookahead);
  const Point target = path.at(progress_ + config_.lookahead);
  const double remaining = path.length()-progress_;
  const double endpoint_distance = distance({pose.x, pose.y}, path.points().back());
  if (remaining <= config_.lookahead && endpoint_distance <= config_.goal_tolerance)
    return {{0, 0}, target, progress_, true};

  const double dx = target.x-pose.x, dy = target.y-pose.y;
  const double local_x = std::cos(pose.yaw)*dx + std::sin(pose.yaw)*dy;
  const double local_y = -std::sin(pose.yaw)*dx + std::cos(pose.yaw)*dy;
  const double bearing = std::atan2(local_y, local_x);
  double v = config_.cruise_speed;
  double w;
  if (std::abs(bearing) > pi/2) {
    v = 0.0;
    w = std::clamp(2*bearing, -config_.max_angular_speed, config_.max_angular_speed);
  } else {
    if (remaining <= config_.lookahead) v = std::min(v, config_.approach_gain*endpoint_distance);
    const double curvature = 2*local_y/std::max(square(dx)+square(dy), 1e-12);
    if (std::abs(curvature) > 1e-12) v = std::min(v, config_.max_angular_speed/std::abs(curvature));
    w = v*curvature;
  }
  const double peak_wheel = std::max(std::abs(v-w*config_.track_width/2),
                                     std::abs(v+w*config_.track_width/2));
  const double scale = std::min(1.0, config_.max_wheel_speed/std::max(peak_wheel, 1e-12));
  return {{v*scale, w*scale}, target, progress_, false};
}

Path make_route(const char* name) {
  const std::string route(name);
  std::vector<Point> points;
  const int n = 400;
  for (int i = 0; i <= n; ++i) {
    const double u = static_cast<double>(i)/n;
    if (route == "line") points.push_back({6*u, 0});
    else if (route == "circle") points.push_back({2*std::sin(2*pi*u), 2*(1-std::cos(2*pi*u))});
    else if (route == "s_curve") points.push_back({8*u, std::sin(2*pi*u)});
    else throw std::invalid_argument("unknown route: " + route);
  }
  return Path(std::move(points));
}
}  // namespace route_robot
