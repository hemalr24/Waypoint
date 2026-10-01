#pragma once

#include <cstddef>
#include <vector>

namespace route_robot {

struct Point { double x{}, y{}; };
struct Pose { double x{}, y{}, yaw{}; };
struct Command { double linear{}, angular{}; };

double wrap_angle(double angle);
double distance(Point a, Point b);

// A route is a finite traversal, even when its endpoints coincide.
class Path {
 public:
  explicit Path(std::vector<Point> points);
  Point at(double arc_length) const;
  double project(Point point, double begin, double end) const;
  double cross_track_error(Point point) const;
  double length() const { return arc_.back(); }
  double final_heading() const;
  const std::vector<Point>& points() const { return points_; }

 private:
  std::vector<Point> points_;
  std::vector<double> arc_;
};

struct ControllerConfig {
  double lookahead{0.5};             // m
  double cruise_speed{0.6};          // m/s
  double max_angular_speed{1.8};     // rad/s
  double track_width{0.36};          // m
  double max_wheel_speed{0.9};       // wheel rim speed, m/s
  double goal_tolerance{0.06};       // m
  double approach_gain{1.2};        // 1/s
};

struct ControlResult {
  Command command;
  Point target;
  double progress{};
  bool complete{};
};

class PurePursuit {
 public:
  explicit PurePursuit(ControllerConfig config = {});
  ControlResult update(const Pose& pose, const Path& path);
  void reset() { progress_ = 0.0; }
 private:
  ControllerConfig config_;
  double progress_{};
};

Path make_route(const char* name);

}  // namespace route_robot
