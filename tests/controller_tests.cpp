#include "route_robot/controller.hpp"
#include "route_robot/simulation.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace route_robot;
namespace {
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const char* message) {
  check(std::abs(actual-expected) <= tolerance, message);
}
template<class F> void rejects(F function) {
  try { function(); } catch (const std::invalid_argument&) { return; }
  throw std::runtime_error("expected invalid_argument");
}
}

int main() {
  try {
    Path line({{0, 0}, {4, 0}});
    near(line.cross_track_error({2, 1}), 1, 1e-12, "segment projection");
    near(line.cross_track_error({5, 0}), 1, 1e-12, "endpoint distance");
    near(line.at(2).x, 2, 1e-12, "arc interpolation");
    PurePursuit straight;
    const auto forward = straight.update({0, 0, 0}, line);
    near(forward.command.angular, 0, 1e-12, "straight command");
    check(forward.command.linear > 0, "forward command");
    PurePursuit rotated;
    const auto turn = rotated.update({0, 0, 1.5707963267948966}, line);
    check(turn.command.angular < 0, "world-to-body steering sign");
    PurePursuit behind;
    const auto reverse_heading = behind.update({0, 0, 3.141592653589793}, line);
    near(reverse_heading.command.linear, 0, 1e-12, "rotate when target behind");
    PurePursuit circle;
    check(!circle.update({0, 0, 0}, make_route("circle")).complete,
          "closed route must not finish at start");
    const Pose arc = integrate({0, 0, 0}, 0, 2, 2, 1.5707963267948966);
    near(arc.x, 1, 1e-12, "exact arc x");
    near(arc.y, 1, 1e-12, "exact arc y");
    rejects([] { Path invalid({{0, 0}, {0, 0}}); });
    rejects([] { ControllerConfig c; c.lookahead = -1; PurePursuit invalid(c); });
    rejects([&] { straight.update({std::numeric_limits<double>::quiet_NaN(), 0, 0}, line); });

    for (const char* name : {"line", "circle", "s_curve"}) {
      const Path path = make_route(name);
      PurePursuit controller;
      Pose pose{0, 0.5, 0.6};
      bool complete = false;
      double progress = 0;
      for (int i = 0; i < 3000; ++i) {
        const auto r = controller.update(pose, path);
        check(r.progress >= progress, "progress must be monotonic");
        progress = r.progress;
        const double left = r.command.linear-r.command.angular*0.18;
        const double right = r.command.linear+r.command.angular*0.18;
        check(std::abs(left) <= 0.9+1e-12 && std::abs(right) <= 0.9+1e-12, "wheel limits");
        check(std::abs(r.command.angular) <= 1.8+1e-12, "angular limit");
        check(r.command.linear >= 0 && r.command.linear <= 0.6+1e-12, "linear limit");
        if (r.complete) {
          complete = true;
          near(r.command.linear, 0, 1e-12, "stop linear");
          near(r.command.angular, 0, 1e-12, "stop angular");
          check(distance({pose.x, pose.y}, path.points().back()) <= 0.06, "finish tolerance");
          break;
        }
        pose = integrate(pose, left, right, 0.36, 0.02);
      }
      check(complete, "route must complete with initial offset");
      controller.reset();
      check(controller.update({0, 0, 0}, path).progress < 1.0, "reset progress");
    }
    std::cout << "All controller and kinematic integration checks passed.\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
