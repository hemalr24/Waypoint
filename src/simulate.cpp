#include "route_robot/controller.hpp"
#include "route_robot/simulation.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace route_robot;

int main(int argc, char** argv) {
  try {
    if (argc != 4) {
      std::cerr << "Usage: simulate <line|circle|s_curve> <nominal|offset|stalled> <output-directory>\n";
      return 2;
    }
    const std::string scenario = argv[2];
    if (scenario != "nominal" && scenario != "offset" && scenario != "stalled")
      throw std::invalid_argument("unknown scenario: " + scenario);
    const Path path = make_route(argv[1]);
    const ControllerConfig config;
    PurePursuit controller(config);
    const auto& points = path.points();
    Pose truth{points[0].x, points[0].y,
               std::atan2(points[1].y-points[0].y, points[1].x-points[0].x)};
    if (scenario == "offset") { truth.y += 0.5; truth.yaw += 0.6; }
    std::filesystem::create_directories(argv[3]);
    const auto output = std::filesystem::path(argv[3]);
    std::ofstream route_file(output/"path.csv");
    std::ofstream log(output/"trajectory.csv");
    if (!route_file || !log) throw std::runtime_error("cannot create output files");
    route_file << std::setprecision(12) << "x,y\n";
    for (Point p : points) route_file << p.x << ',' << p.y << '\n';
    log << std::setprecision(12)
        << "time,true_x,true_y,true_yaw,estimated_x,estimated_y,estimated_yaw,target_x,target_y,"
           "linear,angular,left_command,right_command,left_actual,right_actual,progress,cross_track_error,controller_us,complete,timed_out\n";
    constexpr double dt = 0.02, timeout = 60.0;
    bool complete = false;
    for (int step = 0; step <= static_cast<int>(timeout/dt); ++step) {
      const double time = step*dt;
      // Stage 1 intentionally supplies simulator ground truth to the controller.
      const Pose estimate = truth;
      const auto before = std::chrono::steady_clock::now();
      auto result = controller.update(estimate, path);
      const double elapsed = std::chrono::duration<double, std::micro>(
          std::chrono::steady_clock::now()-before).count();
      const bool timed_out = step == static_cast<int>(timeout/dt) && !result.complete;
      if (timed_out) result.command = {0, 0};
      const double left = result.command.linear-result.command.angular*config.track_width/2;
      const double right = result.command.linear+result.command.angular*config.track_width/2;
      // Explicit actuator-failure fixture, not a physical wheel/ground contact model.
      const double traction = scenario == "stalled" && time >= 2.0 ? 0.0 : 1.0;
      const double actual_left = left*traction, actual_right = right*traction;
      log << time << ',' << truth.x << ',' << truth.y << ',' << truth.yaw << ','
          << estimate.x << ',' << estimate.y << ',' << estimate.yaw << ','
          << result.target.x << ',' << result.target.y << ','
          << result.command.linear << ',' << result.command.angular << ','
          << left << ',' << right << ',' << actual_left << ',' << actual_right << ','
          << result.progress << ',' << path.cross_track_error({truth.x, truth.y}) << ','
          << elapsed << ',' << result.complete << ',' << timed_out << '\n';
      if (result.complete || timed_out) { complete = result.complete; break; }
      truth = integrate(truth, actual_left, actual_right, config.track_width, dt);
    }
    if (!log || !route_file) throw std::runtime_error("failed while writing results");
    std::cout << argv[1] << '/' << scenario << ": " << (complete ? "completed" : "timed out") << '\n';
    return 0;  // Scenario failure is a recorded outcome, not a runner error.
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
