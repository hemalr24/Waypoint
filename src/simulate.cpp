#include "route_robot/controller.hpp"
#include "route_robot/odometry.hpp"
#include "route_robot/ekf.hpp"
#include "route_robot/simulation.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>

using namespace route_robot;

int main(int argc, char** argv) {
  try {
    if (argc != 4 && argc != 6) {
      std::cerr << "Usage: simulate <line|circle|s_curve> "
                   "<nominal|offset|stalled|encoder_noise|radius_mismatch|wheel_slip|combined|gyro_bias|symmetric_slip> "
                   "<output-directory> [<ground_truth|odometry|ekf> <uint32-seed>]\n";
      return 2;
    }
    const std::string scenario = argv[2];
    const std::string feedback = argc == 6 ? argv[4] : "ground_truth";
    if (feedback != "ground_truth" && feedback != "odometry" && feedback != "ekf")
      throw std::invalid_argument("unknown feedback: " + feedback);
    std::uint32_t seed = 0;
    if (argc == 6) {
      const std::string value = argv[5];
      if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos)
        throw std::invalid_argument("seed must be an unsigned 32-bit integer");
      const auto parsed = std::stoull(value);
      if (parsed > std::numeric_limits<std::uint32_t>::max())
        throw std::invalid_argument("seed exceeds uint32 range");
      seed = static_cast<std::uint32_t>(parsed);
    }
    if (scenario != "nominal" && scenario != "offset" && scenario != "stalled" &&
        scenario != "encoder_noise" && scenario != "radius_mismatch" &&
        scenario != "wheel_slip" && scenario != "combined" && scenario != "gyro_bias" && scenario != "symmetric_slip")
      throw std::invalid_argument("unknown scenario: " + scenario);
    const Path path = make_route(argv[1]);
    const ControllerConfig config;
    const EkfConfig ekf_config;
    constexpr double dt = 0.02, timeout = 60.0, assumed_radius = 0.1;
    const bool mismatch = scenario == "radius_mismatch" || scenario == "combined";
    const bool slip = scenario == "wheel_slip" || scenario == "combined" || scenario == "symmetric_slip";
    const double left_traction = scenario == "symmetric_slip" ? 0.7 : (slip ? 0.65 : 1.0);
    const double right_traction = scenario == "symmetric_slip" ? 0.7 : (slip ? 0.95 : 1.0);
    constexpr double gyro_noise_std = 0.02;
    const double gyro_bias = scenario == "gyro_bias" ? 0.08 : 0.0;
    const double noise_std = scenario == "encoder_noise" || scenario == "combined" ? 0.4 : 0.0;
    const double actual_left_radius = assumed_radius*(mismatch ? 0.98 : 1.0);
    const double actual_right_radius = assumed_radius*(mismatch ? 1.02 : 1.0);
    PurePursuit controller(config);
    const auto& points = path.points();
    Pose truth{points[0].x, points[0].y,
               std::atan2(points[1].y-points[0].y, points[1].x-points[0].x)};
    if (scenario == "offset") { truth.y += 0.5; truth.yaw += 0.6; }
    // Known initial pose is the sole initialization shared with the estimator.
    WheelOdometry odometry(truth, assumed_radius, assumed_radius, config.track_width);
    PlanarEkf ekf(truth, ekf_config);
    std::mt19937 generator(seed);
    std::normal_distribution<double> normal(0.0, 1.0);
    // Separate stream leaves existing encoder samples unchanged.
    std::mt19937 gyro_generator(seed ^ 0x9e3779b9U);
    std::normal_distribution<double> gyro_normal(0.0, 1.0);
    std::filesystem::create_directories(argv[3]);
    const auto output = std::filesystem::path(argv[3]);
    std::ofstream route_file(output/"path.csv"), log(output/"trajectory.csv"), metadata(output/"run.json");
    if (!route_file || !log || !metadata) throw std::runtime_error("cannot create output files");
    // Configuration is recorded at its source, avoiding duplicated Python defaults.
    metadata << std::setprecision(12)
      << "{\n  \"route\": \"" << argv[1] << "\", \"scenario\": \"" << scenario
      << "\", \"feedback\": \"" << feedback << "\", \"seed\": " << seed
      << ",\n  \"dt_s\": " << dt << ", \"timeout_s\": " << timeout
      << ",\n  \"assumed_left_radius_m\": " << assumed_radius << ", \"assumed_right_radius_m\": " << assumed_radius
      << ",\n  \"actual_left_radius_m\": " << actual_left_radius << ", \"actual_right_radius_m\": " << actual_right_radius
      << ",\n  \"assumed_track_width_m\": " << config.track_width << ", \"actual_track_width_m\": " << config.track_width
      << ",\n  \"encoder_velocity_noise_std_rad_s\": " << noise_std
      << ",\n  \"gyro_noise_std_rad_s\": " << gyro_noise_std << ", \"gyro_bias_rad_s\": " << gyro_bias
      << ", \"gyro_seed\": " << (seed ^ 0x9e3779b9U)
      << ",\n  \"ekf\": {\"wheel_speed_std_m_s\": " << ekf_config.wheel_speed_std
      << ", \"wheel_turn_std_rad_s\": " << ekf_config.wheel_turn_std
      << ", \"gyro_std_rad_s\": " << ekf_config.gyro_std
      << ", \"acceleration_std_m_s2\": " << ekf_config.acceleration_std
      << ", \"angular_acceleration_std_rad_s2\": " << ekf_config.angular_acceleration_std
      << ", \"initial_pose_variance\": 1e-9, \"initial_rate_variance\": 1}"
      << ",\n  \"slip_start_s\": 2, \"slip_end_s\": 8, \"left_traction_during_slip\": " << left_traction
      << ", \"right_traction_during_slip\": " << right_traction
      << ",\n  \"stall_start_s\": " << (scenario == "stalled" ? "2" : "null")
      << ",\n  \"initial_pose\": [" << truth.x << ',' << truth.y << ',' << truth.yaw << ']'
      << ",\n  \"controller\": {\"lookahead_m\": " << config.lookahead << ", \"cruise_speed_m_s\": " << config.cruise_speed
      << ", \"max_angular_speed_rad_s\": " << config.max_angular_speed << ", \"track_width_m\": " << config.track_width
      << ", \"max_wheel_rim_speed_m_s\": " << config.max_wheel_speed << ", \"goal_tolerance_m\": " << config.goal_tolerance
      << ", \"approach_gain_per_s\": " << config.approach_gain << "}\n}\n";
    route_file << std::setprecision(12) << "x,y\n";
    for (Point p : points) route_file << p.x << ',' << p.y << '\n';
    log << std::setprecision(12)
        << "time,true_x,true_y,true_yaw,estimated_x,estimated_y,estimated_yaw,target_x,target_y,"
           "linear,angular,left_command,right_command,left_actual,right_actual,progress,cross_track_error,controller_us,complete,timed_out,"
           "odometry_x,odometry_y,odometry_yaw,left_encoder_delta,right_encoder_delta,left_encoder_noise,right_encoder_noise,true_progress,"
           "ekf_x,ekf_y,ekf_yaw,ekf_speed,ekf_turn,gyro_rate,gyro_noise,gyro_innovation,gyro_innovation_variance,"
           "ekf_var_x,ekf_var_y,ekf_var_yaw,ekf_var_speed,ekf_var_turn,ekf_us\n";
    bool complete = false;
    double left_delta = 0, right_delta = 0, left_noise = 0, right_noise = 0, true_progress = 0;
    double gyro_rate = 0, gyro_noise = 0, ekf_us = 0;
    for (int step = 0; step <= static_cast<int>(timeout/dt); ++step) {
      const double time = step*dt;
      const Pose odom = odometry.pose();
      const Pose estimate = feedback == "ground_truth" ? truth : (feedback == "odometry" ? odom : ekf.pose());
      const auto before = std::chrono::steady_clock::now();
      auto result = controller.update(estimate, path);
      const double elapsed = std::chrono::duration<double, std::micro>(
          std::chrono::steady_clock::now()-before).count();
      const bool timed_out = step == static_cast<int>(timeout/dt) && !result.complete;
      if (timed_out) result.command = {0, 0};
      const double left = result.command.linear-result.command.angular*config.track_width/2;
      const double right = result.command.linear+result.command.angular*config.track_width/2;
      const bool stalled = scenario == "stalled" && time >= 2.0;
      // Commands use assumed radius. Shaft rotation precedes ground slip.
      const double left_shaft = stalled ? 0 : left/assumed_radius;
      const double right_shaft = stalled ? 0 : right/assumed_radius;
      const bool slipping = slip && time >= 2.0 && time < 8.0;
      const double actual_left = left_shaft*actual_left_radius*(slipping ? left_traction : 1.0);
      const double actual_right = right_shaft*actual_right_radius*(slipping ? right_traction : 1.0);
      // Evaluation-only progress; never read by controller or estimator.
      true_progress = path.project({truth.x, truth.y}, true_progress, true_progress+2*config.lookahead);
      log << time << ',' << truth.x << ',' << truth.y << ',' << truth.yaw << ','
          << estimate.x << ',' << estimate.y << ',' << estimate.yaw << ','
          << result.target.x << ',' << result.target.y << ','
          << result.command.linear << ',' << result.command.angular << ','
          << left << ',' << right << ',' << actual_left << ',' << actual_right << ','
          << result.progress << ',' << path.cross_track_error({truth.x, truth.y}) << ','
          << elapsed << ',' << result.complete << ',' << timed_out << ','
          << odom.x << ',' << odom.y << ',' << odom.yaw << ','
          << left_delta << ',' << right_delta << ',' << left_noise << ',' << right_noise << ',' << true_progress << ','
          << ekf.state()[0] << ',' << ekf.state()[1] << ',' << ekf.state()[2] << ',' << ekf.state()[3] << ',' << ekf.state()[4] << ','
          << gyro_rate << ',' << gyro_noise << ',' << ekf.gyro_innovation() << ',' << ekf.gyro_innovation_variance();
      for (int i = 0; i < 5; ++i) log << ',' << ekf.covariance()[i][i];
      log << ',' << ekf_us << '\n';
      if (result.complete || timed_out) { complete = result.complete; break; }
      truth = integrate(truth, actual_left, actual_right, config.track_width, dt);
      // Velocity noise integrated over dt gives measured angle increments.
      // Fixed draws per step in both modes enable paired trials.
      left_noise = noise_std*normal(generator);
      right_noise = noise_std*normal(generator);
      left_delta = (left_shaft+left_noise)*dt;
      right_delta = (right_shaft+right_noise)*dt;
      odometry.update(left_delta, right_delta);
      // The sensor model observes physical turn rate; the EKF sees only its noisy output.
      gyro_noise = gyro_noise_std*gyro_normal(gyro_generator);
      gyro_rate = (actual_right-actual_left)/config.track_width + gyro_bias + gyro_noise;
      const auto filter_before = std::chrono::steady_clock::now();
      ekf.update(assumed_radius*(left_delta+right_delta)/(2*dt),
                 assumed_radius*(right_delta-left_delta)/(config.track_width*dt), gyro_rate, dt);
      ekf_us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now()-filter_before).count();
    }
    log.flush(); route_file.flush(); metadata.flush();
    if (!log || !route_file || !metadata) throw std::runtime_error("failed while writing results");
    std::cout << argv[1] << '/' << scenario << '/' << feedback << " seed=" << seed << ": "
              << (complete ? "controller complete" : "timed out") << '\n';
    return 0;  // Scenario failure is a recorded outcome, not a runner error.
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
