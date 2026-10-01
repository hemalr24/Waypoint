#include "route_robot/ekf.hpp"
#include "route_robot/simulation.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace route_robot;
namespace {
void check(bool pass, const char* message) { if (!pass) throw std::runtime_error(message); }
void near(double a, double b, double tol, const char* msg) { check(std::abs(a-b)<=tol, msg); }
void positive_definite(const EkfCovariance& p) {
  EkfCovariance l{};
  for (int i=0; i<5; ++i) {
    for (int j=0; j<=i; ++j) {
      near(p[i][j], p[j][i], 1e-12, "symmetric covariance");
      double d = p[i][j];
      for (int k=0; k<j; ++k) d -= l[i][k]*l[j][k];
      if (i==j) { check(d>0 && std::isfinite(d), "positive definite covariance"); l[i][j]=std::sqrt(d); }
      else l[i][j]=d/l[j][j];
    }
  }
}
}
int main() {
  try {
    // Finite-difference derivative independent of hand-coded Jacobian.
    for (double yaw : {-3.0, -0.7, 0.0, 2.0, 3.13}) {
      EkfState s{1, 2, yaw, 0.6, 0.7};
      const auto f = ekf_motion_jacobian(s, 0.02);
      for (int j=0; j<5; ++j) {
        auto a=s, b=s; a[j]+=1e-6; b[j]-=1e-6;
        a=ekf_motion(a, 0.02); b=ekf_motion(b, 0.02);
        for (int i=0; i<5; ++i) {
          const double delta = i==2 ? wrap_angle(a[i]-b[i]) : a[i]-b[i];
          near(f[i][j], delta/2e-6, 1e-8, "motion Jacobian finite differences");
        }
      }
    }
    PlanarEkf stationary({0,0,0});
    for (int i=0;i<2000;++i) stationary.update(0,0,0,0.02);
    near(stationary.pose().x,0,1e-12,"stationary x");
    positive_definite(stationary.covariance());
    PlanarEkf straight({0,0,0});
    for (int i=0;i<1000;++i) straight.update(0.6,0,0,0.02);
    near(straight.pose().x,12,0.03,"constant-speed distance");
    near(straight.pose().y,0,1e-10,"straight lateral position");
    PlanarEkf turning({0,0,0});
    Pose truth{};
    for (int i=0;i<2000;++i) {
      truth=integrate(truth,0.6-0.3*0.18,0.6+0.3*0.18,0.36,0.02);
      turning.update(0.6,0.3,0.3,0.02);
      positive_definite(turning.covariance());
    }
    near(turning.pose().x,truth.x,0.04,"turning x");
    near(turning.pose().y,truth.y,0.04,"turning y");
    near(wrap_angle(turning.pose().yaw-truth.yaw),0,0.01,"wrapped turn heading");
    // A more precise gyro should dominate an inconsistent wheel turn estimate.
    PlanarEkf disagreement({0,0,0});
    for (int i=0;i<500;++i) disagreement.update(0.6,0.2,0,0.02);
    check(std::abs(disagreement.state()[4])<0.005,"gyro weighting");
    // Independent scalar-Gaussian result for the first turn-rate correction.
    PlanarEkf scalar({0,0,0});
    scalar.update(0,0.1,0.2,0.02);
    const double prior_variance=1+16*0.02*0.02;
    const double posterior_variance=1/(1/prior_variance+1/(0.25*0.25)+1/(0.02*0.02));
    const double posterior_mean=posterior_variance*(0.1/(0.25*0.25)+0.2/(0.02*0.02));
    near(scalar.state()[4],posterior_mean,1e-12,"scalar Gaussian posterior mean");
    near(scalar.covariance()[4][4],posterior_variance,1e-12,"scalar Gaussian posterior variance");
    for (int i=0;i<2000;++i) {
      scalar.update(0.5+0.1*std::sin(i),0.9*std::sin(i*0.1),0.7*std::sin(i*0.1)+0.02*std::cos(i),0.02);
      positive_definite(scalar.covariance());
    }
    const auto before=disagreement.state();
    bool rejected=false;
    try { disagreement.update(1,0,std::numeric_limits<double>::quiet_NaN(),0.02); }
    catch (const std::invalid_argument&) { rejected=true; }
    check(rejected && before==disagreement.state(),"reject invalid input before mutation");
    rejected=false;
    try { disagreement.update(1,0,0,0); } catch (const std::invalid_argument&) { rejected=true; }
    check(rejected,"reject zero dt");
    rejected=false;
    try { EkfConfig c; c.gyro_std=0; PlanarEkf bad({0,0,0},c); }
    catch (const std::invalid_argument&) { rejected=true; }
    check(rejected,"reject zero observation variance");
    std::cout << "EKF Jacobian, motion, covariance, weighting, and validation checks passed.\n";
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
