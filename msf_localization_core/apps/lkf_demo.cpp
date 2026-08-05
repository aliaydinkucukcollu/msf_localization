#include <iostream>

#include "filters/filter.hpp"
#include "filters/lkf.hpp"

int main() {
  Eigen::VectorXd x(9);
  x.setZero();

  Eigen::MatrixXd P(9, 9);
  P.setIdentity();

  const double dt{0.01};

  msf_localization_core::LinearKalmanFilter filter(dt, x, P);

  // control vector (IMU)
  Eigen::VectorXd u(6);
  u << 0.1, 0.0, 9.81, 0.0, 0.0, 0.05;

  // measurement vector (GPS)
  Eigen::VectorXd z(3);
  z << 1.02, 0.00, 0.00;

  for (int i = 0; i < 100; ++i) {
    filter.predict(u);

    // Every 1 second (100 steps @ 100Hz), fuse a GPS fix
    if ((i + 1) % 100 == 0) {
      filter.update(z);
    }
  }

  std::cout << "Final state vector:\n" << filter.get_state() << "\n";
  std::cout << "Final state covariance matrix:\n" << filter.get_covariance() << "\n";

  return 0;
}