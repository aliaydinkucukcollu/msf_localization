#include "filters/filter_creator.hpp"
#include "filters/ekf.hpp"
#include "filters/lkf.hpp"
#include <memory>
#include <stdexcept>

namespace msf_localization_core {

std::unique_ptr<Filter>
FilterCreator::create_filter(const std::string &filter_type, const double &dt,
                             const Filter::StateVector &x0,
                             const Filter::StateCovarianceMatrix &P0) {
  if (filter_type == "LKF") {
    return std::make_unique<LinearKalmanFilter>(dt, x0, P0);
  }

  if (filter_type == "EKF") {
    return std::make_unique<ExtendedKalmanFilter>(dt, x0, P0);
  }

  throw std::invalid_argument("Unknown Filter Type: " + filter_type +
                              "\n Supported Filter Types: LKF, EKF\n");
}

} // namespace msf_localization_core