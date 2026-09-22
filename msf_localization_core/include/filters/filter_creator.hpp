#pragma once

#include "filters/ekf.hpp"
#include "filters/filter.hpp"
#include "filters/lkf.hpp"

#include <memory>
#include <string>

namespace msf_localization_core {

class FilterCreator {
public:
  static std::unique_ptr<Filter>
  create_filter(const std::string &filter_type, const double &dt,
                const Filter::StateVector &x0,
                const Filter::StateCovarianceMatrix &P0);
};

} // namespace msf_localization_core