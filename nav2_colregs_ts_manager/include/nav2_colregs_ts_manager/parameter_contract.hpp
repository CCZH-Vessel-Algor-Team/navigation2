#ifndef NAV2_COLREGS_TS_MANAGER__PARAMETER_CONTRACT_HPP_
#define NAV2_COLREGS_TS_MANAGER__PARAMETER_CONTRACT_HPP_

#include <cmath>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace nav2_colregs_ts_manager
{
inline rcl_interfaces::msg::ParameterDescriptor parameterDescription(
  const std::string & description, bool read_only)
{
  rcl_interfaces::msg::ParameterDescriptor result;
  result.description = description;
  result.read_only = read_only;
  result.additional_constraints = read_only ? "Startup-only; restart node to change." :
    "Applied on the next successful avoidance service request.";
  return result;
}

inline void validateNumber(const std::string & name, double value, bool positive = false)
{
  if (!std::isfinite(value) || (positive ? value <= 0.0 : value < 0.0)) {
    throw std::invalid_argument(name + " must be finite and " +
            (positive ? "positive" : "nonnegative"));
  }
}

inline void validateSafetyFactor(double value)
{
  if (!std::isfinite(value) || value < 1.0) {
    throw std::invalid_argument("safety_factor must be finite and at least 1 (radius inflation)");
  }
}
}  // namespace nav2_colregs_ts_manager
#endif  // NAV2_COLREGS_TS_MANAGER__PARAMETER_CONTRACT_HPP_
