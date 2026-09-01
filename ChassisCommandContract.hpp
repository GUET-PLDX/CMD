#pragma once

#include <cmath>

namespace Pldx::ChassisCommandContract {

inline constexpr float MAX_VX_MPS = 2.5f;
inline constexpr float MAX_VY_MPS = 2.5f;
inline constexpr float MAX_WZ_RAD_S = 1.8f;

inline bool is_valid_si(float vx, float vy, float wz) {
  return std::isfinite(vx) && std::isfinite(vy) && std::isfinite(wz) &&
         std::fabs(vx) <= MAX_VX_MPS && std::fabs(vy) <= MAX_VY_MPS &&
         std::fabs(wz) <= MAX_WZ_RAD_S;
}

}  // namespace Pldx::ChassisCommandContract
