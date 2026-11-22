#pragma once

#include <cstdint>

namespace Pldx {

enum class LifterPosition : uint8_t { TOP = 0, BOTTOM = 1, MIDDLE = 2 };

struct LifterFeedback {
  LifterPosition position = LifterPosition::MIDDLE;
  bool is_transformable = false;
  float transform_percent = 0.0f;
};

inline constexpr char LIFTER_TARGET_TOPIC[] = "lifter_target";
inline constexpr char LIFTER_FEEDBACK_TOPIC[] = "lifter_feedback";

}
