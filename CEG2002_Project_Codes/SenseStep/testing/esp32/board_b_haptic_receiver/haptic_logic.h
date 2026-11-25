#pragma once
#include <stdint.h>

namespace SenseStepHaptics {

enum class State : uint8_t { Waiting, Clear, Far, Near, Urgent, LinkFault };
constexpr uint32_t LINK_TIMEOUT_MS = 500;
constexpr uint16_t URGENT_MM = 450;
constexpr uint16_t NEAR_MM = 850;
constexpr uint16_t FAR_MM = 1300;
constexpr uint16_t RELEASE_MARGIN_MM = 75;

// Values are bench examples, not validated obstacle-warning thresholds.
// Escalate immediately; require extra clearance when reducing a warning.
constexpr State classifyDistance(uint16_t distanceMm, State previous) {
  const State candidate = distanceMm <= URGENT_MM ? State::Urgent
                        : distanceMm <= NEAR_MM ? State::Near
                        : distanceMm <= FAR_MM ? State::Far : State::Clear;
  if (previous == State::Urgent && distanceMm <= URGENT_MM + RELEASE_MARGIN_MM)
    return State::Urgent;
  if (candidate == State::Urgent) return candidate;
  if (previous == State::Near && distanceMm <= NEAR_MM + RELEASE_MARGIN_MM)
    return State::Near;
  if (candidate == State::Near) return candidate;
  if (previous == State::Far && distanceMm <= FAR_MM + RELEASE_MARGIN_MM)
    return State::Far;
  return candidate;
}

constexpr bool motorOn(State state, uint32_t elapsedMs) {
  if (state == State::Far) return elapsedMs % 1000 < 180;
  if (state == State::Near) return elapsedMs % 600 < 180;
  if (state == State::Urgent) return elapsedMs % 350 < 200;
  if (state == State::LinkFault) {
    const uint32_t phase = elapsedMs % 2000;
    return phase < 200 || (phase >= 400 && phase < 600);
  }
  return false;
}

// Wrap-aware, provided sequence jumps remain below half the uint32 range.
constexpr bool newerSequence(uint32_t candidate, uint32_t previous) {
  const uint32_t difference = candidate - previous;
  return difference != 0 && difference < 0x80000000u;
}

constexpr bool linkStale(uint32_t now, uint32_t lastFreshPacketAt) {
  return uint32_t(now - lastFreshPacketAt) >= LINK_TIMEOUT_MS;
}

} // namespace SenseStepHaptics
