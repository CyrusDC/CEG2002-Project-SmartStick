#pragma once
#include <stdint.h>

// Alfred (A): distance hysteresis, vibration timing and link freshness checks.
namespace SenseStepHaptics {

enum class State : uint8_t { Waiting, Clear, Far, Near, Urgent, LinkFault };  /* Define the haptic warning and idle states (A). */
constexpr uint32_t LINK_TIMEOUT_MS = 500;  /* Set the maximum age of a valid distance update (A). */
constexpr uint16_t URGENT_MM = 450;  /* Enter the urgent band at 450 mm or closer (A). */
constexpr uint16_t NEAR_MM = 850;  /* Enter the near band at 850 mm or closer (A). */
constexpr uint16_t FAR_MM = 1300;  /* Enter the far band at 1300 mm or closer (A). */
constexpr uint16_t RELEASE_MARGIN_MM = 75;  /* Require extra distance before reducing a warning (A). */

// Return the vibration band for distanceMm, using previous to apply hysteresis.
// Escalate immediately; require extra clearance when reducing a warning.
constexpr State classifyDistance(uint16_t distanceMm, State previous) {
  const State candidate = distanceMm <= URGENT_MM ? State::Urgent
                        : distanceMm <= NEAR_MM ? State::Near
                        : distanceMm <= FAR_MM ? State::Far : State::Clear;  /* Select the base band (A). */
  if (previous == State::Urgent && distanceMm <= URGENT_MM + RELEASE_MARGIN_MM)
    return State::Urgent;  /* Hold the urgent band until the release margin is crossed (A). */
  if (candidate == State::Urgent) return candidate;  /* Allow immediate escalation to urgent (A). */
  if (previous == State::Near && distanceMm <= NEAR_MM + RELEASE_MARGIN_MM)
    return State::Near;  /* Hold the near band until the release margin is crossed (A). */
  if (candidate == State::Near) return candidate;  /* Allow immediate escalation to near (A). */
  if (previous == State::Far && distanceMm <= FAR_MM + RELEASE_MARGIN_MM)
    return State::Far;  /* Hold the far band until the release margin is crossed (A). */
  return candidate;  /* Use the base band after checking the release margins (A). */
}

// Given state and elapsedMs since entering it, return whether the motor
// should be energized at this point in the repeating pulse pattern.
constexpr bool motorOn(State state, uint32_t elapsedMs) {
  if (state == State::Far) return elapsedMs % 1000 < 180;  /* Far: one short pulse per second (A). */
  if (state == State::Near) return elapsedMs % 600 < 180;  /* Near: shorten the repeat period (A). */
  if (state == State::Urgent) return elapsedMs % 350 < 200;  /* Urgent: repeat fastest (A). */
  if (state == State::LinkFault) {
    const uint32_t phase = elapsedMs % 2000;  /* Find the current position in the two-second fault pattern (A). */
    return phase < 200 || (phase >= 400 && phase < 600);  /* Fault pattern has two pulses (A). */
  }
  return false;  /* Leave the motor off in the clear and waiting states (A). */
}

// Return whether candidate follows previous, allowing uint32 wraparound.
// Sequence increments must remain below half the uint32 range.
constexpr bool newerSequence(uint32_t candidate, uint32_t previous) {
  const uint32_t difference = candidate - previous;  /* Calculate the unsigned sequence difference across rollover (A). */
  return difference != 0 && difference < 0x80000000u;  /* Reject duplicates and older values (A). */
}

// Given now and the lastFreshPacketAt timestamp in milliseconds, return
// whether the elapsed time has reached the link timeout.
constexpr bool linkStale(uint32_t now, uint32_t lastFreshPacketAt) {
  return uint32_t(now - lastFreshPacketAt) >= LINK_TIMEOUT_MS;  /* Compare elapsed time across wraparound (A). */
}

} // namespace SenseStepHaptics
